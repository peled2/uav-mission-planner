/**
 * UAV Mission Planner - Unified Web Worker
 *
 * Combines 4 Real-Time Engines over a single shared AWS Terrarium SRTM DEM Tile Cache:
 * 1. Dual 360° Viewshed (LOS) Engine:
 *    - Primary envelope at aircraft altitude (H_UAV)
 *    - Safety envelope at (H_UAV - H_LOS_Safety)
 *    - Analytical O(K) 60 FPS updates from cached horizon ridge steps
 * 2. Terrain Obstacle & Horizontal Safety Buffer Engine:
 *    - Continuous red contour line at threshold (H_UAV - H_Terrain_Clearance)
 *    - 2D Euclidean Distance Transform (EDT) + Marching Squares closed safety buffer polygon
 * 3. Terrain-Aware Emergency Glide Engine:
 *    - 360° Landing Field Catchment Envelope (backward terrain-constrained DP with wind layers & IAS/RoD polar)
 *    - 360° Aircraft Outward Glide Ring ("טבעת מטוס")
 * 4. Aircraft Point Safety & Direct Glide Trajectory Evaluator:
 *    - Ground elevation, AGL altitude, vertical & horizontal terrain clearance margins
 *    - Minimum LOS altitude at aircraft, height above LOS coverage, safety margin compliance
 *    - Direct terrain-aware glide simulation from Aircraft to Landing Field + cross-section profile
 */

import {
  EARTH_RADIUS_M,
  EARTH_RADIUS_KM,
  TILE_SIZE,
  FT_TO_M,
  M_TO_FT,
  NM_TO_KM,
  KM_TO_NM,
  toRadians,
  toDegrees,
  haversineDistanceKm,
  initialBearingDeg,
  lonLatToTileFloat,
  resolveGridMetrics,
  sampleTileBilinear,
  computeEuclideanDistanceField,
  extractIsoLines,
  isoLinesToLatLons,
  extractSafetyFillPolygons,
  linearInterpolate,
  interpolateAngle
} from './mission-math.js';

const DEG_TO_RAD = Math.PI / 180.0;
const RAD_TO_DEG = 180.0 / Math.PI;

const NEAR_LIMIT_M = 15000.0;
const NEAR_STEP_M = 60.0;
const FAR_STEP_M = 240.0;

// Shared LRU Tile Cache across all engines: key = `${z}/${x}/${y}` -> Float32Array(65536)
const MAX_CACHED_TILES = 360;
const tileCache = new Map();
const inFlightTiles = new Map();

// Fallback PNG decode queue on main thread (when OffscreenCanvas is unavailable)
const pendingMainDecodes = new Map();
let decodeReqId = 1;

// Job IDs for cancellation
let losJobId = 0;
let terrainJobId = 0;
let glideJobId = 0;

// ---------------------------------------------------------------------------
// 1. Shared AWS Terrarium DEM Tile Loader
// ---------------------------------------------------------------------------

function getTileFromCache(key) {
  const existing = tileCache.get(key);
  if (existing) {
    tileCache.delete(key);
    tileCache.set(key, existing);
  }
  return existing;
}

function putTileInCache(key, data) {
  if (tileCache.has(key)) {
    tileCache.delete(key);
  }
  tileCache.set(key, data);
  while (tileCache.size > MAX_CACHED_TILES) {
    const oldestKey = tileCache.keys().next().value;
    if (oldestKey !== undefined) {
      tileCache.delete(oldestKey);
    } else {
      break;
    }
  }
}

function decodeTerrariumRgba(rgbaData) {
  const elevations = new Float32Array(256 * 256);
  for (let i = 0, p = 0; i < 65536; i++, p += 4) {
    const r = rgbaData[p];
    const g = rgbaData[p + 1];
    const b = rgbaData[p + 2];
    const elev = r * 256.0 + g + b / 256.0 - 32768.0;
    elevations[i] = elev < -500.0 ? 0.0 : elev;
  }
  return elevations;
}

function requestMainThreadDecode(pngBuffer) {
  return new Promise((resolve) => {
    const id = decodeReqId++;
    pendingMainDecodes.set(id, resolve);
    self.postMessage({ type: 'DECODE_PNG_REQ', id, pngBuffer }, [pngBuffer]);
  });
}

async function loadTerrariumTile(z, x, y) {
  const maxTileIdx = (1 << z) - 1;
  if (y < 0 || y > maxTileIdx) return null;
  const wrappedX = ((x % (1 << z)) + (1 << z)) % (1 << z);
  const key = `${z}/${wrappedX}/${y}`;

  const cached = getTileFromCache(key);
  if (cached) return cached;

  if (inFlightTiles.has(key)) {
    return inFlightTiles.get(key);
  }

  const url = `https://s3.amazonaws.com/elevation-tiles-prod/terrarium/${z}/${wrappedX}/${y}.png`;

  const promise = (async () => {
    try {
      const resp = await fetch(url, { mode: 'cors', credentials: 'omit' });
      if (resp.status === 404) {
        const zero = new Float32Array(256 * 256);
        putTileInCache(key, zero);
        return zero;
      }
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const blob = await resp.blob();

      if (typeof OffscreenCanvas !== 'undefined' && typeof createImageBitmap !== 'undefined') {
        const bitmap = await createImageBitmap(blob);
        const canvas = new OffscreenCanvas(256, 256);
        const ctx = canvas.getContext('2d', { willReadFrequently: true });
        ctx.drawImage(bitmap, 0, 0, 256, 256);
        bitmap.close();
        const imgData = ctx.getImageData(0, 0, 256, 256).data;
        const elevations = decodeTerrariumRgba(imgData);
        putTileInCache(key, elevations);
        return elevations;
      } else {
        const arrayBuf = await blob.arrayBuffer();
        const rgbaBuf = await requestMainThreadDecode(arrayBuf);
        const elevations = decodeTerrariumRgba(new Uint8ClampedArray(rgbaBuf));
        putTileInCache(key, elevations);
        return elevations;
      }
    } catch (err) {
      const zero = new Float32Array(256 * 256);
      putTileInCache(key, zero);
      return zero;
    } finally {
      inFlightTiles.delete(key);
    }
  })();

  inFlightTiles.set(key, promise);
  return promise;
}

function lonToTileXFloat(lon, z) {
  return ((lon + 180.0) / 360.0) * (1 << z);
}

function latToTileYFloat(lat, z) {
  const sinLat = Math.sin(lat * DEG_TO_RAD);
  const clamped = Math.max(-0.9999, Math.min(0.9999, sinLat));
  const y = 0.5 - Math.log((1.0 + clamped) / (1.0 - clamped)) / (4.0 * Math.PI);
  return y * (1 << z);
}

function sampleElevationAtZoom(lat, lon, z) {
  const txFloat = lonToTileXFloat(lon, z);
  const tyFloat = latToTileYFloat(lat, z);
  const tx = Math.floor(txFloat);
  const ty = Math.floor(tyFloat);
  const tile = tileCache.get(`${z}/${tx}/${ty}`);
  if (!tile) return null;

  const px = Math.min(255, Math.max(0, Math.floor((txFloat - tx) * 256.0)));
  const py = Math.min(255, Math.max(0, Math.floor((tyFloat - ty) * 256.0)));
  return tile[py * 256 + px];
}

function sampleBilinearAtZoom(lat, lon, z) {
  const txFloat = lonToTileXFloat(lon, z);
  const tyFloat = latToTileYFloat(lat, z);
  const tx = Math.floor(txFloat);
  const ty = Math.floor(tyFloat);
  const tile = tileCache.get(`${z}/${tx}/${ty}`);
  if (!tile) return null;

  const fracX = (txFloat - tx) * 256.0 - 0.5;
  const fracY = (tyFloat - ty) * 256.0 - 0.5;
  return sampleTileBilinear(tile, fracX, fracY);
}

/**
 * Samples best available elevation (in meters) from cache across zooms 14 -> 12 -> 11 -> 10 -> 9,
 * fetching Zoom 11 if nothing is cached at that coordinate.
 */
async function ensureAndSampleElevationM(lat, lon) {
  for (const z of [14, 12, 11, 10, 9]) {
    const val = sampleBilinearAtZoom(lat, lon, z);
    if (val !== null) return val;
  }
  const tx = Math.floor(lonToTileXFloat(lon, 11));
  const ty = Math.floor(latToTileYFloat(lat, 11));
  await loadTerrariumTile(11, tx, ty);
  return sampleBilinearAtZoom(lat, lon, 11) ?? 0.0;
}

function sampleCachedBestElevationM(lat, lon) {
  for (const z of [12, 11, 10, 9]) {
    const val = sampleElevationAtZoom(lat, lon, z);
    if (val !== null) return val;
  }
  return 0.0;
}

function destinationLatLonFast(lat1Rad, lon1Rad, sinLat1, cosLat1, distM, bearingRad) {
  const angularDist = distM / EARTH_RADIUS_M;
  const sinAd = Math.sin(angularDist);
  const cosAd = Math.cos(angularDist);

  const sinLat2 = sinLat1 * cosAd + cosLat1 * sinAd * Math.cos(bearingRad);
  const lat2Rad = Math.asin(Math.max(-1.0, Math.min(1.0, sinLat2)));
  const y = Math.sin(bearingRad) * sinAd * cosLat1;
  const x = cosAd - sinLat1 * sinLat2;
  const lon2Rad = lon1Rad + Math.atan2(y, x);

  return [
    lat2Rad * RAD_TO_DEG,
    ((lon2Rad * RAD_TO_DEG + 540.0) % 360.0) - 180.0
  ];
}

// ---------------------------------------------------------------------------
// 2. Engine 1: Dual 360° Viewshed (LOS) Engine (Max Range + Safety Margin)
// ---------------------------------------------------------------------------

let stationState = {
  lat0: 31.898,
  lon0: 34.691,
  hGroundM: 32,
  hAntM: 15,
  H0: 47,
  kFactor: 4 / 3,
  Re: (4 / 3) * EARTH_RADIUS_M,
  maxLoadedRangeKm: 200
};

// Map from azimuthKey (Math.round(az * 4)) -> { azimuthDeg, ridges: Float32Array }
const cachedLosRaysMap = new Map();

const NEAR_STEPS_COUNT = Math.floor(NEAR_LIMIT_M / NEAR_STEP_M); // 250
const FAR_STEPS_COUNT = Math.ceil((200000.0 - NEAR_LIMIT_M) / FAR_STEP_M); // 771
const TOTAL_LOS_SAMPLES = NEAR_STEPS_COUNT + FAR_STEPS_COUNT; // 1021
const RAY_DIST_TABLE = new Float32Array(TOTAL_LOS_SAMPLES);
for (let i = 0; i < NEAR_STEPS_COUNT; i++) {
  RAY_DIST_TABLE[i] = (i + 1) * NEAR_STEP_M;
}
for (let i = 0; i < FAR_STEPS_COUNT; i++) {
  RAY_DIST_TABLE[NEAR_STEPS_COUNT + i] = NEAR_LIMIT_M + (i + 1) * FAR_STEP_M;
}

async function preloadTilesAroundPoint(lat0, lon0, nearRadiusKm, farRadiusKm, checkCancelFn) {
  const tasks = [];

  if (nearRadiusKm > 0) {
    const dLatNear = (nearRadiusKm / 6371.0) * RAD_TO_DEG;
    const dLonNear = dLatNear / Math.max(0.2, Math.cos(lat0 * DEG_TO_RAD));
    const z11MinX = Math.floor(lonToTileXFloat(lon0 - dLonNear, 11));
    const z11MaxX = Math.floor(lonToTileXFloat(lon0 + dLonNear, 11));
    const z11MinY = Math.floor(latToTileYFloat(lat0 + dLatNear, 11));
    const z11MaxY = Math.floor(latToTileYFloat(lat0 - dLatNear, 11));

    for (let x = z11MinX; x <= z11MaxX; x++) {
      for (let y = z11MinY; y <= z11MaxY; y++) {
        tasks.push({ z: 11, x, y });
      }
    }
  }

  if (farRadiusKm > 0) {
    const dLatFar = (farRadiusKm / 6371.0) * RAD_TO_DEG;
    const dLonFar = dLatFar / Math.max(0.2, Math.cos(lat0 * DEG_TO_RAD));
    const z9MinX = Math.floor(lonToTileXFloat(lon0 - dLonFar, 9));
    const z9MaxX = Math.floor(lonToTileXFloat(lon0 + dLonFar, 9));
    const z9MinY = Math.floor(latToTileYFloat(lat0 + dLatFar, 9));
    const z9MaxY = Math.floor(latToTileYFloat(lat0 - dLatFar, 9));

    for (let x = z9MinX; x <= z9MaxX; x++) {
      for (let y = z9MinY; y <= z9MaxY; y++) {
        tasks.push({ z: 9, x, y });
      }
    }
  }

  const uncached = tasks.filter((t) => {
    const wrappedX = ((t.x % (1 << t.z)) + (1 << t.z)) % (1 << t.z);
    return !tileCache.has(`${t.z}/${wrappedX}/${t.y}`);
  });

  const CONCURRENCY = 14;
  for (let i = 0; i < uncached.length; i += CONCURRENCY) {
    if (checkCancelFn && checkCancelFn()) return false;
    const batch = uncached.slice(i, i + CONCURRENCY);
    await Promise.all(batch.map((t) => loadTerrariumTile(t.z, t.x, t.y)));
  }
  return !(checkCancelFn && checkCancelFn());
}

function traceLosRayHorizonRidges(azimuthDeg, maxRangeM) {
  const { lat0, lon0, H0, Re } = stationState;
  const lat0Rad = lat0 * DEG_TO_RAD;
  const lon0Rad = lon0 * DEG_TO_RAD;
  const sinLat0 = Math.sin(lat0Rad);
  const cosLat0 = Math.cos(lat0Rad);
  const bearingRad = azimuthDeg * DEG_TO_RAD;
  const inv2Re = 1.0 / (2.0 * Re);

  let alphaMax = -Infinity;
  const ridgesList = [];

  for (let idx = 0; idx < NEAR_STEPS_COUNT; idx++) {
    const d = RAY_DIST_TABLE[idx];
    if (d > maxRangeM) break;

    const [lat, lon] = destinationLatLonFast(lat0Rad, lon0Rad, sinLat0, cosLat0, d, bearingRad);
    let h = sampleElevationAtZoom(lat, lon, 11);
    if (h === null) {
      h = sampleElevationAtZoom(lat, lon, 9) || 0.0;
    }

    const alpha = (h - H0) / d - d * inv2Re;
    if (alpha >= alphaMax) {
      alphaMax = alpha;
      ridgesList.push(d, h);
    }
  }

  if (maxRangeM > NEAR_LIMIT_M) {
    for (let idx = NEAR_STEPS_COUNT; idx < TOTAL_LOS_SAMPLES; idx++) {
      const d = RAY_DIST_TABLE[idx];
      if (d > maxRangeM) break;

      const [lat, lon] = destinationLatLonFast(lat0Rad, lon0Rad, sinLat0, cosLat0, d, bearingRad);
      const h = sampleElevationAtZoom(lat, lon, 9) || 0.0;

      const alpha = (h - H0) / d - d * inv2Re;
      if (alpha >= alphaMax) {
        alphaMax = alpha;
        ridgesList.push(d, h);
      }
    }
  }

  return new Float32Array(ridgesList);
}

function solveRayMaxRangeFromRidges(ridges, targetAltAslM, maxRangeM) {
  const { H0, Re } = stationState;
  const inv2Re = 1.0 / (2.0 * Re);
  const deltaH = targetAltAslM - H0;
  const twoReDeltaH = 2.0 * Re * deltaH;

  let rMax = maxRangeM;
  let blockingDistM = 0;
  let blockingHeightM = 0;

  const hSea = 0.0;
  const dSeaTangent = H0 > hSea ? Math.sqrt(2.0 * Re * (H0 - hSea)) : 60.0;
  if (dSeaTangent < rMax && targetAltAslM >= hSea) {
    const dAircraftFromTangent = Math.sqrt(2.0 * Re * Math.max(0.0, targetAltAslM - hSea));
    const smoothLimitM = dSeaTangent + dAircraftFromTangent;
    if (smoothLimitM < rMax) {
      rMax = smoothLimitM;
      blockingDistM = dSeaTangent;
      blockingHeightM = 0;
    }
  }

  const len = ridges.length;
  for (let i = 0; i < len; i += 2) {
    const dj = ridges[i];
    if (dj >= rMax) break;
    const hj = ridges[i + 1];
    const alphaJ = (hj - H0) / dj - dj * inv2Re;

    const uavAngleAtDj = deltaH / dj - dj * inv2Re;
    if (uavAngleAtDj <= alphaJ) {
      rMax = dj;
      blockingDistM = dj;
      blockingHeightM = hj;
      break;
    }

    const reAlpha = Re * alphaJ;
    const disc = reAlpha * reAlpha + twoReDeltaH;
    if (disc < 0.0) continue;
    const rShadow = -reAlpha + Math.sqrt(disc);

    const effectiveLimit = rShadow > dj ? rShadow : dj;
    if (effectiveLimit < rMax) {
      rMax = effectiveLimit;
      blockingDistM = dj;
      blockingHeightM = hj;
    }
  }

  const clampedR = Math.max(60.0, Math.min(maxRangeM, rMax));
  return {
    rMaxM: clampedR,
    blockingDistM,
    blockingHeightM
  };
}

function evaluateDualLosPolygons(hUavM, losSafetyMarginM, maxRangeKm, isFinal) {
  const maxRangeM = maxRangeKm * 1000.0;
  const hSafetyAltM = hUavM - losSafetyMarginM;
  const { lat0, lon0 } = stationState;
  const lat0Rad = lat0 * DEG_TO_RAD;
  const lon0Rad = lon0 * DEG_TO_RAD;
  const sinLat0 = Math.sin(lat0Rad);
  const cosLat0 = Math.cos(lat0Rad);

  const sortedKeys = Array.from(cachedLosRaysMap.keys()).sort((a, b) => a - b);
  const numRays = sortedKeys.length;

  const primaryPolygonLatLons = new Array(numRays);
  const safetyPolygonLatLons = new Array(numRays);
  const raySummaries = new Array(numRays);

  for (let i = 0; i < numRays; i++) {
    const rayObj = cachedLosRaysMap.get(sortedKeys[i]);
    const azDeg = rayObj.azimuthDeg;
    const azRad = azDeg * DEG_TO_RAD;

    const primaryRes = solveRayMaxRangeFromRidges(rayObj.ridges, hUavM, maxRangeM);
    const safetyRes =
      losSafetyMarginM > 0
        ? solveRayMaxRangeFromRidges(rayObj.ridges, hSafetyAltM, maxRangeM)
        : primaryRes;

    const [latP, lonP] = destinationLatLonFast(
      lat0Rad,
      lon0Rad,
      sinLat0,
      cosLat0,
      primaryRes.rMaxM,
      azRad
    );
    const [latS, lonS] = destinationLatLonFast(
      lat0Rad,
      lon0Rad,
      sinLat0,
      cosLat0,
      safetyRes.rMaxM,
      azRad
    );

    primaryPolygonLatLons[i] = [latP, lonP];
    safetyPolygonLatLons[i] = [latS, lonS];

    raySummaries[i] = {
      azimuthDeg: azDeg,
      primaryRangeKm: primaryRes.rMaxM / 1000.0,
      safetyRangeKm: safetyRes.rMaxM / 1000.0,
      blockingDistKm: primaryRes.blockingDistM / 1000.0,
      blockingHeightM: primaryRes.blockingHeightM
    };
  }

  return {
    isFinal,
    numRays,
    hGroundM: stationState.hGroundM,
    H0: stationState.H0,
    primaryPolygonLatLons,
    safetyPolygonLatLons,
    raySummaries
  };
}

async function runProgressiveLosViewshed(params) {
  const jobId = ++losJobId;
  const {
    lat0,
    lon0,
    hAntM,
    hUavM,
    losSafetyMarginM = 0,
    maxRangeKm = 200,
    kFactor = 4 / 3
  } = params;

  const ok = await preloadTilesAroundPoint(lat0, lon0, 16.5, 205, () => jobId !== losJobId);
  if (!ok || jobId !== losJobId) return;

  let hGroundM = sampleBilinearAtZoom(lat0, lon0, 11);
  if (hGroundM === null) {
    hGroundM = sampleBilinearAtZoom(lat0, lon0, 9) || 0.0;
  }
  hGroundM = Math.round(hGroundM);

  stationState = {
    lat0,
    lon0,
    hGroundM,
    hAntM,
    H0: hGroundM + hAntM,
    kFactor,
    Re: kFactor * EARTH_RADIUS_M,
    maxLoadedRangeKm: 200
  };

  cachedLosRaysMap.clear();
  const maxTraceM = 200000.0;

  // Stage 1: 360 rays (1° step)
  for (let i = 0; i < 360; i++) {
    if (jobId !== losJobId) return;
    const azDeg = i * 1.0;
    const key = Math.round(azDeg * 4);
    const ridges = traceLosRayHorizonRidges(azDeg, maxTraceM);
    cachedLosRaysMap.set(key, { azimuthDeg: azDeg, ridges });
  }

  const stage1 = evaluateDualLosPolygons(hUavM, losSafetyMarginM, maxRangeKm, false);
  self.postMessage({
    type: 'LOS_VIEWSHED_RESULT',
    jobId,
    ...stage1
  });

  await new Promise((r) => setTimeout(r, 10));
  if (jobId !== losJobId) return;

  // Stage 2: 720 rays (0.5° step) + adaptive 0.25° refinement
  for (let i = 0; i < 360; i++) {
    if (jobId !== losJobId) return;
    const azDeg = i + 0.5;
    const key = Math.round(azDeg * 4);
    const ridges = traceLosRayHorizonRidges(azDeg, maxTraceM);
    cachedLosRaysMap.set(key, { azimuthDeg: azDeg, ridges });
  }

  const maxRangeM = maxRangeKm * 1000.0;
  const keys720 = Array.from(cachedLosRaysMap.keys()).sort((a, b) => a - b);
  const ranges720 = keys720.map((k) =>
    solveRayMaxRangeFromRidges(cachedLosRaysMap.get(k).ridges, hUavM, maxRangeM).rMaxM
  );

  for (let i = 0; i < keys720.length; i++) {
    const nextIdx = (i + 1) % keys720.length;
    if (Math.abs(ranges720[nextIdx] - ranges720[i]) > 3500.0) {
      const azMid = keys720[i] / 4.0 + 0.25;
      const midKey = Math.round(azMid * 4);
      if (!cachedLosRaysMap.has(midKey) && midKey < 1440) {
        const ridges = traceLosRayHorizonRidges(azMid, maxTraceM);
        cachedLosRaysMap.set(midKey, { azimuthDeg: azMid, ridges });
      }
    }
  }

  if (jobId !== losJobId) return;
  const stage2 = evaluateDualLosPolygons(hUavM, losSafetyMarginM, maxRangeKm, true);
  self.postMessage({
    type: 'LOS_VIEWSHED_RESULT',
    jobId,
    ...stage2
  });
}

function buildLosRayCrossSectionProfile(azimuthDeg, hUavM, losSafetyMarginM, maxRangeKm) {
  const { lat0, lon0, hGroundM, H0, Re } = stationState;
  const lat0Rad = lat0 * DEG_TO_RAD;
  const lon0Rad = lon0 * DEG_TO_RAD;
  const sinLat0 = Math.sin(lat0Rad);
  const cosLat0 = Math.cos(lat0Rad);
  const normAz = ((azimuthDeg % 360) + 360) % 360;
  const bearingRad = normAz * DEG_TO_RAD;

  const maxDistM = maxRangeKm * 1000.0;
  const numSamples = 160;
  const stepM = maxDistM / numSamples;
  const inv2Re = 1.0 / (2.0 * Re);

  const nearestKey = Math.round(normAz * 2) * 2;
  const cachedRay = cachedLosRaysMap.get(nearestKey % 1440);
  const ridges = cachedRay ? cachedRay.ridges : traceLosRayHorizonRidges(normAz, maxDistM);

  const hSafetyAltM = hUavM - losSafetyMarginM;
  const primaryRes = solveRayMaxRangeFromRidges(ridges, hUavM, maxDistM);
  const safetyRes =
    losSafetyMarginM > 0
      ? solveRayMaxRangeFromRidges(ridges, hSafetyAltM, maxDistM)
      : primaryRes;

  const profileSamples = [];
  let runningAlphaMax = -Infinity;

  for (let i = 0; i <= numSamples; i++) {
    const dM = i * stepM;
    let hM = hGroundM;
    let minLosAslM = H0;

    if (i > 0) {
      const [lat, lon] = destinationLatLonFast(lat0Rad, lon0Rad, sinLat0, cosLat0, dM, bearingRad);
      hM =
        dM <= NEAR_LIMIT_M
          ? sampleElevationAtZoom(lat, lon, 11) ?? sampleElevationAtZoom(lat, lon, 9) ?? 0
          : sampleElevationAtZoom(lat, lon, 9) ?? 0;

      const alpha = (hM - H0) / dM - dM * inv2Re;
      if (alpha >= runningAlphaMax) {
        runningAlphaMax = alpha;
      }
      const rayH = H0 + dM * runningAlphaMax + dM * dM * inv2Re;
      minLosAslM = Math.max(hM, rayH);
    }

    const dropM = dM * dM * inv2Re;
    profileSamples.push({
      distanceKm: dM / 1000.0,
      terrainAslM: Math.round(hM),
      curvatureDropM: dropM,
      effectiveHeightM: hM - dropM,
      minLosAslM,
      isBlockedPrimary: dM > primaryRes.rMaxM,
      isBlockedSafety: dM > safetyRes.rMaxM
    });
  }

  let blockLat = null;
  let blockLon = null;
  if (primaryRes.blockingDistM > 0) {
    const [blat, blon] = destinationLatLonFast(
      lat0Rad,
      lon0Rad,
      sinLat0,
      cosLat0,
      primaryRes.blockingDistM,
      bearingRad
    );
    blockLat = blat;
    blockLon = blon;
  }

  return {
    azimuthDeg: normAz,
    maxRangeKm,
    primaryRangeKm: primaryRes.rMaxM / 1000.0,
    safetyRangeKm: safetyRes.rMaxM / 1000.0,
    blockingDistKm: primaryRes.blockingDistM / 1000.0,
    blockingHeightM: Math.round(primaryRes.blockingHeightM),
    blockLat,
    blockLon,
    H0,
    hUavM,
    hSafetyAltM,
    losSafetyMarginM,
    Re,
    profileSamples
  };
}

/**
 * Exact Minimum LOS Altitude at any target point (targetLat, targetLon)
 */
async function computePointMinLos(targetLat, targetLon) {
  const { lat0, lon0, H0, Re } = stationState;
  const totalDistKm = haversineDistanceKm(lat0, lon0, targetLat, targetLon);
  const totalDistM = totalDistKm * 1000.0;
  const azimuthDeg = initialBearingDeg(lat0, lon0, targetLat, targetLon);

  if (totalDistM < 30.0) {
    return {
      targetLat,
      targetLon,
      distanceKm: 0,
      azimuthDeg: 0,
      targetGroundM: stationState.hGroundM,
      minAslM: H0,
      minAglM: stationState.hAntM,
      blockingDistKm: 0,
      blockingHeightM: 0
    };
  }

  const lat0Rad = lat0 * DEG_TO_RAD;
  const lon0Rad = lon0 * DEG_TO_RAD;
  const sinLat0 = Math.sin(lat0Rad);
  const cosLat0 = Math.cos(lat0Rad);
  const bearingRad = azimuthDeg * DEG_TO_RAD;
  const inv2Re = 1.0 / (2.0 * Re);

  let alphaMax = -Infinity;
  let blockingDistM = 0;
  let blockingHeightM = 0;

  const nearLimit = Math.min(NEAR_LIMIT_M, totalDistM - 30.0);
  for (let d = NEAR_STEP_M; d <= nearLimit; d += NEAR_STEP_M) {
    const [lat, lon] = destinationLatLonFast(lat0Rad, lon0Rad, sinLat0, cosLat0, d, bearingRad);
    const h = sampleElevationAtZoom(lat, lon, 11) ?? sampleElevationAtZoom(lat, lon, 9) ?? 0.0;
    const alpha = (h - H0) / d - d * inv2Re;
    if (alpha > alphaMax) {
      alphaMax = alpha;
      blockingDistM = d;
      blockingHeightM = h;
    }
  }

  if (totalDistM > NEAR_LIMIT_M) {
    for (let d = NEAR_LIMIT_M + FAR_STEP_M; d < totalDistM; d += FAR_STEP_M) {
      const [lat, lon] = destinationLatLonFast(lat0Rad, lon0Rad, sinLat0, cosLat0, d, bearingRad);
      let h = sampleElevationAtZoom(lat, lon, 9);
      if (h === null) {
        const tx = Math.floor(lonToTileXFloat(lon, 9));
        const ty = Math.floor(latToTileYFloat(lat, 9));
        await loadTerrariumTile(9, tx, ty);
        h = sampleElevationAtZoom(lat, lon, 9) ?? 0.0;
      }
      const alpha = (h - H0) / d - d * inv2Re;
      if (alpha > alphaMax) {
        alphaMax = alpha;
        blockingDistM = d;
        blockingHeightM = h;
      }
    }
  }

  const targetGroundM = await ensureAndSampleElevationM(targetLat, targetLon);
  const rayHeightAtTarget =
    alphaMax > -Infinity
      ? H0 + totalDistM * alphaMax + totalDistM * totalDistM * inv2Re
      : H0 + totalDistM * totalDistM * inv2Re;

  const minAslM = Math.max(targetGroundM, rayHeightAtTarget);
  const minAglM = Math.max(0.0, minAslM - targetGroundM);

  return {
    targetLat,
    targetLon,
    distanceKm: totalDistKm,
    azimuthDeg,
    targetGroundM: Math.round(targetGroundM),
    minAslM: Math.round(minAslM),
    minAglM: Math.round(minAglM),
    blockingDistKm: blockingDistM / 1000.0,
    blockingHeightM: Math.round(blockingHeightM)
  };
}

// ---------------------------------------------------------------------------
// 3. Engine 2: Terrain Obstacle & Horizontal Safety Buffer (2D EDT + Contours)
// ---------------------------------------------------------------------------

let lastTerrainGridContext = null;

async function runTerrainSafetyCompute(req) {
  const jobId = ++terrainJobId;
  const tStart = performance.now();

  try {
    const metrics = resolveGridMetrics(req.fetchBounds, req.demZoom);
    const tileCoords = [];
    for (let ty = metrics.minTileY; ty <= metrics.maxTileY; ty++) {
      for (let tx = metrics.minTileX; tx <= metrics.maxTileX; tx++) {
        tileCoords.push({ z: metrics.demZoom, x: tx, y: ty });
      }
    }

    const CONCURRENCY = 10;
    for (let i = 0; i < tileCoords.length; i += CONCURRENCY) {
      if (jobId !== terrainJobId) return;
      const batch = tileCoords.slice(i, i + CONCURRENCY);
      await Promise.all(batch.map((t) => loadTerrariumTile(t.z, t.x, t.y)));
    }
    if (jobId !== terrainJobId) return;

    const { width, height, stride, minTileX, minTileY, demZoom } = metrics;
    const elevations = new Float32Array(width * height);
    const sampleOffset = Math.floor(stride / 2);

    for (let gy = 0; gy < height; gy++) {
      const globalPy = gy * stride + sampleOffset;
      const ty = minTileY + Math.floor(globalPy / TILE_SIZE);
      const localPy = globalPy % TILE_SIZE;
      const rowOut = gy * width;
      const rowTile = localPy * TILE_SIZE;

      for (let gx = 0; gx < width; gx++) {
        const globalPx = gx * stride + sampleOffset;
        const tx = minTileX + Math.floor(globalPx / TILE_SIZE);
        const localPx = globalPx % TILE_SIZE;
        const tile = tileCache.get(`${demZoom}/${tx}/${ty}`);
        elevations[rowOut + gx] = tile ? tile[rowTile + localPx] : 0;
      }
    }

    // Threshold altitude: H_UAV - H_Terrain_Vertical_Safety
    const thresholdFeet = req.hUavFt - req.terrainVerticalClearanceFt;
    const thresholdMeters = thresholdFeet * FT_TO_M;

    const contourPolylines = extractIsoLines(elevations, width, height, thresholdMeters, 0.38);
    const contourLinesLatLon = isoLinesToLatLons(contourPolylines, metrics);

    const { distPx, hasFeature } = computeEuclideanDistanceField(
      elevations,
      width,
      height,
      thresholdMeters
    );

    lastTerrainGridContext = {
      metrics,
      elevations,
      distPx,
      hasFeature,
      thresholdFeet,
      thresholdMeters
    };

    let safetyFillPolygonsLatLon = [];
    let safetyOutlineLatLon = [];

    if (hasFeature) {
      const rawRadiusPx = req.terrainHorizontalSafetyM / metrics.metersPerGridPixel;
      safetyFillPolygonsLatLon = extractSafetyFillPolygons(distPx, metrics, rawRadiusPx);

      if (req.terrainHorizontalSafetyM > 0) {
        const outlineThresholdPx = Math.max(0.25, rawRadiusPx);
        const outlineLines = extractIsoLines(distPx, width, height, outlineThresholdPx, 0.45);
        safetyOutlineLatLon = isoLinesToLatLons(outlineLines, metrics);
      }
    }

    if (jobId !== terrainJobId) return;

    self.postMessage({
      type: 'TERRAIN_SAFETY_RESULT',
      jobId: req.reqId || jobId,
      thresholdFeet,
      contourLinesLatLon,
      safetyFillPolygonsLatLon,
      safetyOutlineLatLon,
      resolutionMetersPerPx: metrics.metersPerGridPixel,
      computeTimeMs: Math.round(performance.now() - tStart),
      tileCount: tileCoords.length,
      hasIntersections: hasFeature
    });
  } catch (err) {
    if (jobId !== terrainJobId) return;
    self.postMessage({
      type: 'TERRAIN_SAFETY_RESULT',
      jobId: req.reqId || jobId,
      thresholdFeet: req.hUavFt - req.terrainVerticalClearanceFt,
      contourLinesLatLon: [],
      safetyFillPolygonsLatLon: [],
      safetyOutlineLatLon: [],
      resolutionMetersPerPx: 0,
      computeTimeMs: Math.round(performance.now() - tStart),
      tileCount: 0,
      hasIntersections: false,
      error: err instanceof Error ? err.message : 'שגיאה בחישוב מכשולי קרקע'
    });
  }
}

/**
 * Query exact horizontal distance (in meters) from (lat, lon) to the nearest terrain hazard cell
 * where elevation >= thresholdMeters.
 */
function queryHorizontalDistanceToHazardM(lat, lon, thresholdMeters) {
  // If (lat, lon) lies inside the active 2D EDT grid with matching threshold, read directly in O(1)!
  if (
    lastTerrainGridContext &&
    Math.abs(lastTerrainGridContext.thresholdMeters - thresholdMeters) < 0.5
  ) {
    const { metrics, distPx, hasFeature } = lastTerrainGridContext;
    if (!hasFeature) return Infinity;
    const [txFloat, tyFloat] = lonLatToTileFloat(lon, lat, metrics.demZoom);
    const gx = Math.floor(((txFloat - metrics.minTileX) * TILE_SIZE) / metrics.stride);
    const gy = Math.floor(((tyFloat - metrics.minTileY) * TILE_SIZE) / metrics.stride);
    if (gx >= 0 && gx < metrics.width && gy >= 0 && gy < metrics.height) {
      return distPx[gy * metrics.width + gx] * metrics.metersPerGridPixel;
    }
  }

  // Fallback local radial scan up to 15 km using cached Zoom 11 / Zoom 9 tiles
  const centerH = sampleCachedBestElevationM(lat, lon);
  if (centerH >= thresholdMeters) return 0.0;

  const latRad = lat * DEG_TO_RAD;
  const lonRad = lon * DEG_TO_RAD;
  const sinLat = Math.sin(latRad);
  const cosLat = Math.cos(latRad);

  for (let dM = 100.0; dM <= 15000.0; dM += 150.0) {
    for (let az = 0; az < 360; az += 10) {
      const [pLat, pLon] = destinationLatLonFast(
        latRad,
        lonRad,
        sinLat,
        cosLat,
        dM,
        az * DEG_TO_RAD
      );
      const h = sampleCachedBestElevationM(pLat, pLon);
      if (h >= thresholdMeters) {
        return dM;
      }
    }
  }
  return Infinity;
}

// ---------------------------------------------------------------------------
// 4. Engine 3: Terrain-Aware Emergency Glide Engine (Landing Field + UAV Ring)
// ---------------------------------------------------------------------------

const GLIDE_NUM_RAYS = 180; // Every 2° (0, 2, 4, ..., 358)
const GLIDE_STEP_KM = 0.25; // 250 m radial step
const GLIDE_MAX_DIST_KM = 120.0; // Up to 120 km glide range
const GLIDE_STEPS_PER_RAY = Math.floor(GLIDE_MAX_DIST_KM / GLIDE_STEP_KM); // 480 steps

let lfCachedOrigin = { lat: null, lon: null, groundFt: 0, radialTerrainFt: null };
let uavCachedOrigin = { lat: null, lon: null, groundFt: 0, radialTerrainFt: null };

async function buildRadialTerrainGridFt(centerLat, centerLon, checkCancelFn) {
  await preloadTilesAroundPoint(centerLat, centerLon, 16.0, 125.0, checkCancelFn);
  if (checkCancelFn && checkCancelFn()) return null;

  const centerGroundM = await ensureAndSampleElevationM(centerLat, centerLon);
  const centerGroundFt = Math.round(centerGroundM * M_TO_FT);

  const radialTerrainFt = new Float32Array(GLIDE_NUM_RAYS * (GLIDE_STEPS_PER_RAY + 1));
  const latRad = centerLat * DEG_TO_RAD;
  const lonRad = centerLon * DEG_TO_RAD;
  const sinLat = Math.sin(latRad);
  const cosLat = Math.cos(latRad);

  for (let r = 0; r < GLIDE_NUM_RAYS; r++) {
    const azDeg = r * 2.0;
    const brngRad = azDeg * DEG_TO_RAD;
    const rowOffset = r * (GLIDE_STEPS_PER_RAY + 1);
    radialTerrainFt[rowOffset] = centerGroundFt;

    for (let k = 1; k <= GLIDE_STEPS_PER_RAY; k++) {
      const distM = k * GLIDE_STEP_KM * 1000.0;
      const [pLat, pLon] = destinationLatLonFast(latRad, lonRad, sinLat, cosLat, distM, brngRad);
      const hM =
        distM <= NEAR_LIMIT_M
          ? sampleElevationAtZoom(pLat, pLon, 11) ?? sampleElevationAtZoom(pLat, pLon, 9) ?? 0.0
          : sampleElevationAtZoom(pLat, pLon, 9) ?? 0.0;
      radialTerrainFt[rowOffset + k] = hM * M_TO_FT;
    }
  }

  return {
    groundFt: centerGroundFt,
    radialTerrainFt
  };
}

/**
 * Precompute candidate airspeed + RoD table and altitude wind table (0..40,000 ft in 100 ft steps)
 * for ultra-fast O(1) inner-loop lookups during 360° terrain-aware glide evaluation.
 */
function prepareGlideLookupTables(glideSpeedKts, glideRodFpm, windLayers) {
  const speed = Number.isFinite(glideSpeedKts) && glideSpeedKts > 0 ? glideSpeedKts : 60;
  const rod = Number.isFinite(glideRodFpm) && glideRodFpm > 0 ? glideRodFpm : 500;
  const candidateSpeeds = [{ speedKts: speed, rodFpm: rod }];

  const validWinds = (windLayers || [])
    .filter(
      (w) =>
        Number.isFinite(w.altitudeKft) &&
        Number.isFinite(w.speed) &&
        Number.isFinite(w.direction) &&
        w.altitudeKft >= 0 &&
        w.speed >= 0
    )
    .sort((a, b) => a.altitudeKft - b.altitudeKft);

  if (validWinds.length === 0) {
    validWinds.push({ altitudeKft: 0, speed: 0, direction: 0 });
  }

  // Precompute wind (windVx, windVy) in mathematical coordinates at every 100 ft from 0 to 45,000 ft (451 buckets)
  // Using exact linear interpolation between wind layers per 1,000 ft
  const MAX_ALT_BUCKETS = 451;
  const windVxTable = new Float32Array(MAX_ALT_BUCKETS);
  const windVyTable = new Float32Array(MAX_ALT_BUCKETS);

  for (let b = 0; b < MAX_ALT_BUCKETS; b++) {
    const altKft = (b * 100.0) / 1000.0;
    let wSpeed = 0;
    let wDirFrom = 0;

    if (validWinds.length === 1) {
      wSpeed = validWinds[0].speed;
      wDirFrom = validWinds[0].direction;
    } else if (altKft <= validWinds[0].altitudeKft) {
      wSpeed = validWinds[0].speed;
      wDirFrom = validWinds[0].direction;
    } else if (altKft >= validWinds[validWinds.length - 1].altitudeKft) {
      wSpeed = validWinds[validWinds.length - 1].speed;
      wDirFrom = validWinds[validWinds.length - 1].direction;
    } else {
      for (let k = 0; k < validWinds.length - 1; k++) {
        if (altKft >= validWinds[k].altitudeKft && altKft <= validWinds[k + 1].altitudeKft) {
          wSpeed = linearInterpolate(
            altKft,
            validWinds[k].altitudeKft,
            validWinds[k].speed,
            validWinds[k + 1].altitudeKft,
            validWinds[k + 1].speed
          );
          wDirFrom = interpolateAngle(
            altKft,
            validWinds[k].altitudeKft,
            validWinds[k].direction,
            validWinds[k + 1].altitudeKft,
            validWinds[k + 1].direction
          );
          break;
        }
      }
    }

    const windGoesTo = (wDirFrom + 180.0) % 360.0;
    const windAngleRad = ((90.0 - windGoesTo + 360.0) % 360.0) * DEG_TO_RAD;
    windVxTable[b] = wSpeed * Math.cos(windAngleRad);
    windVyTable[b] = wSpeed * Math.sin(windAngleRad);
  }

  return { candidateSpeeds, windVxTable, windVyTable };
}

function getGroundSpeedKnotsFast(
  airspeedKts,
  flightUnitCos,
  flightUnitSin,
  altitudeFt,
  windVxTable,
  windVyTable
) {
  const bucket = Math.max(0, Math.min(450, Math.round(altitudeFt / 100.0)));
  const wVx = windVxTable[bucket];
  const wVy = windVyTable[bucket];

  const airVx = airspeedKts * flightUnitCos;
  const airVy = airspeedKts * flightUnitSin;
  const gVx = airVx + wVx;
  const gVy = airVy + wVy;

  // Ensure aircraft makes positive forward progress along flight direction
  const forwardDot = gVx * flightUnitCos + gVy * flightUnitSin;
  if (forwardDot <= 0.5) return 0.0;

  return Math.sqrt(gVx * gVx + gVy * gVy);
}

/**
 * Evaluate 360° Terrain-Aware Landing Field Catchment Polygon:
 * Along each azimuth `phi` from the Landing Field, the aircraft flies towards the field
 * in heading `flightDir = (phi + 180) % 360`.
 * Uses backward terrain-constrained dynamic programming from the Landing Field (d=0) outward.
 */
function computeLandingFieldCatchmentPolygon(params, tables) {
  const {
    latLf,
    lonLf,
    hUavFt,
    lfSafetyMarginFt,
    ridgeClearanceFt,
    batteryTimeMin,
    postArrivalDescentMin = 0
  } = params;

  const { groundFt, radialTerrainFt } = lfCachedOrigin;
  if (!radialTerrainFt) return { polygonLatLons: [], rayDetails: [], lfGroundFt: groundFt };

  const arrivalTargetFt = groundFt + lfSafetyMarginFt;
  const effectiveGlideBatteryMin = batteryTimeMin - postArrivalDescentMin;
  const stepNm = GLIDE_STEP_KM * KM_TO_NM;
  const latRad = latLf * DEG_TO_RAD;
  const lonRad = lonLf * DEG_TO_RAD;
  const sinLat = Math.sin(latRad);
  const cosLat = Math.cos(latRad);

  const polygonLatLons = new Array(GLIDE_NUM_RAYS);
  const rayDetails = new Array(GLIDE_NUM_RAYS);

  const { candidateSpeeds, windVxTable, windVyTable } = tables;

  for (let r = 0; r < GLIDE_NUM_RAYS; r++) {
    const azimuthFromLfDeg = r * 2.0;
    const flightDirToLfDeg = (azimuthFromLfDeg + 180.0) % 360.0;
    const flightMathRad = ((90.0 - flightDirToLfDeg + 360.0) % 360.0) * DEG_TO_RAD;
    const fCos = Math.cos(flightMathRad);
    const fSin = Math.sin(flightMathRad);
    const rowOffset = r * (GLIDE_STEPS_PER_RAY + 1);

    let bestDistKm = 0.05;
    let bestSpeedKts = candidateSpeeds[0].speedKts;
    let bestTimeMin = 0;
    let limitedByTerrain = false;

    if (hUavFt > arrivalTargetFt && effectiveGlideBatteryMin > 0) {
      for (let sIdx = 0; sIdx < candidateSpeeds.length; sIdx++) {
        const { speedKts, rodFpm } = candidateSpeeds[sIdx];
        let reqAltFt = arrivalTargetFt;
        let elapsedMin = 0.0;
        let reachedKm = 0.0;
        let terrainHit = false;

        for (let k = 1; k <= GLIDE_STEPS_PER_RAY; k++) {
          const terrFt = radialTerrainFt[rowOffset + k];
          const ridgeFloorFt = terrFt + ridgeClearanceFt;

          // If the terrain ridge itself + emergency clearance is above H_UAV, aircraft cannot cross this point!
          if (ridgeFloorFt > hUavFt) {
            terrainHit = true;
            break;
          }

          const gsKts = getGroundSpeedKnotsFast(
            speedKts,
            fCos,
            fSin,
            reqAltFt,
            windVxTable,
            windVyTable
          );
          if (gsKts <= 0.5) break;

          const dtMin = (stepNm / gsKts) * 60.0;
          const dhFt = rodFpm * dtMin;

          if (reqAltFt + dhFt > hUavFt || elapsedMin + dtMin > effectiveGlideBatteryMin) {
            const fracAlt = dhFt > 0 ? (hUavFt - reqAltFt) / dhFt : 1.0;
            const fracTime = dtMin > 0 ? (effectiveGlideBatteryMin - elapsedMin) / dtMin : 1.0;
            const frac = Math.max(0.0, Math.min(1.0, Math.min(fracAlt, fracTime)));
            reachedKm += frac * GLIDE_STEP_KM;
            elapsedMin += frac * dtMin;
            break;
          }

          reqAltFt = Math.max(reqAltFt + dhFt, ridgeFloorFt);
          elapsedMin += dtMin;
          reachedKm = k * GLIDE_STEP_KM;
        }

        if (reachedKm > bestDistKm) {
          bestDistKm = reachedKm;
          bestSpeedKts = speedKts;
          bestTimeMin = elapsedMin;
          limitedByTerrain = terrainHit;
        }
      }
    }

    const [pLat, pLon] = destinationLatLonFast(
      latRad,
      lonRad,
      sinLat,
      cosLat,
      bestDistKm * 1000.0,
      azimuthFromLfDeg * DEG_TO_RAD
    );
    polygonLatLons[r] = [pLat, pLon];
    rayDetails[r] = {
      azimuthFromLfDeg,
      flightDirToLfDeg,
      distanceKm: bestDistKm,
      optimalSpeedKts: bestSpeedKts,
      timeMin: bestTimeMin,
      limitedByTerrain
    };
  }

  return {
    lfGroundFt: groundFt,
    polygonLatLons,
    rayDetails
  };
}

/**
 * Evaluate 360° Outward Aircraft Glide Ring ("טבעת מטוס"):
 * Along each azimuth `theta` outward from the Aircraft (latUav, lonUav), traces forward from H_UAV
 * checking intermediate terrain clearance (terrain + ridgeClearanceFt), arrival floor
 * (max(lfArrivalTargetFt, terrain + ridgeClearanceFt)), and effective battery time (batteryTimeMin - postArrivalDescentMin).
 */
function computeAircraftOutwardGlideRing(params, tables) {
  const {
    latUav,
    lonUav,
    hUavFt,
    lfSafetyMarginFt,
    ridgeClearanceFt,
    batteryTimeMin,
    postArrivalDescentMin = 0
  } = params;

  const { groundFt: uavGroundFt, radialTerrainFt } = uavCachedOrigin;
  if (!radialTerrainFt) return { polygonLatLons: [], uavGroundFt };

  const lfGroundFt = lfCachedOrigin.groundFt || 0;
  const lfArrivalTargetFt = lfGroundFt + lfSafetyMarginFt;
  const effectiveGlideBatteryMin = batteryTimeMin - postArrivalDescentMin;

  const stepNm = GLIDE_STEP_KM * KM_TO_NM;
  const latRad = latUav * DEG_TO_RAD;
  const lonRad = lonUav * DEG_TO_RAD;
  const sinLat = Math.sin(latRad);
  const cosLat = Math.cos(latRad);

  const polygonLatLons = new Array(GLIDE_NUM_RAYS);
  const { candidateSpeeds, windVxTable, windVyTable } = tables;

  for (let r = 0; r < GLIDE_NUM_RAYS; r++) {
    const flightDirDeg = r * 2.0;
    const flightMathRad = ((90.0 - flightDirDeg + 360.0) % 360.0) * DEG_TO_RAD;
    const fCos = Math.cos(flightMathRad);
    const fSin = Math.sin(flightMathRad);
    const rowOffset = r * (GLIDE_STEPS_PER_RAY + 1);

    let bestDistKm = 0.05;

    if (effectiveGlideBatteryMin > 0 && hUavFt > uavGroundFt + ridgeClearanceFt) {
      for (let sIdx = 0; sIdx < candidateSpeeds.length; sIdx++) {
        const { speedKts, rodFpm } = candidateSpeeds[sIdx];
        let currAltFt = hUavFt;
        let elapsedMin = 0.0;
        let maxValidArrivalKm = 0.0;

        for (let k = 1; k <= GLIDE_STEPS_PER_RAY; k++) {
          const gsKts = getGroundSpeedKnotsFast(
            speedKts,
            fCos,
            fSin,
            currAltFt,
            windVxTable,
            windVyTable
          );
          if (gsKts <= 0.5) break;

          const dtMin = (stepNm / gsKts) * 60.0;
          const dhFt = rodFpm * dtMin;
          const nextAltFt = currAltFt - dhFt;
          const nextTimeMin = elapsedMin + dtMin;

          const terrFt = radialTerrainFt[rowOffset + k];
          const ridgeFloorFt = terrFt + ridgeClearanceFt;
          const arrivalFloorFt = Math.max(lfArrivalTargetFt, ridgeFloorFt);

          if (nextTimeMin > effectiveGlideBatteryMin || nextAltFt < ridgeFloorFt) {
            if (currAltFt >= arrivalFloorFt) {
              const fracAlt = dhFt > 0 ? Math.max(0, (currAltFt - arrivalFloorFt) / dhFt) : 0;
              const fracTime = dtMin > 0 ? Math.max(0, (effectiveGlideBatteryMin - elapsedMin) / dtMin) : 0;
              const frac = Math.min(1.0, Math.min(fracAlt, fracTime));
              const candidateKm = (k - 1 + frac) * GLIDE_STEP_KM;
              if (candidateKm > maxValidArrivalKm) maxValidArrivalKm = candidateKm;
            }
            break;
          }

          currAltFt = nextAltFt;
          elapsedMin = nextTimeMin;

          if (currAltFt >= arrivalFloorFt) {
            maxValidArrivalKm = k * GLIDE_STEP_KM;
          }
        }

        if (maxValidArrivalKm > bestDistKm) {
          bestDistKm = maxValidArrivalKm;
        }
      }
    }

    const [pLat, pLon] = destinationLatLonFast(
      latRad,
      lonRad,
      sinLat,
      cosLat,
      bestDistKm * 1000.0,
      flightDirDeg * DEG_TO_RAD
    );
    polygonLatLons[r] = [pLat, pLon];
  }

  return {
    uavGroundFt,
    polygonLatLons
  };
}

/**
 * Direct Terrain-Aware Glide Path Simulation from Aircraft (latUav, lonUav) to Landing Field (latLf, lonLf)
 * Builds dense cross-section profile + exact pass/fail diagnostics including post-arrival descent battery check.
 */
async function evaluateDirectGlideToLandingField(params, tables) {
  const {
    latUav,
    lonUav,
    latLf,
    lonLf,
    hUavFt,
    lfSafetyMarginFt,
    ridgeClearanceFt,
    batteryTimeMin,
    postArrivalDescentMin = 0
  } = params;

  const totalDistKm = haversineDistanceKm(latUav, lonUav, latLf, lonLf);
  const bearingToLfDeg = initialBearingDeg(latUav, lonUav, latLf, lonLf);
  const azimuthFromLfDeg = initialBearingDeg(latLf, lonLf, latUav, lonUav);

  const uavGroundM = await ensureAndSampleElevationM(latUav, lonUav);
  const lfGroundM = await ensureAndSampleElevationM(latLf, lonLf);
  const uavGroundFt = Math.round(uavGroundM * M_TO_FT);
  const lfGroundFt = Math.round(lfGroundM * M_TO_FT);
  const arrivalTargetFt = lfGroundFt + lfSafetyMarginFt;
  const effectiveGlideBatteryMin = batteryTimeMin - postArrivalDescentMin;

  // Sample terrain along the direct path from Aircraft (d=0) to Landing Field (d=totalDistKm)
  const numSteps = Math.max(40, Math.min(240, Math.ceil(totalDistKm / 0.2)));
  const stepKm = totalDistKm > 0.05 ? totalDistKm / numSteps : 0.05;
  const stepNm = stepKm * KM_TO_NM;

  const lat1Rad = latUav * DEG_TO_RAD;
  const lon1Rad = lonUav * DEG_TO_RAD;
  const sinLat1 = Math.sin(lat1Rad);
  const cosLat1 = Math.cos(lat1Rad);
  const brngRad = bearingToLfDeg * DEG_TO_RAD;

  const pathTerrainFt = new Float32Array(numSteps + 1);
  const pathLats = new Float64Array(numSteps + 1);
  const pathLons = new Float64Array(numSteps + 1);

  pathTerrainFt[0] = uavGroundFt;
  pathLats[0] = latUav;
  pathLons[0] = lonUav;

  for (let i = 1; i <= numSteps; i++) {
    const dM = i * stepKm * 1000.0;
    const [pLat, pLon] = destinationLatLonFast(lat1Rad, lon1Rad, sinLat1, cosLat1, dM, brngRad);
    pathLats[i] = pLat;
    pathLons[i] = pLon;
    if (i === numSteps) {
      pathTerrainFt[i] = lfGroundFt;
    } else {
      const hM = sampleCachedBestElevationM(pLat, pLon);
      pathTerrainFt[i] = Math.round(hM * M_TO_FT);
    }
  }

  const flightMathRad = ((90.0 - bearingToLfDeg + 360.0) % 360.0) * DEG_TO_RAD;
  const fCos = Math.cos(flightMathRad);
  const fSin = Math.sin(flightMathRad);
  const { candidateSpeeds, windVxTable, windVyTable } = tables;

  let bestCandidate = null;

  for (let sIdx = 0; sIdx < candidateSpeeds.length; sIdx++) {
    const { speedKts, rodFpm } = candidateSpeeds[sIdx];
    let currAltFt = hUavFt;
    let elapsedMin = 0.0;
    let minRidgeMarginFt = hUavFt - (uavGroundFt + ridgeClearanceFt);
    let criticalRidgeIdx = 0;
    let hitRidgeIdx = -1;
    let batteryExpiredIdx = -1;
    const altProfileFt = new Float32Array(numSteps + 1);
    altProfileFt[0] = hUavFt;

    for (let i = 1; i <= numSteps; i++) {
      if (totalDistKm <= 0.05) {
        altProfileFt[i] = hUavFt;
        continue;
      }

      const gsKts = getGroundSpeedKnotsFast(
        speedKts,
        fCos,
        fSin,
        currAltFt,
        windVxTable,
        windVyTable
      );
      if (gsKts <= 0.5) {
        hitRidgeIdx = i;
        altProfileFt[i] = currAltFt;
        break;
      }

      const dtMin = (stepNm / gsKts) * 60.0;
      const dhFt = rodFpm * dtMin;
      currAltFt -= dhFt;
      elapsedMin += dtMin;
      altProfileFt[i] = currAltFt;

      const requiredFloorFt =
        i === numSteps ? arrivalTargetFt : pathTerrainFt[i] + ridgeClearanceFt;
      const marginFt = currAltFt - requiredFloorFt;

      if (i < numSteps && marginFt < minRidgeMarginFt) {
        minRidgeMarginFt = marginFt;
        criticalRidgeIdx = i;
      }

      if (i < numSteps && currAltFt < pathTerrainFt[i] + ridgeClearanceFt && hitRidgeIdx === -1) {
        hitRidgeIdx = i;
      }
      if (elapsedMin > effectiveGlideBatteryMin && batteryExpiredIdx === -1) {
        batteryExpiredIdx = i;
      }
    }

    const arrivalAltFt = altProfileFt[numSteps];
    const arrivalMarginFt = arrivalAltFt - arrivalTargetFt;
    const totalMissionTimeMin = elapsedMin + postArrivalDescentMin;
    const hasElectricalPotential = totalMissionTimeMin <= batteryTimeMin && effectiveGlideBatteryMin > 0;

    const isSuccess =
      hitRidgeIdx === -1 &&
      batteryExpiredIdx === -1 &&
      hasElectricalPotential &&
      arrivalMarginFt >= 0 &&
      hUavFt >= uavGroundFt + ridgeClearanceFt;

    const bottleneckMargin = Math.min(arrivalMarginFt, minRidgeMarginFt);
    if (
      !bestCandidate ||
      (isSuccess && !bestCandidate.isSuccess) ||
      (isSuccess === bestCandidate.isSuccess && bottleneckMargin > bestCandidate.bottleneckMargin)
    ) {
      bestCandidate = {
        speedKts,
        rodFpm,
        elapsedMin,
        totalMissionTimeMin,
        hasElectricalPotential,
        arrivalAltFt,
        arrivalMarginFt,
        minRidgeMarginFt,
        criticalRidgeIdx,
        hitRidgeIdx,
        batteryExpiredIdx,
        isSuccess,
        bottleneckMargin,
        altProfileFt
      };
    }
  }

  let maxGlideTowardsLfKm = 0;
  if (lfCachedOrigin.lastRayDetails && lfCachedOrigin.lastRayDetails.length) {
    const rayIdx = Math.round(azimuthFromLfDeg / 2.0) % GLIDE_NUM_RAYS;
    maxGlideTowardsLfKm = lfCachedOrigin.lastRayDetails[rayIdx]?.distanceKm || 0;
  }

  let statusCode = 'SAFE';
  if (hUavFt <= arrivalTargetFt) {
    statusCode = 'TOO_LOW_FOR_FIELD';
  } else if (!bestCandidate.isSuccess) {
    if (bestCandidate.hitRidgeIdx !== -1) {
      statusCode = 'BLOCKED_BY_RIDGE';
    } else if (!bestCandidate.hasElectricalPotential || bestCandidate.batteryExpiredIdx !== -1) {
      statusCode = 'BATTERY_LIMIT';
    } else {
      statusCode = 'OUT_OF_RANGE';
    }
  }

  const critIdx =
    bestCandidate.hitRidgeIdx !== -1
      ? bestCandidate.hitRidgeIdx
      : bestCandidate.criticalRidgeIdx;

  const profileSamples = [];
  for (let i = 0; i <= numSteps; i++) {
    const dKm = i * stepKm;
    const terrFt = pathTerrainFt[i];
    const reqFloorFt = i === numSteps ? arrivalTargetFt : terrFt + ridgeClearanceFt;
    const glideAltFt = bestCandidate.altProfileFt[i];
    profileSamples.push({
      distanceKm: dKm,
      terrainFt: Math.round(terrFt),
      requiredFloorFt: Math.round(reqFloorFt),
      glideAltFt: Math.round(glideAltFt),
      isBlocked: glideAltFt < reqFloorFt
    });
  }

  return {
    canGlideToLf: bestCandidate.isSuccess,
    hasElectricalPotential: bestCandidate.hasElectricalPotential,
    statusCode,
    distToLfKm: totalDistKm,
    bearingToLfDeg,
    azimuthFromLfDeg,
    maxGlideTowardsLfKm,
    optimalSpeedKts: bestCandidate.speedKts,
    rodFpm: bestCandidate.rodFpm,
    timeToLfMin: bestCandidate.elapsedMin,
    postArrivalDescentMin,
    totalMissionTimeMin: bestCandidate.totalMissionTimeMin,
    batteryTimeMin,
    arrivalAltFt: Math.round(bestCandidate.arrivalAltFt),
    arrivalTargetFt: Math.round(arrivalTargetFt),
    arrivalMarginFt: Math.round(bestCandidate.arrivalMarginFt),
    minRidgeMarginFt: Math.round(bestCandidate.minRidgeMarginFt),
    criticalRidgeDistKm: critIdx * stepKm,
    criticalRidgeHeightFt: Math.round(pathTerrainFt[critIdx]),
    criticalRidgeLat: pathLats[critIdx],
    criticalRidgeLon: pathLons[critIdx],
    uavGroundFt,
    lfGroundFt,
    hUavFt,
    lfSafetyMarginFt,
    ridgeClearanceFt,
    profileSamples
  };
}

// ---------------------------------------------------------------------------
// 5. Full Glide + Aircraft Status Orchestrator
// ---------------------------------------------------------------------------

async function runGlideAndAircraftEvaluation(params) {
  const jobId = ++glideJobId;
  const {
    latLf,
    lonLf,
    latUav,
    lonUav,
    hUavFt,
    lfSafetyMarginFt,
    ridgeClearanceFt,
    batteryTimeMin,
    postArrivalDescentMin = 0,
    glideSpeedKts = 60,
    glideRodFpm = 500,
    windLayers,
    showUavGlideRing,
    terrainVerticalClearanceFt,
    terrainHorizontalSafetyM,
    losSafetyMarginFt
  } = params;

  // 1. Rebuild Landing Field radial terrain cache ONLY if Landing Field moved
  if (
    lfCachedOrigin.lat !== latLf ||
    lfCachedOrigin.lon !== lonLf ||
    !lfCachedOrigin.radialTerrainFt
  ) {
    const built = await buildRadialTerrainGridFt(latLf, lonLf, () => jobId !== glideJobId);
    if (!built || jobId !== glideJobId) return;
    lfCachedOrigin = {
      lat: latLf,
      lon: lonLf,
      groundFt: built.groundFt,
      radialTerrainFt: built.radialTerrainFt,
      lastRayDetails: []
    };
  }

  // 2. Rebuild Aircraft radial terrain cache ONLY if Aircraft moved AND showUavGlideRing is active
  if (
    showUavGlideRing &&
    (uavCachedOrigin.lat !== latUav ||
      uavCachedOrigin.lon !== lonUav ||
      !uavCachedOrigin.radialTerrainFt)
  ) {
    const builtUav = await buildRadialTerrainGridFt(latUav, lonUav, () => jobId !== glideJobId);
    if (!builtUav || jobId !== glideJobId) return;
    uavCachedOrigin = {
      lat: latUav,
      lon: lonUav,
      groundFt: builtUav.groundFt,
      radialTerrainFt: builtUav.radialTerrainFt
    };
  }

  const tables = prepareGlideLookupTables(glideSpeedKts, glideRodFpm, windLayers);

  // 3. Compute Landing Field 360° Terrain-Aware Catchment Envelope
  const lfEnvelope = computeLandingFieldCatchmentPolygon(params, tables);
  lfCachedOrigin.lastRayDetails = lfEnvelope.rayDetails;

  // 4. Compute Aircraft 360° Outward Glide Ring (if toggled on)
  let uavRingPolygonLatLons = [];
  if (showUavGlideRing) {
    const uavRing = computeAircraftOutwardGlideRing(params, tables);
    uavRingPolygonLatLons = uavRing.polygonLatLons;
  }

  // 5. Evaluate Direct Glide Trajectory from Aircraft to Landing Field
  const directGlide = await evaluateDirectGlideToLandingField(params, tables);
  if (jobId !== glideJobId) return;

  // 6. Evaluate Point LOS & Terrain Clearance at the Aircraft's exact position
  const pointLos = await computePointMinLos(latUav, lonUav);
  if (jobId !== glideJobId) return;

  const minLosAslFt = Math.ceil(pointLos.minAslM * M_TO_FT);
  const heightAboveLosFt = Math.round(hUavFt - minLosAslFt);
  const losSafetyMarginDiffFt = Math.round(heightAboveLosFt - losSafetyMarginFt);
  const isLosMaxOk = heightAboveLosFt >= 0;
  const isLosSafetyOk = losSafetyMarginDiffFt >= 0;

  const uavGroundFt = directGlide.uavGroundFt;
  const uavAglFt = Math.round(hUavFt - uavGroundFt);
  const verticalTerrainMarginFt = Math.round(uavAglFt - terrainVerticalClearanceFt);
  const thresholdMeters = (hUavFt - terrainVerticalClearanceFt) * FT_TO_M;
  const horizontalDistToHazardM = Math.round(
    queryHorizontalDistanceToHazardM(latUav, lonUav, thresholdMeters)
  );
  const isTerrainVerticalOk = verticalTerrainMarginFt >= 0;
  const isTerrainHorizontalOk = horizontalDistToHazardM >= terrainHorizontalSafetyM;
  const isTerrainSafe = isTerrainVerticalOk && isTerrainHorizontalOk;

  self.postMessage({
    type: 'GLIDE_AND_AIRCRAFT_RESULT',
    jobId,
    lfGroundFt: lfEnvelope.lfGroundFt,
    lfCatchmentPolygonLatLons: lfEnvelope.polygonLatLons,
    uavRingPolygonLatLons,
    aircraftStatus: {
      latUav,
      lonUav,
      hUavFt,
      // 1. Terrain safety at Aircraft
      uavGroundFt,
      uavAglFt,
      verticalTerrainMarginFt,
      horizontalDistToHazardM,
      isTerrainVerticalOk,
      isTerrainHorizontalOk,
      isTerrainSafe,
      // 2. LOS communication at Aircraft
      distFromStationKm: pointLos.distanceKm,
      azimuthFromStationDeg: pointLos.azimuthDeg,
      minLosAslFt,
      heightAboveLosFt,
      losSafetyMarginFt,
      losSafetyMarginDiffFt,
      isLosMaxOk,
      isLosSafetyOk,
      losBlockingDistKm: pointLos.blockingDistKm,
      losBlockingHeightFt: Math.round(pointLos.blockingHeightM * M_TO_FT),
      // 3. Emergency Glide from Aircraft to Landing Field
      glide: directGlide,
      // Overall combined safety status
      isAllSafe: isTerrainSafe && isLosSafetyOk && directGlide.canGlideToLf
    }
  });
}

// ---------------------------------------------------------------------------
// 6. Worker Message Dispatcher
// ---------------------------------------------------------------------------

self.onmessage = async (e) => {
  const msg = e.data;
  if (!msg || !msg.type) return;

  if (msg.type === 'DECODE_PNG_RES') {
    const resolver = pendingMainDecodes.get(msg.id);
    if (resolver) {
      pendingMainDecodes.delete(msg.id);
      resolver(msg.rgbaBuffer);
    }
    return;
  }

  if (msg.type === 'COMPUTE_LOS_VIEWSHED') {
    runProgressiveLosViewshed(msg.payload);
    return;
  }

  if (msg.type === 'FAST_UPDATE_LOS_ALT') {
    if (cachedLosRaysMap.size === 0) return;
    const { hUavM, losSafetyMarginM = 0, maxRangeKm = 200 } = msg.payload;
    const result = evaluateDualLosPolygons(hUavM, losSafetyMarginM, maxRangeKm, true);
    self.postMessage({
      type: 'LOS_VIEWSHED_RESULT',
      jobId: losJobId,
      ...result
    });
    return;
  }

  if (msg.type === 'GET_LOS_RAY_PROFILE') {
    if (cachedLosRaysMap.size === 0) return;
    const { azimuthDeg, hUavM, losSafetyMarginM = 0, maxRangeKm = 200 } = msg.payload;
    const profile = buildLosRayCrossSectionProfile(
      azimuthDeg,
      hUavM,
      losSafetyMarginM,
      maxRangeKm
    );
    self.postMessage({
      type: 'LOS_RAY_PROFILE_RESULT',
      profile
    });
    return;
  }

  if (msg.type === 'COMPUTE_TERRAIN_SAFETY') {
    runTerrainSafetyCompute(msg.payload);
    return;
  }

  if (msg.type === 'COMPUTE_GLIDE_AND_AIRCRAFT') {
    runGlideAndAircraftEvaluation(msg.payload);
    return;
  }

  if (msg.type === 'PROBE_POINT_ELEVATION') {
    const { reqId, lat, lon } = msg.payload;
    const elevM = await ensureAndSampleElevationM(lat, lon);
    const pointLos = await computePointMinLos(lat, lon);
    self.postMessage({
      type: 'PROBE_POINT_RESULT',
      reqId,
      lat,
      lon,
      elevationM: Math.round(elevM),
      elevationFt: Math.round(elevM * M_TO_FT),
      minLosAslFt: Math.ceil(pointLos.minAslM * M_TO_FT),
      distanceFromStationKm: pointLos.distanceKm,
      azimuthFromStationDeg: pointLos.azimuthDeg
    });
  }
};
