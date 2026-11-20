/**
 * UAV Mission Planner - Geodesic, WGS84 UTM Converter, DEM Grid Metrics,
 * 2D Euclidean Distance Transform (EDT), Marching Squares & Glide Interpolation Math
 */

export const EARTH_RADIUS_M = 6371000.0;
export const EARTH_RADIUS_KM = 6371.0;
export const EARTH_CIRCUMFERENCE_M = 40075016.68557849;
export const TILE_SIZE = 256;
export const FT_TO_M = 0.3048;
export const M_TO_FT = 1.0 / FT_TO_M;
export const NM_TO_KM = 1.852;
export const KM_TO_NM = 1.0 / NM_TO_KM;

// Terrain Safety Grid Limits
export const MIN_SUPPORTED_MAP_ZOOM = 8.0;
export const MAX_GRID_DIMENSION = 420;
export const MAX_TILES_PER_JOB = 20;

// WGS84 Ellipsoid Constants for UTM Conversion
const WGS84_A = 6378137.0;
const WGS84_F = 1.0 / 298.257223563;
const WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F;
const WGS84_EP2 = WGS84_E2 / (1 - WGS84_E2);
const UTM_K0 = 0.9996;

export function toRadians(deg) {
  return (deg * Math.PI) / 180.0;
}

export function toDegrees(rad) {
  return (rad * 180.0) / Math.PI;
}

export function feetToMeters(ft) {
  return ft * FT_TO_M;
}

export function metersToFeet(m) {
  return m * M_TO_FT;
}

/**
 * Great-circle distance between two coordinates using Haversine formula (returns km)
 */
export function haversineDistanceKm(lat1, lon1, lat2, lon2) {
  const dLat = toRadians(lat2 - lat1);
  const dLon = toRadians(lon2 - lon1);
  const rLat1 = toRadians(lat1);
  const rLat2 = toRadians(lat2);

  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(rLat1) * Math.cos(rLat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(Math.max(0, 1 - a)));
  return EARTH_RADIUS_KM * c;
}

/**
 * Forward azimuth (initial bearing) in degrees [0, 360) from point 1 to point 2
 */
export function initialBearingDeg(lat1, lon1, lat2, lon2) {
  const rLat1 = toRadians(lat1);
  const rLat2 = toRadians(lat2);
  const dLon = toRadians(lon2 - lon1);

  const y = Math.sin(dLon) * Math.cos(rLat2);
  const x =
    Math.cos(rLat1) * Math.sin(rLat2) -
    Math.sin(rLat1) * Math.cos(rLat2) * Math.cos(dLon);
  const brng = toDegrees(Math.atan2(y, x));
  return (brng + 360.0) % 360.0;
}

/**
 * Destination coordinate given starting point, distance (km), and bearing (degrees)
 */
export function destinationPoint(lat, lon, distanceKm, bearingDeg) {
  const angularDist = distanceKm / EARTH_RADIUS_KM;
  const brng = toRadians(bearingDeg);
  const lat1 = toRadians(lat);
  const lon1 = toRadians(lon);

  const sinLat1 = Math.sin(lat1);
  const cosLat1 = Math.cos(lat1);
  const sinAd = Math.sin(angularDist);
  const cosAd = Math.cos(angularDist);

  const sinLat2 = sinLat1 * cosAd + cosLat1 * sinAd * Math.cos(brng);
  const lat2 = Math.asin(Math.max(-1, Math.min(1, sinLat2)));
  const y = Math.sin(brng) * sinAd * cosLat1;
  const x = cosAd - sinLat1 * sinLat2;
  const lon2 = lon1 + Math.atan2(y, x);

  return {
    lat: toDegrees(lat2),
    lon: ((toDegrees(lon2) + 540.0) % 360.0) - 180.0
  };
}

/**
 * Convert WGS84 Geographic (lat, lon in degrees) to UTM WGS84
 */
export function latLonToUtm(lat, lon, forceZone = null) {
  const clampedLat = Math.max(-80, Math.min(84, lat));
  const normLon = (((lon + 180) % 360) + 360) % 360 - 180;

  const zone = forceZone || Math.floor((normLon + 180) / 6) + 1;
  const lonOrigin = (zone - 1) * 6 - 180 + 3;
  const lonOriginRad = toRadians(lonOrigin);

  const latRad = toRadians(clampedLat);
  const lonRad = toRadians(normLon);

  const sinLat = Math.sin(latRad);
  const cosLat = Math.cos(latRad);
  const tanLat = Math.tan(latRad);

  const N = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat * sinLat);
  const T = tanLat * tanLat;
  const C = WGS84_EP2 * cosLat * cosLat;
  const A = cosLat * (lonRad - lonOriginRad);

  const e2 = WGS84_E2;
  const e4 = e2 * e2;
  const e6 = e4 * e2;

  const M =
    WGS84_A *
    ((1 - e2 / 4 - (3 * e4) / 64 - (5 * e6) / 256) * latRad -
      ((3 * e2) / 8 + (3 * e4) / 32 + (45 * e6) / 1024) * Math.sin(2 * latRad) +
      ((15 * e4) / 256 + (45 * e6) / 1024) * Math.sin(4 * latRad) -
      ((35 * e6) / 3072) * Math.sin(6 * latRad));

  const easting =
    UTM_K0 *
      N *
      (A +
        ((1 - T + C) * A * A * A) / 6 +
        ((5 - 18 * T + T * T + 72 * C - 58 * WGS84_EP2) * Math.pow(A, 5)) / 120) +
    500000.0;

  let northing =
    UTM_K0 *
    (M +
      N *
        tanLat *
        ((A * A) / 2 +
          ((5 - T + 9 * C + 4 * C * C) * Math.pow(A, 4)) / 24 +
          ((61 - 58 * T + T * T + 600 * C - 330 * WGS84_EP2) * Math.pow(A, 6)) / 720));

  const hemisphere = clampedLat < 0 ? 'S' : 'N';
  if (clampedLat < 0) {
    northing += 10000000.0;
  }

  return {
    zone,
    hemisphere,
    easting: Math.round(easting),
    northing: Math.round(northing)
  };
}

/**
 * Convert UTM WGS84 (zone, hemisphere, easting, northing) to Geographic (lat, lon in degrees)
 */
export function utmToLatLon(zone, hemisphere, easting, northing) {
  if (
    !Number.isFinite(zone) ||
    zone < 1 ||
    zone > 60 ||
    !Number.isFinite(easting) ||
    !Number.isFinite(northing)
  ) {
    return null;
  }

  const x = easting - 500000.0;
  const y = String(hemisphere).toUpperCase() === 'S' ? northing - 10000000.0 : northing;

  const lonOrigin = (zone - 1) * 6 - 180 + 3;
  const e2 = WGS84_E2;
  const e4 = e2 * e2;
  const e6 = e4 * e2;
  const e1 = (1 - Math.sqrt(1 - e2)) / (1 + Math.sqrt(1 - e2));

  const M = y / UTM_K0;
  const mu = M / (WGS84_A * (1 - e2 / 4 - (3 * e4) / 64 - (5 * e6) / 256));

  const phi1Rad =
    mu +
    ((3 * e1) / 2 - (27 * Math.pow(e1, 3)) / 32) * Math.sin(2 * mu) +
    ((21 * e1 * e1) / 16 - (55 * Math.pow(e1, 4)) / 32) * Math.sin(4 * mu) +
    ((151 * Math.pow(e1, 3)) / 96) * Math.sin(6 * mu) +
    ((1097 * Math.pow(e1, 4)) / 512) * Math.sin(8 * mu);

  const sinPhi1 = Math.sin(phi1Rad);
  const cosPhi1 = Math.cos(phi1Rad);
  const tanPhi1 = Math.tan(phi1Rad);

  const N1 = WGS84_A / Math.sqrt(1 - e2 * sinPhi1 * sinPhi1);
  const T1 = tanPhi1 * tanPhi1;
  const C1 = WGS84_EP2 * cosPhi1 * cosPhi1;
  const R1 = (WGS84_A * (1 - e2)) / Math.pow(1 - e2 * sinPhi1 * sinPhi1, 1.5);
  const D = x / (N1 * UTM_K0);

  const latRad =
    phi1Rad -
    ((N1 * tanPhi1) / R1) *
      ((D * D) / 2 -
        ((5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * WGS84_EP2) * Math.pow(D, 4)) / 24 +
        ((61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * WGS84_EP2 - 3 * C1 * C1) *
          Math.pow(D, 6)) /
          720);

  const lonRad =
    toRadians(lonOrigin) +
    (D -
      ((1 + 2 * T1 + C1) * Math.pow(D, 3)) / 6 +
      ((5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * WGS84_EP2 + 24 * T1 * T1) *
        Math.pow(D, 5)) /
        120) /
      cosPhi1;

  const lat = toDegrees(latRad);
  const lon = toDegrees(lonRad);
  if (!Number.isFinite(lat) || !Number.isFinite(lon) || Math.abs(lat) > 85) {
    return null;
  }
  return { lat, lon };
}

/**
 * Web Mercator Tile Conversions
 */
export function metersPerTilePixel(lat, zoom) {
  const clampedLat = Math.max(-85.05112878, Math.min(85.05112878, lat));
  const latRad = toRadians(clampedLat);
  return (EARTH_CIRCUMFERENCE_M * Math.cos(latRad)) / (TILE_SIZE * Math.pow(2, zoom));
}

export function lonLatToTileFloat(lon, lat, zoom) {
  const n = Math.pow(2, zoom);
  const clampedLat = Math.max(-85.05112878, Math.min(85.05112878, lat));
  const latRad = toRadians(clampedLat);
  const x = ((lon + 180.0) / 360.0) * n;
  const y = ((1.0 - Math.log(Math.tan(latRad) + 1.0 / Math.cos(latRad)) / Math.PI) / 2.0) * n;
  return [x, y];
}

export function tileFloatToLonLat(x, y, zoom) {
  const n = Math.pow(2, zoom);
  const lon = (x / n) * 360.0 - 180.0;
  const latRad = Math.atan(Math.sinh(Math.PI * (1.0 - (2.0 * y) / n)));
  const lat = toDegrees(latRad);
  return [lon, lat];
}

/**
 * Maps continuous map zoom to discrete DEM zoom tiers (10, 12, 14) with hysteresis
 */
export function selectDemZoomTier(mapZoom, currentDemZoom = null) {
  if (mapZoom < MIN_SUPPORTED_MAP_ZOOM) {
    return null;
  }

  if (currentDemZoom === 14) {
    if (mapZoom >= 12.2) return 14;
  } else if (currentDemZoom === 12) {
    if (mapZoom >= 13.0) return 14;
    if (mapZoom >= 10.2) return 12;
  } else if (currentDemZoom === 10) {
    if (mapZoom >= 13.0) return 14;
    if (mapZoom >= 11.0) return 12;
    if (mapZoom >= MIN_SUPPORTED_MAP_ZOOM) return 10;
  }

  if (mapZoom >= 12.6) return 14;
  if (mapZoom >= 10.6) return 12;
  return 10;
}

export function expandBBoxByMeters(bbox, meters) {
  const centerLat = (bbox.north + bbox.south) / 2;
  const latDegPerMeter = 1 / 111320;
  const cosLat = Math.max(0.15, Math.cos(toRadians(centerLat)));
  const lonDegPerMeter = 1 / (111320 * cosLat);

  const dLat = meters * latDegPerMeter;
  const dLon = meters * lonDegPerMeter;

  return {
    west: Math.max(-180, bbox.west - dLon),
    south: Math.max(-85, bbox.south - dLat),
    east: Math.min(180, bbox.east + dLon),
    north: Math.min(85, bbox.north + dLat)
  };
}

export function isViewportInsideBounds(viewport, outer) {
  return (
    viewport.west >= outer.west &&
    viewport.east <= outer.east &&
    viewport.south >= outer.south &&
    viewport.north <= outer.north
  );
}

export function computeTileRange(bounds, demZoom) {
  const maxIndex = Math.pow(2, demZoom) - 1;
  const [x1, y1] = lonLatToTileFloat(bounds.west, bounds.north, demZoom);
  const [x2, y2] = lonLatToTileFloat(bounds.east, bounds.south, demZoom);

  const minTileX = Math.max(0, Math.min(maxIndex, Math.floor(x1)));
  const maxTileX = Math.max(minTileX, Math.min(maxIndex, Math.floor(x2)));
  const minTileY = Math.max(0, Math.min(maxIndex, Math.floor(y1)));
  const maxTileY = Math.max(minTileY, Math.min(maxIndex, Math.floor(y2)));

  return { minTileX, maxTileX, minTileY, maxTileY };
}

export function resolveGridMetrics(bounds, requestedDemZoom) {
  let demZoom = requestedDemZoom;
  let range = computeTileRange(bounds, demZoom);

  while (
    demZoom > 8 &&
    (range.maxTileX - range.minTileX + 1) * (range.maxTileY - range.minTileY + 1) >
      MAX_TILES_PER_JOB
  ) {
    demZoom -= 1;
    range = computeTileRange(bounds, demZoom);
  }

  const tilesX = range.maxTileX - range.minTileX + 1;
  const tilesY = range.maxTileY - range.minTileY + 1;
  const rawWidth = tilesX * TILE_SIZE;
  const rawHeight = tilesY * TILE_SIZE;

  const maxRaw = Math.max(rawWidth, rawHeight);
  const stride = Math.max(1, Math.ceil(maxRaw / MAX_GRID_DIMENSION));
  const width = Math.floor(rawWidth / stride);
  const height = Math.floor(rawHeight / stride);

  const centerLat = (bounds.north + bounds.south) / 2;
  const metersPerGridPixel = metersPerTilePixel(centerLat, demZoom) * stride;

  return {
    demZoom,
    minTileX: range.minTileX,
    maxTileX: range.maxTileX,
    minTileY: range.minTileY,
    maxTileY: range.maxTileY,
    stride,
    width,
    height,
    metersPerGridPixel
  };
}

export function buildComputationEnvelope(viewport, demZoom, safetyRangeMeters, overscanRatio = 0.35) {
  const lonSpan = Math.max(0.001, viewport.east - viewport.west);
  const latSpan = Math.max(0.001, viewport.north - viewport.south);

  const candidateRatios = [overscanRatio, 0.2, 0.1];
  for (const ratio of candidateRatios) {
    const overscanBounds = {
      west: Math.max(-180, viewport.west - lonSpan * ratio),
      south: Math.max(-85, viewport.south - latSpan * ratio),
      east: Math.min(180, viewport.east + lonSpan * ratio),
      north: Math.min(85, viewport.north + latSpan * ratio)
    };

    const paddedOverscan = expandBBoxByMeters(overscanBounds, safetyRangeMeters);
    const tileRange = computeTileRange(paddedOverscan, demZoom);
    const tileCount =
      (tileRange.maxTileX - tileRange.minTileX + 1) *
      (tileRange.maxTileY - tileRange.minTileY + 1);

    if (tileCount <= MAX_TILES_PER_JOB) {
      return {
        demZoom,
        fetchBounds: paddedOverscan,
        safeInnerBounds: overscanBounds,
        safetyRangeMeters
      };
    }
  }

  const tightFetch = expandBBoxByMeters(viewport, safetyRangeMeters);
  const metrics = resolveGridMetrics(tightFetch, demZoom);
  return {
    demZoom: metrics.demZoom,
    fetchBounds: tightFetch,
    safeInnerBounds: viewport,
    safetyRangeMeters
  };
}

export function gridPointToLonLat(gx, gy, metrics) {
  const tileX = metrics.minTileX + (gx * metrics.stride) / TILE_SIZE;
  const tileY = metrics.minTileY + (gy * metrics.stride) / TILE_SIZE;
  return tileFloatToLonLat(tileX, tileY, metrics.demZoom);
}

export function sampleTileBilinear(tileData, fracPixelX, fracPixelY) {
  const x0 = Math.max(0, Math.min(TILE_SIZE - 1, Math.floor(fracPixelX)));
  const y0 = Math.max(0, Math.min(TILE_SIZE - 1, Math.floor(fracPixelY)));
  const x1 = Math.min(TILE_SIZE - 1, x0 + 1);
  const y1 = Math.min(TILE_SIZE - 1, y0 + 1);

  const dx = Math.max(0, Math.min(1, fracPixelX - x0));
  const dy = Math.max(0, Math.min(1, fracPixelY - y0));

  const v00 = tileData[y0 * TILE_SIZE + x0];
  const v10 = tileData[y0 * TILE_SIZE + x1];
  const v01 = tileData[y1 * TILE_SIZE + x0];
  const v11 = tileData[y1 * TILE_SIZE + x1];

  const top = v00 * (1 - dx) + v10 * dx;
  const bottom = v01 * (1 - dx) + v11 * dx;
  return top * (1 - dy) + bottom * dy;
}

/* ==========================================================================
   2D Euclidean Distance Transform (Felzenszwalb & Huttenlocher, O(N))
   & Marching Squares Contour / Polygon Extraction
   ========================================================================== */

const INF = 1e20;

function edt1d(f, n, d, v, z) {
  let k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = INF;

  for (let q = 1; q < n; q++) {
    let s = 0;
    while (true) {
      const r = v[k];
      s = (f[q] + q * q - (f[r] + r * r)) / (2 * q - 2 * r);
      if (s <= z[k]) {
        k--;
      } else {
        break;
      }
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = INF;
  }

  k = 0;
  for (let q = 0; q < n; q++) {
    while (z[k + 1] < q) {
      k++;
    }
    const r = v[k];
    const diff = q - r;
    d[q] = diff * diff + f[r];
  }
}

export function computeEuclideanDistanceField(elevations, width, height, threshold) {
  const total = width * height;
  const distSq = new Float32Array(total);
  let hasFeature = false;

  for (let i = 0; i < total; i++) {
    if (elevations[i] >= threshold) {
      distSq[i] = 0;
      hasFeature = true;
    } else {
      distSq[i] = INF;
    }
  }

  if (!hasFeature) {
    return { distPx: distSq, hasFeature: false };
  }

  const maxDim = Math.max(width, height);
  const f = new Float32Array(maxDim);
  const d = new Float32Array(maxDim);
  const v = new Int32Array(maxDim);
  const z = new Float32Array(maxDim + 1);

  for (let x = 0; x < width; x++) {
    for (let y = 0; y < height; y++) {
      f[y] = distSq[y * width + x];
    }
    edt1d(f, height, d, v, z);
    for (let y = 0; y < height; y++) {
      distSq[y * width + x] = d[y];
    }
  }

  for (let y = 0; y < height; y++) {
    const rowOffset = y * width;
    for (let x = 0; x < width; x++) {
      f[x] = distSq[rowOffset + x];
    }
    edt1d(f, width, d, v, z);
    for (let x = 0; x < width; x++) {
      distSq[rowOffset + x] = Math.sqrt(d[x]);
    }
  }

  return { distPx: distSq, hasFeature: true };
}

function perpendicularDistance(px, py, x1, y1, x2, y2) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  if (dx === 0 && dy === 0) {
    return Math.hypot(px - x1, py - y1);
  }
  const num = Math.abs(dy * px - dx * py + x2 * y1 - y2 * x1);
  return num / Math.hypot(dx, dy);
}

export function simplifyPolyline(points, tolerance) {
  if (points.length <= 2 || tolerance <= 0) {
    return points;
  }

  const keep = new Uint8Array(points.length);
  keep[0] = 1;
  keep[points.length - 1] = 1;

  const stack = [[0, points.length - 1]];
  while (stack.length > 0) {
    const [start, end] = stack.pop();
    if (end - start <= 1) continue;

    let maxDist = 0;
    let maxIdx = -1;
    const [x1, y1] = points[start];
    const [x2, y2] = points[end];

    for (let i = start + 1; i < end; i++) {
      const dist = perpendicularDistance(points[i][0], points[i][1], x1, y1, x2, y2);
      if (dist > maxDist) {
        maxDist = dist;
        maxIdx = i;
      }
    }

    if (maxDist > tolerance && maxIdx !== -1) {
      keep[maxIdx] = 1;
      stack.push([start, maxIdx], [maxIdx, end]);
    }
  }

  const result = [];
  for (let i = 0; i < points.length; i++) {
    if (keep[i]) result.push(points[i]);
  }
  return result;
}

function interp(a, b, threshold) {
  const denom = b - a;
  if (Math.abs(denom) < 1e-9) return 0.5;
  return Math.max(0, Math.min(1, (threshold - a) / denom));
}

export function extractIsoLines(values, width, height, threshold, simplifyTolerancePx = 0.35) {
  if (width < 2 || height < 2) return [];

  const stride = width + 1;
  const edgeKeyH = (col, row) => (row * stride + col) * 2;
  const edgeKeyV = (col, row) => (row * stride + col) * 2 + 1;

  const edgePoints = new Map();
  const adj = new Map();

  const addSeg = (k1, p1, k2, p2) => {
    if (!edgePoints.has(k1)) edgePoints.set(k1, p1);
    if (!edgePoints.has(k2)) edgePoints.set(k2, p2);

    let list1 = adj.get(k1);
    if (!list1) {
      list1 = [];
      adj.set(k1, list1);
    }
    list1.push(k2);

    let list2 = adj.get(k2);
    if (!list2) {
      list2 = [];
      adj.set(k2, list2);
    }
    list2.push(k1);
  };

  for (let j = 0; j < height - 1; j++) {
    const row0 = j * width;
    const row1 = (j + 1) * width;

    for (let i = 0; i < width - 1; i++) {
      const v0 = values[row0 + i];
      const v1 = values[row0 + i + 1];
      const v2 = values[row1 + i + 1];
      const v3 = values[row1 + i];

      const b0 = v0 >= threshold ? 8 : 0;
      const b1 = v1 >= threshold ? 4 : 0;
      const b2 = v2 >= threshold ? 2 : 0;
      const b3 = v3 >= threshold ? 1 : 0;
      const code = b0 | b1 | b2 | b3;

      if (code === 0 || code === 15) continue;

      const topPt = () => [i + 0.5 + interp(v0, v1, threshold), j + 0.5];
      const rightPt = () => [i + 1.5, j + 0.5 + interp(v1, v2, threshold)];
      const bottomPt = () => [i + 0.5 + interp(v3, v2, threshold), j + 1.5];
      const leftPt = () => [i + 0.5, j + 0.5 + interp(v0, v3, threshold)];

      const kTop = edgeKeyH(i, j);
      const kRight = edgeKeyV(i + 1, j);
      const kBottom = edgeKeyH(i, j + 1);
      const kLeft = edgeKeyV(i, j);

      switch (code) {
        case 1:
        case 14:
          addSeg(kLeft, leftPt(), kBottom, bottomPt());
          break;
        case 2:
        case 13:
          addSeg(kBottom, bottomPt(), kRight, rightPt());
          break;
        case 3:
        case 12:
          addSeg(kLeft, leftPt(), kRight, rightPt());
          break;
        case 4:
        case 11:
          addSeg(kTop, topPt(), kRight, rightPt());
          break;
        case 6:
        case 9:
          addSeg(kTop, topPt(), kBottom, bottomPt());
          break;
        case 7:
        case 8:
          addSeg(kLeft, leftPt(), kTop, topPt());
          break;
        case 5: {
          const center = 0.25 * (v0 + v1 + v2 + v3);
          if (center >= threshold) {
            addSeg(kLeft, leftPt(), kBottom, bottomPt());
            addSeg(kTop, topPt(), kRight, rightPt());
          } else {
            addSeg(kLeft, leftPt(), kTop, topPt());
            addSeg(kBottom, bottomPt(), kRight, rightPt());
          }
          break;
        }
        case 10: {
          const center = 0.25 * (v0 + v1 + v2 + v3);
          if (center >= threshold) {
            addSeg(kLeft, leftPt(), kTop, topPt());
            addSeg(kBottom, bottomPt(), kRight, rightPt());
          } else {
            addSeg(kLeft, leftPt(), kBottom, bottomPt());
            addSeg(kTop, topPt(), kRight, rightPt());
          }
          break;
        }
      }
    }
  }

  const visitedEdges = new Set();
  const KEY_BASE = (stride * (height + 1) + 1) * 2 + 4;
  const segId = (a, b) => (a < b ? a * KEY_BASE + b : b * KEY_BASE + a);
  const polylines = [];

  const traceFrom = (startKey, firstNeighbor, closeLoop) => {
    const chain = [];
    const pStart = edgePoints.get(startKey);
    if (!pStart) return;
    chain.push(pStart);

    let prev = startKey;
    let curr = firstNeighbor;
    visitedEdges.add(segId(prev, curr));

    while (true) {
      const pt = edgePoints.get(curr);
      if (pt) chain.push(pt);

      if (curr === startKey && closeLoop) {
        break;
      }

      const neighbors = adj.get(curr);
      if (!neighbors) break;

      let next = null;
      for (const n of neighbors) {
        if (!visitedEdges.has(segId(curr, n))) {
          next = n;
          break;
        }
      }
      if (next === null) break;

      visitedEdges.add(segId(curr, next));
      prev = curr;
      curr = next;
    }

    if (chain.length >= 2) {
      const isClosed =
        chain.length >= 4 &&
        chain[0][0] === chain[chain.length - 1][0] &&
        chain[0][1] === chain[chain.length - 1][1];

      if (isClosed) {
        const simplified = simplifyPolyline(chain.slice(0, -1), simplifyTolerancePx);
        if (simplified.length >= 3) {
          simplified.push(simplified[0]);
          polylines.push(simplified);
        }
      } else {
        const simplified = simplifyPolyline(chain, simplifyTolerancePx);
        if (simplified.length >= 2) {
          polylines.push(simplified);
        }
      }
    }
  };

  for (const [key, neighbors] of adj.entries()) {
    if (neighbors.length === 1) {
      const n = neighbors[0];
      if (!visitedEdges.has(segId(key, n))) {
        traceFrom(key, n, false);
      }
    }
  }

  for (const [key, neighbors] of adj.entries()) {
    for (const n of neighbors) {
      if (!visitedEdges.has(segId(key, n))) {
        traceFrom(key, n, true);
      }
    }
  }

  return polylines;
}

export function isoLinesToLatLons(polylines, metrics) {
  return polylines.map((line) =>
    line.map(([gx, gy]) => {
      const [lon, lat] = gridPointToLonLat(gx, gy, metrics);
      return [lat, lon];
    })
  );
}

/**
 * Extracts guaranteed closed polygon rings (in [lat, lon] format for Leaflet)
 * representing all terrain >= target plus the horizontal safety buffer (distPx <= safetyRadiusPx).
 * Uses a 1-cell exterior boundary pad (< 0) so every Marching Squares contour is topologically closed.
 */
export function extractSafetyFillPolygons(distPx, metrics, safetyRadiusPx) {
  const { width, height } = metrics;
  const padW = width + 2;
  const padH = height + 2;
  const effectiveRadius = Math.max(0.05, safetyRadiusPx);

  // Padded score field: interior has (effectiveRadius - distPx), 1-cell border has -1.0
  const paddedScore = new Float32Array(padW * padH);
  paddedScore.fill(-1.0);

  for (let y = 0; y < height; y++) {
    const srcRow = y * width;
    const dstRow = (y + 1) * padW;
    for (let x = 0; x < width; x++) {
      paddedScore[dstRow + (x + 1)] = effectiveRadius - distPx[srcRow + x];
    }
  }

  const rawLoops = extractIsoLines(paddedScore, padW, padH, 0.0, 0.42);
  const ringsLatLon = [];

  for (const loop of rawLoops) {
    if (loop.length < 4) continue;
    const ring = [];
    for (let i = 0; i < loop.length; i++) {
      // Shift back by 1.0 cell to account for the 1-cell padding and clamp to grid domain
      const gx = Math.max(0, Math.min(width, loop[i][0] - 1.0));
      const gy = Math.max(0, Math.min(height, loop[i][1] - 1.0));
      const [lon, lat] = gridPointToLonLat(gx, gy, metrics);
      ring.push([lat, lon]);
    }
    if (ring.length >= 4) {
      ringsLatLon.push(ring);
    }
  }

  return ringsLatLon;
}

/* ==========================================================================
   Glide Interpolation Utilities
   ========================================================================== */

export function linearInterpolate(x, x0, y0, x1, y1) {
  if (x0 === x1) return y0;
  if (x <= x0) return y0;
  if (x >= x1) return y1;
  return y0 + ((x - x0) * (y1 - y0)) / (x1 - x0);
}

export function interpolateAngle(currentAltKft, alt0Kft, angle0, alt1Kft, angle1) {
  if (alt0Kft === alt1Kft) return angle0;
  if (currentAltKft <= alt0Kft) return angle0;
  if (currentAltKft >= alt1Kft) return angle1;

  let diff = angle1 - angle0;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  const interpolated = angle0 + diff * ((currentAltKft - alt0Kft) / (alt1Kft - alt0Kft));
  return (interpolated + 360.0) % 360.0;
}
