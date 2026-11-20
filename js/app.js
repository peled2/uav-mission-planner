/**
 * UAV Unified Mission Planner PWA - Main Application Controller
 *
 * Integrates:
 * 1. Dual 360° LOS Viewshed (Max range at H_UAV + Safety envelope at H_UAV - H_LOS_Safety)
 * 2. Terrain Obstacle Contour (H_UAV - H_Terrain_Safety) & Horizontal Safety Buffer (2D EDT)
 * 3. Terrain-Aware Emergency Glide Catchment Envelope around Landing Field
 * 4. Aircraft Position Simulator ("נקודת מטוס" + "טבעת מטוס") & Real-Time Safety Telemetry
 */

import {
  FT_TO_M,
  M_TO_FT,
  initialBearingDeg,
  destinationPoint,
  haversineDistanceKm,
  latLonToUtm,
  utmToLatLon,
  selectDemZoomTier,
  buildComputationEnvelope,
  expandBBoxByMeters,
  isViewportInsideBounds
} from './mission-math.js';
import { LosChartRenderer, GlideChartRenderer } from './chart-renderer.js';

const STORAGE_KEY_GLIDE_CONFIG = 'uav_mission_planner_glide_v1';

const DEFAULT_SPEED_ROD_PAIRS = [
  { speed: 52, rod: 400 },
  { speed: 75, rod: 1200 }
];

const DEFAULT_WIND_LAYERS = [
  { altitudeKft: 0, speed: 5, direction: 270 },
  { altitudeKft: 5, speed: 12, direction: 280 },
  { altitudeKft: 10, speed: 18, direction: 290 }
];

const state = {
  mapType: 'topo', // 'topo' | 'ortho'
  sidebarCollapsed: false,
  activeTab: 'aircraft', // 'aircraft' | 'los' | 'terrain' | 'glide'
  placementMode: 'nav', // 'nav' | 'uav' | 'station' | 'lf'

  // Layer Visibility
  showLosLayer: true,
  showTerrainLayer: true,
  showGlideLayer: true,
  showUavRingLayer: false,
  showRangeRings: true,

  // Global Aircraft Altitude (ft ASL)
  hUavFt: 6000,

  // 1. Aircraft Position (Default: Judean Foothills / Modi'in area)
  latUav: 31.935,
  lonUav: 34.935,

  // 2. Ground Communication Station (Default: Central Coastal Plain)
  lat0: 31.898,
  lon0: 34.691,
  stationGroundM: 32,
  hAntM: 15,
  losSafetyMarginFt: 2000, // e.g., H_UAV = 6,000 ft -> Safety envelope at 4,000 ft
  rMaxKm: 150,
  curvatureMode: 'rf', // 'rf' (4/3) | 'opt' (1.0)
  kFactor: 4 / 3,
  losRayMode: 'uav', // 'uav' (towards aircraft) | 'hover' (map cursor hover)
  inspectAzimuthDeg: 65,
  raySummaries: [],

  // 3. Terrain Obstacles & Safety Buffer
  terrainVerticalClearanceFt: 1000, // Threshold = H_UAV - 1,000 ft
  terrainHorizontalSafetyM: 500,
  isTerrainAreaLocked: false,
  terrainEnvelope: null,
  terrainReqId: 0,

  // 4. Emergency Glide & Landing Field (Default: Separate Runway East of Station)
  latLf: 31.915,
  lonLf: 34.785,
  lfGroundFt: 210,
  lfSafetyMarginFt: 500, // Minimum arrival altitude above landing field
  ridgeClearanceFt: 200, // Minimum terrain clearance along emergency glide path
  batteryTimeMin: 20,
  speedRoDPairs: [...DEFAULT_SPEED_ROD_PAIRS],
  windLayers: [...DEFAULT_WIND_LAYERS],

  // Latest Evaluated Aircraft Status
  aircraftStatus: null,
  lastProbeTime: 0,
  probeReqId: 0
};

let map = null;
let canvasRenderer = null;
let topoLayerGroup = null;
let orthoLayerGroup = null;

// Map Overlay Layer Groups
let rangeRingsGroup = null;
let terrainFillGroup = null;
let terrainOutlineGroup = null;
let terrainContourGroup = null;
let terrainLockedBoxLayer = null;

let losPrimaryHaloLayer = null;
let losPrimaryPolygonLayer = null;
let losSafetyPolygonLayer = null;
let losInspectRayLayer = null;
let losInspectShadowLayer = null;

let lfGlideHaloLayer = null;
let lfGlidePolygonLayer = null;
let uavGlideRingLayer = null;
let directGlideLineLayer = null;
let criticalRidgeMarker = null;

// Entity Markers
let stationMarker = null;
let lfMarker = null;
let uavMarker = null;

// Worker & Charts
let worker = null;
let losChart = null;
let glideChart = null;

let terrainDebounceTimer = null;
let glideDebounceTimer = null;

/**
 * Load saved Aircraft Polar (IAS/RoD) & Wind Layers from localStorage
 */
function loadSavedGlideConfig() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY_GLIDE_CONFIG);
    if (!raw) return;
    const parsed = JSON.parse(raw);
    if (Array.isArray(parsed.speedRoDPairs) && parsed.speedRoDPairs.length >= 2) {
      state.speedRoDPairs = parsed.speedRoDPairs;
    }
    if (Array.isArray(parsed.windLayers) && parsed.windLayers.length >= 1) {
      state.windLayers = parsed.windLayers;
    }
    if (Number.isFinite(parsed.lfSafetyMarginFt)) {
      state.lfSafetyMarginFt = parsed.lfSafetyMarginFt;
    }
    if (Number.isFinite(parsed.ridgeClearanceFt)) {
      state.ridgeClearanceFt = parsed.ridgeClearanceFt;
    }
    if (Number.isFinite(parsed.batteryTimeMin) && parsed.batteryTimeMin > 0) {
      state.batteryTimeMin = parsed.batteryTimeMin;
    }
  } catch (e) {
    // Use defaults
  }
}

function saveGlideConfig() {
  try {
    localStorage.setItem(
      STORAGE_KEY_GLIDE_CONFIG,
      JSON.stringify({
        speedRoDPairs: state.speedRoDPairs,
        windLayers: state.windLayers,
        lfSafetyMarginFt: state.lfSafetyMarginFt,
        ridgeClearanceFt: state.ridgeClearanceFt,
        batteryTimeMin: state.batteryTimeMin
      })
    );
  } catch (e) {
    // Ignore storage quota errors
  }
}

/**
 * Initialize Full-Viewport Leaflet Map with High-FPS Canvas Renderer
 */
function initMap() {
  canvasRenderer = L.canvas({ padding: 0.5 });

  map = L.map('map', {
    center: [31.915, 34.85],
    zoom: 10,
    zoomControl: false,
    preferCanvas: true,
    renderer: canvasRenderer,
    attributionControl: true
  });

  L.control.scale({ imperial: false, metric: true, position: 'bottomleft' }).addTo(map);

  // Base Layer 1: Topographic (OpenTopoMap)
  const openTopo = L.tileLayer('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png', {
    maxZoom: 17,
    attribution: '&copy; OpenTopoMap'
  });
  topoLayerGroup = L.layerGroup([openTopo]);

  // Base Layer 2: Orthophoto (Esri World Imagery + Places)
  const esriImagery = L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    {
      maxZoom: 19,
      attribution: '&copy; Esri World Imagery'
    }
  );
  const esriPlaces = L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}',
    {
      maxZoom: 19,
      opacity: 0.85
    }
  );
  orthoLayerGroup = L.layerGroup([esriImagery, esriPlaces]);

  topoLayerGroup.addTo(map);

  // 1. Terrain Hazard & Safety Buffer Layers
  terrainFillGroup = L.layerGroup().addTo(map);
  terrainOutlineGroup = L.layerGroup().addTo(map);
  terrainContourGroup = L.layerGroup().addTo(map);
  terrainLockedBoxLayer = L.polyline([], {
    color: '#38bdf8',
    weight: 1.8,
    dashArray: '6 4',
    opacity: 0.85,
    interactive: false
  }).addTo(map);

  // 2. Concentric Range Rings around Ground Station
  rangeRingsGroup = L.layerGroup().addTo(map);

  // 3. Dual LOS Viewshed Polygons (Max Range at H_UAV + Safety Range at H_UAV - H_LOS_Safety)
  losPrimaryHaloLayer = L.polygon([], {
    color: '#050b12',
    weight: 5.5,
    opacity: 0.8,
    fill: false,
    interactive: false
  }).addTo(map);

  losPrimaryPolygonLayer = L.polygon([], {
    color: '#22c55e',
    weight: 2.5,
    opacity: 0.95,
    fillColor: '#22c55e',
    fillOpacity: 0.08,
    interactive: false
  }).addTo(map);

  losSafetyPolygonLayer = L.polygon([], {
    color: '#38bdf8',
    weight: 2.3,
    opacity: 0.95,
    dashArray: '7 5',
    fillColor: '#38bdf8',
    fillOpacity: 0.14,
    interactive: false
  }).addTo(map);

  losInspectRayLayer = L.polyline([], {
    color: '#38bdf8',
    weight: 2.0,
    opacity: 0.85,
    interactive: false
  }).addTo(map);

  losInspectShadowLayer = L.polyline([], {
    color: '#ef4444',
    weight: 1.6,
    opacity: 0.6,
    dashArray: '4 4',
    interactive: false
  }).addTo(map);

  // 4. Landing Field Terrain-Aware Glide Catchment Envelope
  lfGlideHaloLayer = L.polygon([], {
    color: '#050b12',
    weight: 5.0,
    opacity: 0.75,
    fill: false,
    interactive: false
  }).addTo(map);

  lfGlidePolygonLayer = L.polygon([], {
    color: '#a855f7',
    weight: 2.5,
    opacity: 0.95,
    fillColor: '#a855f7',
    fillOpacity: 0.15,
    interactive: false
  }).addTo(map);

  // 5. Aircraft Outward Glide Ring ("טבעת מטוס")
  uavGlideRingLayer = L.polygon([], {
    color: '#f59e0b',
    weight: 2.4,
    opacity: 0.95,
    dashArray: '6 4',
    fillColor: '#f59e0b',
    fillOpacity: 0.11,
    interactive: false
  });

  // 6. Direct Glide Line between Aircraft and Landing Field
  directGlideLineLayer = L.polyline([], {
    color: '#22c55e',
    weight: 2.4,
    opacity: 0.9,
    dashArray: '6 4',
    interactive: false
  }).addTo(map);

  // 7. Draggable Entity Markers: Ground Station (📡), Landing Field (🛬), Aircraft (✈️)
  stationMarker = L.marker([state.lat0, state.lon0], {
    draggable: true,
    icon: L.divIcon({
      className: 'entity-marker-wrap',
      iconSize: [36, 36],
      iconAnchor: [18, 18],
      html: `
        <div class="entity-pin pin-station" title="תחנת תקשורת קרקעית (גרור לשינוי מיקום)">
          <span class="pin-badge-label">תחנה</span>
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="#22c55e" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M4.9 19.1C1 15.2 1 8.8 4.9 4.9"/>
            <path d="M7.8 16.2c-2.3-2.3-2.3-6.1 0-8.5"/>
            <circle cx="12" cy="12" r="2"/>
            <path d="M16.2 7.8c2.3 2.3 2.3 6.1 0 8.5"/>
            <path d="M19.1 4.9C23 8.8 23 15.1 19.1 19"/>
            <line x1="12" y1="14" x2="12" y2="22"/>
            <line x1="8" y1="22" x2="16" y2="22"/>
          </svg>
        </div>
      `
    })
  }).addTo(map);

  lfMarker = L.marker([state.latLf, state.lonLf], {
    draggable: true,
    icon: L.divIcon({
      className: 'entity-marker-wrap',
      iconSize: [36, 36],
      iconAnchor: [18, 18],
      html: `
        <div class="entity-pin pin-lf" title="שדה נחיתה לחירום (גרור לשינוי מיקום)">
          <span class="pin-badge-label">שדה נחיתה</span>
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="#c084fc" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M2 22h20"/>
            <path d="M3.77 10.77 2 9l2-4.5 1.1.55c.55.28.9.84.9 1.45s.35 1.17.9 1.45L8 8.5l3-6 1.05.53a2 2 0 0 1 1.09 1.52l.72 5.4a2 2 0 0 0 1.09 1.52l4.4 2.2c.42.22.78.55 1.01.96l.6 1.03c.49.88-.06 1.98-1.06 2.1l-1.18.15c-.47.06-.95-.02-1.37-.24L4.29 11.15a2 2 0 0 1-.52-.38Z"/>
          </svg>
        </div>
      `
    })
  }).addTo(map);

  uavMarker = L.marker([state.latUav, state.lonUav], {
    draggable: true,
    zIndexOffset: 500,
    icon: L.divIcon({
      className: 'entity-marker-wrap',
      iconSize: [38, 38],
      iconAnchor: [19, 19],
      html: `
        <div class="entity-pin pin-uav" title="מיקום מטוס (גרור לבדיקת בטיחות בנקודה)">
          <span class="pin-badge-label">מטוס</span>
          <svg width="19" height="19" viewBox="0 0 24 24" fill="none" stroke="#38bdf8" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M17.8 19.2 16 11l3.5-3.5C21 6 21.5 4 21 3c-1-.5-3 0-4.5 1.5L13 8 4.8 6.2c-.5-.1-.9.1-1.1.5l-.3.5c-.2.5-.1 1 .3 1.3L9 12l-2 3H4l-1 1 3 2 2 3 1-1v-3l3-2 3.5 5.3c.3.4.8.5 1.3.3l.5-.2c.4-.3.6-.7.5-1.2z"/>
          </svg>
        </div>
      `
    })
  }).addTo(map);

  // Drag handlers for all 3 markers
  stationMarker.on('drag', (e) => {
    const pos = e.target.getLatLng();
    state.lat0 = pos.lat;
    state.lon0 = pos.lng;
    syncUtmInputs('station', state.lat0, state.lon0);
    drawConcentricRangeRings();
  });
  stationMarker.on('dragend', () => {
    triggerLosViewshedCompute();
    triggerGlideAndAircraftCompute();
  });

  lfMarker.on('drag', (e) => {
    const pos = e.target.getLatLng();
    state.latLf = pos.lat;
    state.lonLf = pos.lng;
    syncUtmInputs('lf', state.latLf, state.lonLf);
    updateDirectGlideLineGeometry();
  });
  lfMarker.on('dragend', () => {
    triggerGlideAndAircraftCompute();
  });

  uavMarker.on('drag', (e) => {
    const pos = e.target.getLatLng();
    state.latUav = pos.lat;
    state.lonUav = pos.lng;
    syncUtmInputs('uav', state.latUav, state.lonUav);
    updateDirectGlideLineGeometry();
    if (state.losRayMode === 'uav') {
      const az = initialBearingDeg(state.lat0, state.lon0, state.latUav, state.lonUav);
      updateLosInspectionRay(az, false);
    }
  });
  uavMarker.on('dragend', () => {
    triggerGlideAndAircraftCompute();
    if (state.losRayMode === 'uav') {
      const az = initialBearingDeg(state.lat0, state.lon0, state.latUav, state.lonUav);
      updateLosInspectionRay(az, true);
    }
  });

  // Map click placement based on active placementMode ('uav' | 'station' | 'lf')
  map.on('click', (e) => {
    if (state.placementMode === 'uav') {
      state.latUav = e.latlng.lat;
      state.lonUav = e.latlng.lng;
      uavMarker.setLatLng([state.latUav, state.lonUav]);
      syncUtmInputs('uav', state.latUav, state.lonUav);
      updateDirectGlideLineGeometry();
      triggerGlideAndAircraftCompute();
      if (state.losRayMode === 'uav') {
        updateLosInspectionRay(
          initialBearingDeg(state.lat0, state.lon0, state.latUav, state.lonUav),
          true
        );
      }
    } else if (state.placementMode === 'station') {
      state.lat0 = e.latlng.lat;
      state.lon0 = e.latlng.lng;
      stationMarker.setLatLng([state.lat0, state.lon0]);
      syncUtmInputs('station', state.lat0, state.lon0);
      drawConcentricRangeRings();
      triggerLosViewshedCompute();
      triggerGlideAndAircraftCompute();
    } else if (state.placementMode === 'lf') {
      state.latLf = e.latlng.lat;
      state.lonLf = e.latlng.lng;
      lfMarker.setLatLng([state.latLf, state.lonLf]);
      syncUtmInputs('lf', state.latLf, state.lonLf);
      updateDirectGlideLineGeometry();
      triggerGlideAndAircraftCompute();
    }
  });

  // Map mousemove: Cursor elevation probe + optional hover LOS ray
  map.on('mousemove', (e) => {
    const now = performance.now();
    if (now - state.lastProbeTime > 75 && worker) {
      state.lastProbeTime = now;
      const reqId = ++state.probeReqId;
      worker.postMessage({
        type: 'PROBE_POINT_ELEVATION',
        payload: { reqId, lat: e.latlng.lat, lon: e.latlng.lng }
      });
    }

    if (state.losRayMode === 'hover' && state.raySummaries.length > 0) {
      const distKm = haversineDistanceKm(state.lat0, state.lon0, e.latlng.lat, e.latlng.lng);
      if (distKm >= 0.3) {
        const az = initialBearingDeg(state.lat0, state.lon0, e.latlng.lat, e.latlng.lng);
        updateLosInspectionRay(az, false);
      }
    }
  });

  // Map moveend / zoomend: Recompute Terrain Safety Contours & Buffer with hysteresis
  const onViewportChange = () => {
    if (state.isTerrainAreaLocked) return;
    if (terrainDebounceTimer) clearTimeout(terrainDebounceTimer);
    terrainDebounceTimer = setTimeout(() => {
      triggerTerrainSafetyCompute(false);
    }, 180);
  };
  map.on('moveend', onViewportChange);
  map.on('zoomend', onViewportChange);

  drawConcentricRangeRings();
  updateDirectGlideLineGeometry();
}

/**
 * Draw Concentric Range Rings around Ground Station (50, 100, 150, R_max km)
 */
function drawConcentricRangeRings() {
  if (!rangeRingsGroup) return;
  rangeRingsGroup.clearLayers();
  if (!state.showLosLayer || !state.showRangeRings) return;

  const standardRings = [50, 100, 150];
  const ringDistances = standardRings.filter((r) => r < state.rMaxKm);
  ringDistances.push(state.rMaxKm);

  for (const rKm of ringDistances) {
    const isOuter = rKm === state.rMaxKm;
    L.circle([state.lat0, state.lon0], {
      radius: rKm * 1000.0,
      color: isOuter ? '#38bdf8' : '#64748b',
      weight: isOuter ? 1.4 : 1.0,
      opacity: isOuter ? 0.55 : 0.35,
      fill: false,
      dashArray: isOuter ? '6 5' : '4 6',
      interactive: false
    }).addTo(rangeRingsGroup);

    const labelPt = destinationPoint(state.lat0, state.lon0, rKm, 0);
    L.marker([labelPt.lat, labelPt.lon], {
      interactive: false,
      icon: L.divIcon({
        className: 'ring-label-icon',
        iconSize: null,
        html: `<div class="ring-label-badge" dir="ltr">${rKm} km</div>`
      })
    }).addTo(rangeRingsGroup);
  }
}

function updateDirectGlideLineGeometry() {
  if (!directGlideLineLayer) return;
  if (!state.showGlideLayer) {
    directGlideLineLayer.setLatLngs([]);
    return;
  }
  directGlideLineLayer.setLatLngs([
    [state.latUav, state.lonUav],
    [state.latLf, state.lonLf]
  ]);
}

/**
 * Initialize Unified Web Worker (`js/mission-worker.js`)
 */
function initWorker() {
  worker = new Worker('/js/mission-worker.js?v=1.0', { type: 'module' });

  worker.onmessage = async (e) => {
    const msg = e.data;
    if (!msg || !msg.type) return;

    if (msg.type === 'DECODE_PNG_REQ') {
      const rgbaBuffer = await decodePngOnMainThread(msg.pngBuffer);
      worker.postMessage({ type: 'DECODE_PNG_RES', id: msg.id, rgbaBuffer }, [rgbaBuffer]);
      return;
    }

    if (msg.type === 'LOS_VIEWSHED_RESULT') {
      state.stationGroundM = msg.hGroundM;
      state.raySummaries = msg.raySummaries || [];
      renderDualLosPolygons(msg.primaryPolygonLatLons, msg.safetyPolygonLatLons);
      updateGlobalAndLosReadouts();

      const targetAz =
        state.losRayMode === 'uav'
          ? initialBearingDeg(state.lat0, state.lon0, state.latUav, state.lonUav)
          : state.inspectAzimuthDeg;
      updateLosInspectionRay(targetAz, true);
      return;
    }

    if (msg.type === 'LOS_RAY_PROFILE_RESULT') {
      if (losChart) {
        losChart.setProfile(msg.profile);
      }
      return;
    }

    if (msg.type === 'TERRAIN_SAFETY_RESULT') {
      renderTerrainSafetyLayers(msg);
      return;
    }

    if (msg.type === 'GLIDE_AND_AIRCRAFT_RESULT') {
      state.lfGroundFt = msg.lfGroundFt;
      state.aircraftStatus = msg.aircraftStatus;
      renderGlidePolygons(msg.lfCatchmentPolygonLatLons, msg.uavRingPolygonLatLons);
      updateAircraftSafetyDashboard(msg.aircraftStatus);
      if (glideChart && msg.aircraftStatus?.glide) {
        glideChart.setGlideData(msg.aircraftStatus.glide);
      }
      return;
    }

    if (msg.type === 'PROBE_POINT_RESULT') {
      if (msg.reqId !== state.probeReqId) return;
      const probeEl = document.getElementById('hud-val-probe');
      if (probeEl) {
        probeEl.textContent = `${msg.elevationFt.toLocaleString('en-US')} ft (${msg.elevationM} m)`;
      }
    }
  };
}

async function decodePngOnMainThread(pngArrayBuffer) {
  try {
    const blob = new Blob([pngArrayBuffer], { type: 'image/png' });
    const bmp = await createImageBitmap(blob);
    const canvas = document.getElementById('decode-fallback-canvas');
    const ctx = canvas.getContext('2d', { willReadFrequently: true });
    ctx.clearRect(0, 0, 256, 256);
    ctx.drawImage(bmp, 0, 0, 256, 256);
    bmp.close();
    return ctx.getImageData(0, 0, 256, 256).data.buffer;
  } catch (err) {
    return new Uint8ClampedArray(256 * 256 * 4).buffer;
  }
}

// ---------------------------------------------------------------------------
// Rendering Map Layers from Worker Results
// ---------------------------------------------------------------------------

function renderDualLosPolygons(primaryLatLons, safetyLatLons) {
  if (!state.showLosLayer) {
    losPrimaryHaloLayer.setLatLngs([]);
    losPrimaryPolygonLayer.setLatLngs([]);
    losSafetyPolygonLayer.setLatLngs([]);
    return;
  }

  losPrimaryHaloLayer.setLatLngs(primaryLatLons || []);
  losPrimaryPolygonLayer.setLatLngs(primaryLatLons || []);

  if (state.losSafetyMarginFt > 0 && safetyLatLons && safetyLatLons.length) {
    losSafetyPolygonLayer.setLatLngs(safetyLatLons);
  } else {
    losSafetyPolygonLayer.setLatLngs([]);
  }
}

function renderTerrainSafetyLayers(res) {
  terrainFillGroup.clearLayers();
  terrainOutlineGroup.clearLayers();
  terrainContourGroup.clearLayers();

  const statsEl = document.getElementById('readout-terrain-stats');
  if (res.error) {
    statsEl.textContent = `שגיאה: ${res.error}`;
    return;
  }

  const resPx = res.resolutionMetersPerPx ? `~${Math.round(res.resolutionMetersPerPx)}m/px` : '';
  statsEl.textContent = res.hasIntersections
    ? `${resPx} | ${res.computeTimeMs} ms`
    : `נקי ממכשולים (${res.computeTimeMs} ms)`;

  if (!state.showTerrainLayer || !res.hasIntersections) return;

  // 1. Shaded safety buffer fill (all terrain >= threshold + horizontal safety distance)
  if (res.safetyFillPolygonsLatLon && res.safetyFillPolygonsLatLon.length > 0) {
    L.polygon(res.safetyFillPolygonsLatLon, {
      renderer: canvasRenderer,
      color: 'transparent',
      weight: 0,
      fillColor: '#dc2626',
      fillOpacity: 0.25,
      fillRule: 'evenodd',
      interactive: false
    }).addTo(terrainFillGroup);
  }

  // 2. Safety buffer outer boundary lines
  if (res.safetyOutlineLatLon && res.safetyOutlineLatLon.length > 0) {
    L.polyline(res.safetyOutlineLatLon, {
      renderer: canvasRenderer,
      color: '#fca5a5',
      weight: 1.8,
      opacity: 0.9,
      interactive: false
    }).addTo(terrainOutlineGroup);
  }

  // 3. Continuous red mountain contour lines at (H_UAV - H_Terrain_Clearance)
  if (res.contourLinesLatLon && res.contourLinesLatLon.length > 0) {
    L.polyline(res.contourLinesLatLon, {
      renderer: canvasRenderer,
      color: '#ef233c',
      weight: 2.5,
      opacity: 1.0,
      interactive: false
    }).addTo(terrainContourGroup);
  }
}

function renderGlidePolygons(lfCatchmentLatLons, uavRingLatLons) {
  if (state.showGlideLayer && lfCatchmentLatLons && lfCatchmentLatLons.length) {
    lfGlideHaloLayer.setLatLngs(lfCatchmentLatLons);
    lfGlidePolygonLayer.setLatLngs(lfCatchmentLatLons);
  } else {
    lfGlideHaloLayer.setLatLngs([]);
    lfGlidePolygonLayer.setLatLngs([]);
  }

  if (state.showUavRingLayer && uavRingLatLons && uavRingLatLons.length) {
    if (!map.hasLayer(uavGlideRingLayer)) {
      uavGlideRingLayer.addTo(map);
    }
    uavGlideRingLayer.setLatLngs(uavRingLatLons);
  } else {
    uavGlideRingLayer.setLatLngs([]);
    if (map.hasLayer(uavGlideRingLayer)) {
      map.removeLayer(uavGlideRingLayer);
    }
  }
}

// ---------------------------------------------------------------------------
// Triggering Worker Calculations
// ---------------------------------------------------------------------------

function triggerLosViewshedCompute() {
  if (!worker) return;
  worker.postMessage({
    type: 'COMPUTE_LOS_VIEWSHED',
    payload: {
      lat0: state.lat0,
      lon0: state.lon0,
      hAntM: state.hAntM,
      hUavM: state.hUavFt * FT_TO_M,
      losSafetyMarginM: state.losSafetyMarginFt * FT_TO_M,
      maxRangeKm: state.rMaxKm,
      kFactor: state.kFactor
    }
  });
}

function triggerFastLosAltitudeUpdate() {
  if (!worker) return;
  worker.postMessage({
    type: 'FAST_UPDATE_LOS_ALT',
    payload: {
      hUavM: state.hUavFt * FT_TO_M,
      losSafetyMarginM: state.losSafetyMarginFt * FT_TO_M,
      maxRangeKm: state.rMaxKm
    }
  });
}

function updateLosInspectionRay(azimuthDeg, forceProfile = false) {
  state.inspectAzimuthDeg = ((azimuthDeg % 360) + 360) % 360;

  if (!state.showLosLayer || state.raySummaries.length === 0) {
    losInspectRayLayer.setLatLngs([]);
    losInspectShadowLayer.setLatLngs([]);
    return;
  }

  let bestRay = state.raySummaries[0];
  let minDiff = 360;
  for (const r of state.raySummaries) {
    let diff = Math.abs(r.azimuthDeg - state.inspectAzimuthDeg);
    if (diff > 180) diff = 360 - diff;
    if (diff < minDiff) {
      minDiff = diff;
      bestRay = r;
    }
  }

  const ptEnd = destinationPoint(state.lat0, state.lon0, bestRay.primaryRangeKm, bestRay.azimuthDeg);
  losInspectRayLayer.setLatLngs([
    [state.lat0, state.lon0],
    [ptEnd.lat, ptEnd.lon]
  ]);

  if (bestRay.primaryRangeKm < state.rMaxKm - 0.5) {
    const ptMax = destinationPoint(state.lat0, state.lon0, state.rMaxKm, bestRay.azimuthDeg);
    losInspectShadowLayer.setLatLngs([
      [ptEnd.lat, ptEnd.lon],
      [ptMax.lat, ptMax.lon]
    ]);
  } else {
    losInspectShadowLayer.setLatLngs([]);
  }

  const telemEl = document.getElementById('los-ray-telemetry');
  if (telemEl) {
    telemEl.textContent = `Az: ${bestRay.azimuthDeg.toFixed(1)}° | Max: ${bestRay.primaryRangeKm.toFixed(1)} km | Safe: ${bestRay.safetyRangeKm.toFixed(1)} km`;
  }

  if (forceProfile && worker) {
    worker.postMessage({
      type: 'GET_LOS_RAY_PROFILE',
      payload: {
        azimuthDeg: state.inspectAzimuthDeg,
        hUavM: state.hUavFt * FT_TO_M,
        losSafetyMarginM: state.losSafetyMarginFt * FT_TO_M,
        maxRangeKm: state.rMaxKm
      }
    });
  }
}

function triggerTerrainSafetyCompute(forceRebuildEnvelope = false) {
  if (!worker || !map) return;

  const mapZoom = map.getZoom();
  const bounds = map.getBounds();
  const viewport = {
    west: bounds.getWest(),
    south: bounds.getSouth(),
    east: bounds.getEast(),
    north: bounds.getNorth()
  };

  const currentEnv = state.terrainEnvelope;
  if (state.isTerrainAreaLocked && currentEnv && !forceRebuildEnvelope) {
    return;
  }

  const demZoom =
    state.isTerrainAreaLocked && currentEnv
      ? currentEnv.demZoom
      : selectDemZoomTier(mapZoom, currentEnv?.demZoom ?? null);

  const statsEl = document.getElementById('readout-terrain-stats');
  if (demZoom === null) {
    statsEl.textContent = 'התקרב למפה (Zoom 8+) לחישוב קרקע';
    terrainFillGroup.clearLayers();
    terrainOutlineGroup.clearLayers();
    terrainContourGroup.clearLayers();
    return;
  }

  const safetyMeters = state.terrainHorizontalSafetyM;
  let nextEnv = currentEnv;

  const canReuse =
    !forceRebuildEnvelope &&
    currentEnv !== null &&
    currentEnv.demZoom === demZoom &&
    currentEnv.safetyRangeMeters >= safetyMeters &&
    isViewportInsideBounds(viewport, currentEnv.safeInnerBounds);

  if (!canReuse) {
    if (state.isTerrainAreaLocked && currentEnv) {
      nextEnv = {
        demZoom: currentEnv.demZoom,
        safeInnerBounds: currentEnv.safeInnerBounds,
        fetchBounds: expandBBoxByMeters(currentEnv.safeInnerBounds, safetyMeters),
        safetyRangeMeters: safetyMeters
      };
    } else {
      nextEnv = buildComputationEnvelope(viewport, demZoom, safetyMeters);
    }
    state.terrainEnvelope = nextEnv;
  }

  if (!nextEnv) return;
  statsEl.textContent = 'מחשב קווי גובה ורצועת בטיחות...';

  const reqId = ++state.terrainReqId;
  worker.postMessage({
    type: 'COMPUTE_TERRAIN_SAFETY',
    payload: {
      reqId,
      fetchBounds: nextEnv.fetchBounds,
      demZoom: nextEnv.demZoom,
      hUavFt: state.hUavFt,
      terrainVerticalClearanceFt: state.terrainVerticalClearanceFt,
      terrainHorizontalSafetyM: state.terrainHorizontalSafetyM
    }
  });
}

function triggerGlideAndAircraftCompute() {
  if (!worker) return;
  worker.postMessage({
    type: 'COMPUTE_GLIDE_AND_AIRCRAFT',
    payload: {
      latLf: state.latLf,
      lonLf: state.lonLf,
      latUav: state.latUav,
      lonUav: state.lonUav,
      hUavFt: state.hUavFt,
      lfSafetyMarginFt: state.lfSafetyMarginFt,
      ridgeClearanceFt: state.ridgeClearanceFt,
      batteryTimeMin: state.batteryTimeMin,
      speedRoDPairs: state.speedRoDPairs,
      windLayers: state.windLayers,
      showUavGlideRing: state.showUavRingLayer,
      terrainVerticalClearanceFt: state.terrainVerticalClearanceFt,
      terrainHorizontalSafetyM: state.terrainHorizontalSafetyM,
      losSafetyMarginFt: state.losSafetyMarginFt
    }
  });
}

// ---------------------------------------------------------------------------
// Updating DOM Telemetry & Readouts
// ---------------------------------------------------------------------------

function updateGlobalAndLosReadouts() {
  const hUavM = Math.round(state.hUavFt * FT_TO_M);
  const stationGroundFt = Math.round(state.stationGroundM * M_TO_FT);
  const H0Ft = Math.round((state.stationGroundM + state.hAntM) * M_TO_FT);
  const safetyAltFt = Math.max(0, state.hUavFt - state.losSafetyMarginFt);
  const terrainThreshFt = state.hUavFt - state.terrainVerticalClearanceFt;

  document.getElementById('readout-h-uav-m').textContent = `${hUavM.toLocaleString('en-US')} m`;
  document.getElementById('sidebar-collapsed-summary').textContent = `${state.hUavFt.toLocaleString('en-US')} ft ASL`;

  document.getElementById('readout-station-ground').textContent = `${stationGroundFt.toLocaleString('en-US')} ft ASL`;
  document.getElementById('readout-h0-asl').textContent = `סה"כ: ${H0Ft.toLocaleString('en-US')} ft ASL`;
  document.getElementById('readout-los-safety-alt').textContent = `מעטפת ב-${safetyAltFt.toLocaleString('en-US')} ft`;
  document.getElementById('leg-los-max-ft').textContent = `${state.hUavFt.toLocaleString('en-US')} ft`;
  document.getElementById('leg-los-safe-ft').textContent = `${safetyAltFt.toLocaleString('en-US')} ft`;

  document.getElementById('readout-terrain-threshold').textContent = `סף קרקע: ${terrainThreshFt.toLocaleString('en-US')} ft`;
  document.getElementById('note-terrain-thresh-ft').textContent = `${terrainThreshFt.toLocaleString('en-US')} ft`;
  document.getElementById('readout-terrain-h-km').textContent = `${(state.terrainHorizontalSafetyM / 1000).toFixed(2)} km`;

  // Highlight active preset pills
  document.querySelectorAll('[data-alt-preset]').forEach((btn) => {
    btn.classList.toggle('active', Number(btn.dataset.altPreset) === state.hUavFt);
  });
  document.querySelectorAll('[data-terr-v]').forEach((btn) => {
    btn.classList.toggle('active', Number(btn.dataset.terrV) === state.terrainVerticalClearanceFt);
  });
  document.querySelectorAll('[data-terr-h]').forEach((btn) => {
    btn.classList.toggle('active', Number(btn.dataset.terrH) === state.terrainHorizontalSafetyM);
  });
}

function setStatusClass(el, statusClass) {
  if (!el) return;
  el.classList.remove('status-ok', 'status-warn', 'status-danger');
  el.classList.add(statusClass);
}

function updateAircraftSafetyDashboard(st) {
  if (!st) return;

  document.getElementById('readout-lf-ground').textContent = `${st.glide.lfGroundFt.toLocaleString('en-US')} ft ASL`;
  document.getElementById('glide-chart-summary').textContent = `${st.glide.distToLfKm.toFixed(1)} km | ${st.glide.bearingToLfDeg.toFixed(0)}°`;

  // 1. Terrain Card & Bottom HUD Chip
  const terrainClass = st.isTerrainSafe
    ? 'status-ok'
    : st.isTerrainVerticalOk
    ? 'status-warn'
    : 'status-danger';
  setStatusClass(document.getElementById('card-status-terrain'), terrainClass);
  setStatusClass(document.getElementById('hud-chip-terrain'), terrainClass);

  document.getElementById('badge-status-terrain').textContent = st.isTerrainSafe
    ? 'בטוח'
    : !st.isTerrainVerticalOk
    ? 'חריגת גובה!'
    : 'בתוך חיץ אופקי';

  document.getElementById('val-uav-agl').textContent = `${st.uavAglFt.toLocaleString('en-US')} ft`;
  document.getElementById('val-uav-ground').textContent = `${st.uavGroundFt.toLocaleString('en-US')} ft ASL`;
  const vSign = st.verticalTerrainMarginFt >= 0 ? '+' : '';
  document.getElementById('val-terrain-v-margin').textContent = `${vSign}${st.verticalTerrainMarginFt.toLocaleString('en-US')} ft`;
  document.getElementById('val-terrain-h-dist').textContent =
    st.horizontalDistToHazardM >= 15000
      ? '> 15 km (נקי)'
      : `${st.horizontalDistToHazardM.toLocaleString('en-US')} m`;
  document.getElementById('hud-val-terrain').textContent = `${st.uavAglFt.toLocaleString('en-US')} ft AGL`;

  // 2. LOS Communication Card & Bottom HUD Chip
  const losClass = st.isLosSafetyOk
    ? 'status-ok'
    : st.isLosMaxOk
    ? 'status-warn'
    : 'status-danger';
  setStatusClass(document.getElementById('card-status-los'), losClass);
  setStatusClass(document.getElementById('hud-chip-los'), losClass);

  document.getElementById('badge-status-los').textContent = st.isLosSafetyOk
    ? 'מעל מרווח בטיחות'
    : st.isLosMaxOk
    ? 'מתחת למרווח בטיחות'
    : 'אין קו ראייה!';

  const losSign = st.heightAboveLosFt >= 0 ? '+' : '';
  const losSafeSign = st.losSafetyMarginDiffFt >= 0 ? '+' : '';
  document.getElementById('val-uav-above-los').textContent = `${losSign}${st.heightAboveLosFt.toLocaleString('en-US')} ft`;
  document.getElementById('val-uav-min-los').textContent = `${st.minLosAslFt.toLocaleString('en-US')} ft ASL`;
  document.getElementById('val-los-safety-diff').textContent = `${losSafeSign}${st.losSafetyMarginDiffFt.toLocaleString('en-US')} ft`;
  document.getElementById('val-uav-station-vec').textContent = `${st.distFromStationKm.toFixed(1)} km | ${st.azimuthFromStationDeg.toFixed(0)}°`;
  document.getElementById('hud-val-los').textContent = `${losSign}${st.heightAboveLosFt.toLocaleString('en-US')} ft`;

  // 3. Emergency Glide Card, Direct Map Line & Bottom HUD Chip
  const g = st.glide;
  const glideClass = g.canGlideToLf ? 'status-ok' : 'status-danger';
  setStatusClass(document.getElementById('card-status-glide'), glideClass);
  setStatusClass(document.getElementById('hud-chip-glide'), glideClass);

  let glideBadgeText = 'בטווח גלישה בטוח';
  if (!g.canGlideToLf) {
    if (g.statusCode === 'BLOCKED_BY_RIDGE') glideBadgeText = 'חסום ע"י רכס בדרך!';
    else if (g.statusCode === 'BATTERY_LIMIT') glideBadgeText = 'חריגת זמן סוללה!';
    else if (g.statusCode === 'TOO_LOW_FOR_FIELD') glideBadgeText = 'נמוך מגובה השדה!';
    else glideBadgeText = 'מחוץ לטווח גלישה!';
  }
  document.getElementById('badge-status-glide').textContent = glideBadgeText;

  document.getElementById('val-glide-dist-compare').textContent = `${g.distToLfKm.toFixed(1)} / ${g.maxGlideTowardsLfKm.toFixed(1)} km`;
  const arrSign = g.arrivalMarginFt >= 0 ? '+' : '';
  document.getElementById('val-glide-arrival-margin').textContent = `${arrSign}${g.arrivalMarginFt.toLocaleString('en-US')} ft`;
  document.getElementById('val-glide-speed-time').textContent = `${g.optimalSpeedKts.toFixed(0)} kts | ${g.timeToLfMin.toFixed(1)} min`;
  const ridgeSign = g.minRidgeMarginFt >= 0 ? '+' : '';
  document.getElementById('val-glide-ridge-margin').textContent = `${ridgeSign}${g.minRidgeMarginFt.toLocaleString('en-US')} ft`;

  document.getElementById('hud-val-glide').textContent = g.canGlideToLf
    ? `✓ בטווח (${arrSign}${g.arrivalMarginFt} ft)`
    : `✕ ${glideBadgeText}`;

  // Style direct glide line on map based on glide feasibility
  if (directGlideLineLayer) {
    directGlideLineLayer.setStyle({
      color: g.canGlideToLf ? '#22c55e' : '#ef4444',
      weight: g.canGlideToLf ? 2.4 : 2.6,
      dashArray: g.canGlideToLf ? '6 4' : '4 5'
    });
  }

  // Show critical ridge blocker dot on direct glide path if blocked by terrain ridge
  if (!g.canGlideToLf && g.statusCode === 'BLOCKED_BY_RIDGE' && g.criticalRidgeLat) {
    if (!criticalRidgeMarker) {
      criticalRidgeMarker = L.circleMarker([g.criticalRidgeLat, g.criticalRidgeLon], {
        radius: 6,
        color: '#090e14',
        weight: 2,
        fillColor: '#ef4444',
        fillOpacity: 1.0,
        interactive: false
      }).addTo(map);
    } else {
      criticalRidgeMarker.setLatLng([g.criticalRidgeLat, g.criticalRidgeLon]);
    }
  } else if (criticalRidgeMarker) {
    map.removeLayer(criticalRidgeMarker);
    criticalRidgeMarker = null;
  }

  // 4. Overall Combined Status Banner & HUD Badge
  const overallClass =
    terrainClass === 'status-danger' ||
    losClass === 'status-danger' ||
    glideClass === 'status-danger'
      ? 'status-danger'
      : terrainClass === 'status-warn' || losClass === 'status-warn'
      ? 'status-warn'
      : 'status-ok';

  const bannerEl = document.getElementById('uav-overall-banner');
  const hudBadgeEl = document.getElementById('hud-overall-badge');
  setStatusClass(bannerEl, overallClass);
  setStatusClass(hudBadgeEl, overallClass);

  if (overallClass === 'status-ok') {
    document.getElementById('uav-banner-title').textContent = '✓ המטוס באזור טיסה בטוח';
    document.getElementById('uav-banner-sub').textContent =
      'עומד בדרישות מרווח קרקע, מרווח בטיחות תקשורת ופוטנציאל גלישה לשדה';
    hudBadgeEl.textContent = '✓ בטוח לטיסה';
  } else if (overallClass === 'status-warn') {
    document.getElementById('uav-banner-title').textContent = '⚠️ התרעת מרווח בטיחות';
    document.getElementById('uav-banner-sub').textContent =
      'בקו ראייה ובטווח גלישה, אך מתחת למרווח הבטיחות הנוסף שהוגדר';
    hudBadgeEl.textContent = '⚠️ התרעת מרווח';
  } else {
    const reasons = [];
    if (!st.isTerrainSafe) reasons.push('קרבה מסוכנת לקרקע');
    if (!st.isLosMaxOk) reasons.push('איבוד תקשורת (LOS)');
    if (!g.canGlideToLf) reasons.push('מחוץ לפוטנציאל גלישה לשדה');
    document.getElementById('uav-banner-title').textContent = '✕ חריגת בטיחות במיקום המטוס';
    document.getElementById('uav-banner-sub').textContent = reasons.join(' • ');
    hudBadgeEl.textContent = '✕ חריגת בטיחות';
  }
}

// ---------------------------------------------------------------------------
// UTM Coordinate Synchronization & Dynamic Glide Tables (IAS/RoD + Wind)
// ---------------------------------------------------------------------------

function syncUtmInputs(prefix, lat, lon) {
  const utm = latLonToUtm(lat, lon);
  const zEl = document.getElementById(`input-${prefix}-zone`);
  const eEl = document.getElementById(`input-${prefix}-e`);
  const nEl = document.getElementById(`input-${prefix}-n`);
  if (zEl) zEl.value = `${utm.zone}${utm.hemisphere}`;
  if (eEl) eEl.value = utm.easting;
  if (nEl) nEl.value = utm.northing;
}

function readUtmInputsToLatLon(prefix) {
  const rawZone = document.getElementById(`input-${prefix}-zone`).value.trim().toUpperCase();
  const match = rawZone.match(/^(\d{1,2})\s*([NS]?)$/);
  const zoneNum = match ? parseInt(match[1], 10) : 36;
  const hemi = match && match[2] === 'S' ? 'S' : 'N';
  const easting = parseFloat(document.getElementById(`input-${prefix}-e`).value);
  const northing = parseFloat(document.getElementById(`input-${prefix}-n`).value);
  return utmToLatLon(zoneNum, hemi, easting, northing);
}

function renderDynamicPolarTable() {
  const container = document.getElementById('polar-rows-container');
  container.innerHTML = '';

  state.speedRoDPairs.forEach((pair, idx) => {
    const row = document.createElement('div');
    row.className = 'dyn-row';
    row.innerHTML = `
      <input type="number" class="dyn-input" value="${pair.speed}" min="1" max="300" step="1" data-polar-idx="${idx}" data-field="speed" />
      <input type="number" class="dyn-input" value="${pair.rod}" min="1" max="10000" step="25" data-polar-idx="${idx}" data-field="rod" />
      <button type="button" class="btn-del-row" data-del-polar="${idx}" ${state.speedRoDPairs.length <= 2 ? 'disabled' : ''} title="מחק שורה">✕</button>
    `;
    container.appendChild(row);
  });

  container.querySelectorAll('.dyn-input').forEach((inp) => {
    inp.addEventListener('change', (e) => {
      const idx = Number(e.target.dataset.polarIdx);
      const field = e.target.dataset.field;
      const val = Math.max(1, parseFloat(e.target.value) || 1);
      state.speedRoDPairs[idx][field] = val;
      saveGlideConfig();
      triggerGlideAndAircraftCompute();
    });
  });

  container.querySelectorAll('[data-del-polar]').forEach((btn) => {
    btn.addEventListener('click', () => {
      if (state.speedRoDPairs.length <= 2) return;
      const idx = Number(btn.dataset.delPolar);
      state.speedRoDPairs.splice(idx, 1);
      saveGlideConfig();
      renderDynamicPolarTable();
      triggerGlideAndAircraftCompute();
    });
  });
}

function renderDynamicWindTable() {
  const container = document.getElementById('wind-rows-container');
  container.innerHTML = '';

  state.windLayers.forEach((layer, idx) => {
    const row = document.createElement('div');
    row.className = 'dyn-row wind-row';
    row.innerHTML = `
      <input type="number" class="dyn-input" value="${layer.altitudeKft}" min="0" max="45" step="1" data-wind-idx="${idx}" data-field="altitudeKft" />
      <input type="number" class="dyn-input" value="${layer.speed}" min="0" max="200" step="1" data-wind-idx="${idx}" data-field="speed" />
      <input type="number" class="dyn-input" value="${layer.direction}" min="0" max="360" step="5" data-wind-idx="${idx}" data-field="direction" />
      <button type="button" class="btn-del-row" data-del-wind="${idx}" ${state.windLayers.length <= 1 ? 'disabled' : ''} title="מחק שכבת רוח">✕</button>
    `;
    container.appendChild(row);
  });

  container.querySelectorAll('.dyn-input').forEach((inp) => {
    inp.addEventListener('change', (e) => {
      const idx = Number(e.target.dataset.windIdx);
      const field = e.target.dataset.field;
      let val = parseFloat(e.target.value) || 0;
      if (field === 'direction') {
        val = ((val % 360) + 360) % 360;
      } else {
        val = Math.max(0, val);
      }
      state.windLayers[idx][field] = val;
      saveGlideConfig();
      triggerGlideAndAircraftCompute();
    });
  });

  container.querySelectorAll('[data-del-wind]').forEach((btn) => {
    btn.addEventListener('click', () => {
      if (state.windLayers.length <= 1) return;
      const idx = Number(btn.dataset.delWind);
      state.windLayers.splice(idx, 1);
      saveGlideConfig();
      renderDynamicWindTable();
      triggerGlideAndAircraftCompute();
    });
  });
}

// ---------------------------------------------------------------------------
// UI Event Bindings
// ---------------------------------------------------------------------------

function setPlacementMode(mode) {
  state.placementMode = mode;
  document.body.classList.toggle('placement-mode-active', mode !== 'nav');

  document.getElementById('btn-mode-nav').classList.toggle('active', mode === 'nav');
  document.getElementById('btn-mode-uav').classList.toggle('active', mode === 'uav');
  document.getElementById('btn-mode-station').classList.toggle('active', mode === 'station');
  document.getElementById('btn-mode-lf').classList.toggle('active', mode === 'lf');

  document.getElementById('btn-place-uav-inline').classList.toggle('active', mode === 'uav');
  document.getElementById('btn-place-station-inline').classList.toggle('active', mode === 'station');
  document.getElementById('btn-place-lf-inline').classList.toggle('active', mode === 'lf');
}

function switchCategoryTab(tabName) {
  state.activeTab = tabName;
  document.querySelectorAll('.cat-tab').forEach((btn) => {
    const isMatch = btn.dataset.tab === tabName;
    btn.classList.toggle('active', isMatch);
    btn.setAttribute('aria-selected', isMatch ? 'true' : 'false');
  });

  document.querySelectorAll('.cat-panel').forEach((panel) => {
    panel.classList.toggle('active', panel.id === `panel-${tabName}`);
  });

  if (tabName === 'los' && losChart) {
    setTimeout(() => losChart.render(), 40);
  } else if (tabName === 'glide' && glideChart) {
    setTimeout(() => glideChart.render(), 40);
  }
}

function onGlobalAltitudeChange(newAltFt) {
  const num = parseFloat(newAltFt);
  if (!Number.isFinite(num)) return;
  state.hUavFt = Math.max(100, Math.min(40000, Math.round(num)));
  document.getElementById('input-h-uav-ft').value = state.hUavFt;
  document.getElementById('slider-h-uav-ft').value = Math.min(35000, state.hUavFt);
  updateGlobalAndLosReadouts();

  // 1. Instant 60 FPS update of Dual LOS Viewshed Polygons (< 1.5 ms)
  triggerFastLosAltitudeUpdate();

  // 2. Debounced update of Glide Envelopes + Aircraft Status (~40 ms)
  if (glideDebounceTimer) clearTimeout(glideDebounceTimer);
  glideDebounceTimer = setTimeout(() => {
    triggerGlideAndAircraftCompute();
  }, 40);

  // 3. Debounced update of Terrain Obstacle Contours & Safety Buffer (~110 ms)
  if (terrainDebounceTimer) clearTimeout(terrainDebounceTimer);
  terrainDebounceTimer = setTimeout(() => {
    triggerTerrainSafetyCompute(true);
  }, 110);
}

function bindUIEvents() {
  // Sidebar Collapse / Expand
  const sidebarEl = document.getElementById('mission-sidebar');
  const toggleBtn = document.getElementById('btn-toggle-sidebar');
  document.getElementById('sidebar-header').addEventListener('click', () => {
    state.sidebarCollapsed = !state.sidebarCollapsed;
    sidebarEl.classList.toggle('is-collapsed', state.sidebarCollapsed);
    toggleBtn.textContent = state.sidebarCollapsed ? 'הרחב ▾' : 'צמצם ▴';
  });

  // Category Tabs & Bottom HUD Chip Shortcuts
  document.querySelectorAll('.cat-tab').forEach((btn) => {
    btn.addEventListener('click', () => switchCategoryTab(btn.dataset.tab));
  });
  document.getElementById('hud-chip-terrain').addEventListener('click', () => {
    if (state.sidebarCollapsed) document.getElementById('sidebar-header').click();
    switchCategoryTab('terrain');
  });
  document.getElementById('hud-chip-los').addEventListener('click', () => {
    if (state.sidebarCollapsed) document.getElementById('sidebar-header').click();
    switchCategoryTab('los');
  });
  document.getElementById('hud-chip-glide').addEventListener('click', () => {
    if (state.sidebarCollapsed) document.getElementById('sidebar-header').click();
    switchCategoryTab('glide');
  });

  // Placement Mode Toolbar & Inline Placement Buttons
  document.getElementById('btn-mode-nav').addEventListener('click', () => setPlacementMode('nav'));
  document.getElementById('btn-mode-uav').addEventListener('click', () =>
    setPlacementMode(state.placementMode === 'uav' ? 'nav' : 'uav')
  );
  document.getElementById('btn-mode-station').addEventListener('click', () =>
    setPlacementMode(state.placementMode === 'station' ? 'nav' : 'station')
  );
  document.getElementById('btn-mode-lf').addEventListener('click', () =>
    setPlacementMode(state.placementMode === 'lf' ? 'nav' : 'lf')
  );

  document.getElementById('btn-place-uav-inline').addEventListener('click', () =>
    setPlacementMode(state.placementMode === 'uav' ? 'nav' : 'uav')
  );
  document.getElementById('btn-place-station-inline').addEventListener('click', () =>
    setPlacementMode(state.placementMode === 'station' ? 'nav' : 'station')
  );
  document.getElementById('btn-place-lf-inline').addEventListener('click', () =>
    setPlacementMode(state.placementMode === 'lf' ? 'nav' : 'lf')
  );

  // Layer Visibility Pills
  document.getElementById('btn-vis-los').addEventListener('click', (e) => {
    state.showLosLayer = !state.showLosLayer;
    e.currentTarget.classList.toggle('active', state.showLosLayer);
    drawConcentricRangeRings();
    triggerFastLosAltitudeUpdate();
  });

  document.getElementById('btn-vis-terrain').addEventListener('click', (e) => {
    state.showTerrainLayer = !state.showTerrainLayer;
    e.currentTarget.classList.toggle('active', state.showTerrainLayer);
    if (!state.showTerrainLayer) {
      terrainFillGroup.clearLayers();
      terrainOutlineGroup.clearLayers();
      terrainContourGroup.clearLayers();
    } else {
      triggerTerrainSafetyCompute(true);
    }
  });

  document.getElementById('btn-vis-glide').addEventListener('click', (e) => {
    state.showGlideLayer = !state.showGlideLayer;
    e.currentTarget.classList.toggle('active', state.showGlideLayer);
    updateDirectGlideLineGeometry();
    triggerGlideAndAircraftCompute();
  });

  const syncUavRingToggle = (enabled) => {
    state.showUavRingLayer = enabled;
    document.getElementById('btn-vis-uav-ring').classList.toggle('active', enabled);
    document.getElementById('chk-uav-glide-ring').checked = enabled;
    triggerGlideAndAircraftCompute();
  };
  document.getElementById('btn-vis-uav-ring').addEventListener('click', () =>
    syncUavRingToggle(!state.showUavRingLayer)
  );
  document.getElementById('chk-uav-glide-ring').addEventListener('change', (e) =>
    syncUavRingToggle(e.target.checked)
  );

  // Base Map Switcher (Topographic vs Orthophoto)
  document.getElementById('btn-map-topo').addEventListener('click', () => {
    state.mapType = 'topo';
    if (map.hasLayer(orthoLayerGroup)) map.removeLayer(orthoLayerGroup);
    if (!map.hasLayer(topoLayerGroup)) topoLayerGroup.addTo(map);
    document.getElementById('btn-map-topo').classList.add('active');
    document.getElementById('btn-map-ortho').classList.remove('active');
  });

  document.getElementById('btn-map-ortho').addEventListener('click', () => {
    state.mapType = 'ortho';
    if (map.hasLayer(topoLayerGroup)) map.removeLayer(topoLayerGroup);
    if (!map.hasLayer(orthoLayerGroup)) orthoLayerGroup.addTo(map);
    document.getElementById('btn-map-ortho').classList.add('active');
    document.getElementById('btn-map-topo').classList.remove('active');
  });

  // Global Aircraft Altitude Controls
  document.getElementById('slider-h-uav-ft').addEventListener('input', (e) =>
    onGlobalAltitudeChange(e.target.value)
  );
  document.getElementById('input-h-uav-ft').addEventListener('change', (e) =>
    onGlobalAltitudeChange(e.target.value)
  );
  document.getElementById('btn-alt-minus').addEventListener('click', () =>
    onGlobalAltitudeChange(state.hUavFt - 200)
  );
  document.getElementById('btn-alt-plus').addEventListener('click', () =>
    onGlobalAltitudeChange(state.hUavFt + 200)
  );
  document.querySelectorAll('[data-alt-preset]').forEach((btn) => {
    btn.addEventListener('click', () => onGlobalAltitudeChange(btn.dataset.altPreset));
  });

  // UTM Inputs for Aircraft, Station, Landing Field
  ['uav', 'station', 'lf'].forEach((prefix) => {
    ['zone', 'e', 'n'].forEach((field) => {
      document.getElementById(`input-${prefix}-${field}`).addEventListener('change', () => {
        const coords = readUtmInputsToLatLon(prefix);
        if (!coords) return;
        if (prefix === 'uav') {
          state.latUav = coords.lat;
          state.lonUav = coords.lon;
          uavMarker.setLatLng([state.latUav, state.lonUav]);
          map.panTo([state.latUav, state.lonUav]);
          updateDirectGlideLineGeometry();
          triggerGlideAndAircraftCompute();
        } else if (prefix === 'station') {
          state.lat0 = coords.lat;
          state.lon0 = coords.lon;
          stationMarker.setLatLng([state.lat0, state.lon0]);
          map.panTo([state.lat0, state.lon0]);
          drawConcentricRangeRings();
          triggerLosViewshedCompute();
          triggerGlideAndAircraftCompute();
        } else if (prefix === 'lf') {
          state.latLf = coords.lat;
          state.lonLf = coords.lon;
          lfMarker.setLatLng([state.latLf, state.lonLf]);
          map.panTo([state.latLf, state.lonLf]);
          updateDirectGlideLineGeometry();
          triggerGlideAndAircraftCompute();
        }
      });
    });
  });

  // GPS Station Button & Copy Station to Landing Field Button
  document.getElementById('btn-gps-station').addEventListener('click', () => {
    if (!navigator.geolocation) return;
    navigator.geolocation.getCurrentPosition(
      (pos) => {
        state.lat0 = pos.coords.latitude;
        state.lon0 = pos.coords.longitude;
        stationMarker.setLatLng([state.lat0, state.lon0]);
        map.panTo([state.lat0, state.lon0]);
        syncUtmInputs('station', state.lat0, state.lon0);
        drawConcentricRangeRings();
        triggerLosViewshedCompute();
        triggerGlideAndAircraftCompute();
      },
      () => {}
    );
  });

  document.getElementById('btn-copy-station-to-lf').addEventListener('click', () => {
    state.latLf = state.lat0;
    state.lonLf = state.lon0;
    lfMarker.setLatLng([state.latLf, state.lonLf]);
    syncUtmInputs('lf', state.latLf, state.lonLf);
    updateDirectGlideLineGeometry();
    triggerGlideAndAircraftCompute();
  });

  // Tab 2 (LOS) Specific Controls
  const onLosSafetyChange = (val) => {
    const num = parseFloat(val);
    if (!Number.isFinite(num)) return;
    state.losSafetyMarginFt = Math.max(0, Math.min(25000, Math.round(num)));
    document.getElementById('input-los-safety-ft').value = state.losSafetyMarginFt;
    document.getElementById('slider-los-safety-ft').value = Math.min(15000, state.losSafetyMarginFt);
    updateGlobalAndLosReadouts();
    triggerFastLosAltitudeUpdate();
    if (glideDebounceTimer) clearTimeout(glideDebounceTimer);
    glideDebounceTimer = setTimeout(() => triggerGlideAndAircraftCompute(), 60);
  };
  document.getElementById('slider-los-safety-ft').addEventListener('input', (e) =>
    onLosSafetyChange(e.target.value)
  );
  document.getElementById('input-los-safety-ft').addEventListener('change', (e) =>
    onLosSafetyChange(e.target.value)
  );

  let antTimer = null;
  const onAntChange = (val) => {
    const num = parseFloat(val);
    if (!Number.isFinite(num)) return;
    state.hAntM = Math.max(0, Math.min(500, Math.round(num)));
    document.getElementById('input-h-ant').value = state.hAntM;
    document.getElementById('slider-h-ant').value = Math.min(150, state.hAntM);
    updateGlobalAndLosReadouts();
    if (antTimer) clearTimeout(antTimer);
    antTimer = setTimeout(() => {
      triggerLosViewshedCompute();
      triggerGlideAndAircraftCompute();
    }, 90);
  };
  document.getElementById('slider-h-ant').addEventListener('input', (e) => onAntChange(e.target.value));
  document.getElementById('input-h-ant').addEventListener('change', (e) => onAntChange(e.target.value));

  const onRangeMaxChange = (val) => {
    const num = parseFloat(val);
    if (!Number.isFinite(num)) return;
    state.rMaxKm = Math.max(10, Math.min(200, Math.round(num)));
    document.getElementById('input-r-max').value = state.rMaxKm;
    document.getElementById('slider-r-max').value = state.rMaxKm;
    drawConcentricRangeRings();
    triggerFastLosAltitudeUpdate();
  };
  document.getElementById('slider-r-max').addEventListener('input', (e) =>
    onRangeMaxChange(e.target.value)
  );
  document.getElementById('input-r-max').addEventListener('change', (e) =>
    onRangeMaxChange(e.target.value)
  );

  document.getElementById('chk-show-range-rings').addEventListener('change', (e) => {
    state.showRangeRings = e.target.checked;
    drawConcentricRangeRings();
  });

  document.getElementById('btn-k-rf').addEventListener('click', () => {
    if (state.curvatureMode === 'rf') return;
    state.curvatureMode = 'rf';
    state.kFactor = 4 / 3;
    document.getElementById('btn-k-rf').classList.add('active');
    document.getElementById('btn-k-opt').classList.remove('active');
    triggerLosViewshedCompute();
    triggerGlideAndAircraftCompute();
  });

  document.getElementById('btn-k-opt').addEventListener('click', () => {
    if (state.curvatureMode === 'opt') return;
    state.curvatureMode = 'opt';
    state.kFactor = 1.0;
    document.getElementById('btn-k-opt').classList.add('active');
    document.getElementById('btn-k-rf').classList.remove('active');
    triggerLosViewshedCompute();
    triggerGlideAndAircraftCompute();
  });

  document.getElementById('btn-ray-to-uav').addEventListener('click', () => {
    state.losRayMode = 'uav';
    document.getElementById('btn-ray-to-uav').classList.add('active');
    document.getElementById('btn-ray-hover').classList.remove('active');
    const az = initialBearingDeg(state.lat0, state.lon0, state.latUav, state.lonUav);
    updateLosInspectionRay(az, true);
  });

  document.getElementById('btn-ray-hover').addEventListener('click', () => {
    state.losRayMode = 'hover';
    document.getElementById('btn-ray-hover').classList.add('active');
    document.getElementById('btn-ray-to-uav').classList.remove('active');
  });

  // Tab 3 (Terrain Safety) Specific Controls
  const onTerrainVerticalChange = (val) => {
    const num = parseFloat(val);
    if (!Number.isFinite(num)) return;
    state.terrainVerticalClearanceFt = Math.max(0, Math.min(20000, Math.round(num)));
    document.getElementById('input-terrain-v-ft').value = state.terrainVerticalClearanceFt;
    document.getElementById('slider-terrain-v-ft').value = Math.min(
      10000,
      state.terrainVerticalClearanceFt
    );
    updateGlobalAndLosReadouts();
    if (terrainDebounceTimer) clearTimeout(terrainDebounceTimer);
    terrainDebounceTimer = setTimeout(() => {
      triggerTerrainSafetyCompute(true);
      triggerGlideAndAircraftCompute();
    }, 100);
  };
  document.getElementById('slider-terrain-v-ft').addEventListener('input', (e) =>
    onTerrainVerticalChange(e.target.value)
  );
  document.getElementById('input-terrain-v-ft').addEventListener('change', (e) =>
    onTerrainVerticalChange(e.target.value)
  );
  document.querySelectorAll('[data-terr-v]').forEach((btn) => {
    btn.addEventListener('click', () => onTerrainVerticalChange(btn.dataset.terrV));
  });

  const onTerrainHorizontalChange = (val) => {
    const num = parseFloat(val);
    if (!Number.isFinite(num)) return;
    state.terrainHorizontalSafetyM = Math.max(0, Math.min(15000, Math.round(num)));
    document.getElementById('input-terrain-h-m').value = state.terrainHorizontalSafetyM;
    document.getElementById('slider-terrain-h-m').value = Math.min(
      5000,
      state.terrainHorizontalSafetyM
    );
    updateGlobalAndLosReadouts();
    if (terrainDebounceTimer) clearTimeout(terrainDebounceTimer);
    terrainDebounceTimer = setTimeout(() => {
      triggerTerrainSafetyCompute(true);
      triggerGlideAndAircraftCompute();
    }, 100);
  };
  document.getElementById('slider-terrain-h-m').addEventListener('input', (e) =>
    onTerrainHorizontalChange(e.target.value)
  );
  document.getElementById('input-terrain-h-m').addEventListener('change', (e) =>
    onTerrainHorizontalChange(e.target.value)
  );
  document.querySelectorAll('[data-terr-h]').forEach((btn) => {
    btn.addEventListener('click', () => onTerrainHorizontalChange(btn.dataset.terrH));
  });

  document.getElementById('btn-lock-terrain-area').addEventListener('click', (e) => {
    state.isTerrainAreaLocked = !state.isTerrainAreaLocked;
    const btn = e.currentTarget;
    btn.classList.toggle('active', state.isTerrainAreaLocked);
    btn.textContent = state.isTerrainAreaLocked ? '🔒 גזרת חישוב נעולה' : '🔓 נעל גזרת חישוב';

    if (state.isTerrainAreaLocked && state.terrainEnvelope) {
      const b = state.terrainEnvelope.safeInnerBounds;
      terrainLockedBoxLayer.setLatLngs([
        [b.south, b.west],
        [b.south, b.east],
        [b.north, b.east],
        [b.north, b.west],
        [b.south, b.west]
      ]);
    } else {
      terrainLockedBoxLayer.setLatLngs([]);
      triggerTerrainSafetyCompute(false);
    }
  });

  // Tab 4 (Emergency Glide) Specific Controls
  document.getElementById('input-lf-safety-ft').addEventListener('change', (e) => {
    state.lfSafetyMarginFt = Math.max(0, parseFloat(e.target.value) || 0);
    saveGlideConfig();
    triggerGlideAndAircraftCompute();
  });

  document.getElementById('input-ridge-clear-ft').addEventListener('change', (e) => {
    state.ridgeClearanceFt = Math.max(0, parseFloat(e.target.value) || 0);
    saveGlideConfig();
    triggerGlideAndAircraftCompute();
  });

  document.getElementById('input-battery-min').addEventListener('change', (e) => {
    state.batteryTimeMin = Math.max(1, parseFloat(e.target.value) || 1);
    saveGlideConfig();
    triggerGlideAndAircraftCompute();
  });

  document.getElementById('btn-add-polar-row').addEventListener('click', () => {
    const last = state.speedRoDPairs[state.speedRoDPairs.length - 1] || { speed: 60, rod: 600 };
    state.speedRoDPairs.push({ speed: last.speed + 10, rod: last.rod + 250 });
    saveGlideConfig();
    renderDynamicPolarTable();
    triggerGlideAndAircraftCompute();
  });

  document.getElementById('btn-add-wind-row').addEventListener('click', () => {
    const last = state.windLayers[state.windLayers.length - 1] || {
      altitudeKft: 0,
      speed: 10,
      direction: 270
    };
    state.windLayers.push({
      altitudeKft: last.altitudeKft + 5,
      speed: last.speed,
      direction: last.direction
    });
    saveGlideConfig();
    renderDynamicWindTable();
    triggerGlideAndAircraftCompute();
  });
}

// ---------------------------------------------------------------------------
// Bootstrap Application
// ---------------------------------------------------------------------------

function bootstrap() {
  loadSavedGlideConfig();

  document.getElementById('input-lf-safety-ft').value = state.lfSafetyMarginFt;
  document.getElementById('input-ridge-clear-ft').value = state.ridgeClearanceFt;
  document.getElementById('input-battery-min').value = state.batteryTimeMin;

  initMap();
  initWorker();

  losChart = new LosChartRenderer(document.getElementById('los-chart-canvas'));
  glideChart = new GlideChartRenderer(document.getElementById('glide-chart-canvas'));

  syncUtmInputs('uav', state.latUav, state.lonUav);
  syncUtmInputs('station', state.lat0, state.lon0);
  syncUtmInputs('lf', state.latLf, state.lonLf);

  renderDynamicPolarTable();
  renderDynamicWindTable();
  updateGlobalAndLosReadouts();
  bindUIEvents();

  // Launch initial calculations across all 3 engines
  triggerLosViewshedCompute();
  triggerTerrainSafetyCompute(true);
  triggerGlideAndAircraftCompute();

  if ('serviceWorker' in navigator) {
    window.addEventListener('load', () => {
      navigator.serviceWorker.register('/sw.js').catch(() => {});
    });
  }
}

document.addEventListener('DOMContentLoaded', bootstrap);
