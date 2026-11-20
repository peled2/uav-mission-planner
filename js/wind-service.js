/**
 * UAV Mission Planner - Real-Time & Forecast Wind Profiles by Altitude (per 1,000 ft)
 *
 * Fetches atmospheric pressure-level winds from Open-Meteo Forecast API:
 * Levels: 1000, 975, 950, 925, 900, 850, 800, 700, 600, 500, 400, 300, 250, 200 hPa
 * Plus surface 10m wind & geopotential heights (meters ASL).
 * Converts and interpolates exact wind speed (knots) and direction (degrees FROM)
 * for every 1,000 ft (0 kft, 1 kft, 2 kft, ..., up to maxKft).
 */

import { M_TO_FT, linearInterpolate, interpolateAngle } from './mission-math.js';

const PRESSURE_LEVELS_HPA = [
  1000, 975, 950, 925, 900, 850, 800, 700, 600, 500, 400, 300, 250, 200
];

// Standard Atmosphere fallback altitudes (ft ASL) if geopotential height is missing
const STD_ALT_FT_BY_HPA = {
  1000: 364,
  975: 1050,
  950: 1773,
  925: 2500,
  900: 3243,
  850: 4781,
  800: 6394,
  700: 9882,
  600: 13800,
  500: 18289,
  400: 23574,
  300: 30065,
  250: 33999,
  200: 38662
};

/**
 * Build Open-Meteo hourly parameter list for 10m + all pressure levels
 */
function buildHourlyParams() {
  const params = ['wind_speed_10m', 'wind_direction_10m'];
  for (const p of PRESSURE_LEVELS_HPA) {
    params.push(`wind_speed_${p}hPa`);
    params.push(`wind_direction_${p}hPa`);
    params.push(`geopotential_height_${p}hPa`);
  }
  return params.join(',');
}

/**
 * Find the hourly index in Open-Meteo `hourly.time` array closest to `targetDateObj`
 */
function findClosestHourIndex(isoTimes, targetDateObj) {
  const targetMs = targetDateObj.getTime();
  let bestIdx = 0;
  let minDiff = Infinity;
  for (let i = 0; i < isoTimes.length; i++) {
    const tMs = new Date(isoTimes[i]).getTime();
    const diff = Math.abs(tMs - targetMs);
    if (diff < minDiff) {
      minDiff = diff;
      bestIdx = i;
    }
  }
  return bestIdx;
}

/**
 * Interpolate raw pressure-level wind observations (sorted by altitudeFt)
 * onto a clean 1,000-ft grid: 0 kft, 1 kft, 2 kft, ..., maxKft.
 */
export function interpolateWindProfileTo1000FtSteps(rawPoints, maxAltFt = 25000) {
  const sorted = [...rawPoints]
    .filter((pt) => Number.isFinite(pt.altFt) && Number.isFinite(pt.speedKts) && Number.isFinite(pt.dirDeg))
    .sort((a, b) => a.altFt - b.altFt);

  if (sorted.length === 0) {
    return [{ altitudeKft: 0, speed: 0, direction: 0 }];
  }

  const maxKft = Math.max(5, Math.min(40, Math.ceil(maxAltFt / 1000)));
  const layers = [];

  for (let kft = 0; kft <= maxKft; kft += 1) {
    const targetFt = kft * 1000.0;
    let speed = sorted[0].speedKts;
    let direction = sorted[0].dirDeg;

    if (targetFt <= sorted[0].altFt) {
      speed = sorted[0].speedKts;
      direction = sorted[0].dirDeg;
    } else if (targetFt >= sorted[sorted.length - 1].altFt) {
      speed = sorted[sorted.length - 1].speedKts;
      direction = sorted[sorted.length - 1].dirDeg;
    } else {
      for (let i = 0; i < sorted.length - 1; i++) {
        const p0 = sorted[i];
        const p1 = sorted[i + 1];
        if (targetFt >= p0.altFt && targetFt <= p1.altFt) {
          speed = linearInterpolate(targetFt, p0.altFt, p0.speedKts, p1.altFt, p1.speedKts);
          direction = interpolateAngle(targetFt, p0.altFt, p0.dirDeg, p1.altFt, p1.dirDeg);
          break;
        }
      }
    }

    layers.push({
      altitudeKft: kft,
      speed: Math.round(speed * 10) / 10,
      direction: Math.round(direction) % 360
    });
  }

  return layers;
}

/**
 * Fetch real-time or forecast wind layers (every 1,000 ft) for a given coordinate and target time.
 * @param {number} lat - Latitude
 * @param {number} lon - Longitude
 * @param {Date} targetDate - Target date/time (now for real-time, or user-selected forecast date/time)
 * @param {number} maxAltFt - Maximum altitude in feet to generate 1,000-ft layers for
 */
export async function fetchWindLayersPer1000Ft(lat, lon, targetDate = new Date(), maxAltFt = 20000) {
  const hourlyParams = buildHourlyParams();
  const url = `https://api.open-meteo.com/v1/forecast?latitude=${lat.toFixed(4)}&longitude=${lon.toFixed(4)}&hourly=${hourlyParams}&wind_speed_unit=kn&timezone=auto&forecast_days=7`;

  const response = await fetch(url, { mode: 'cors' });
  if (!response.ok) {
    throw new Error(`שגיאת שרת תחזית רוחות (HTTP ${response.status})`);
  }

  const data = await response.json();
  if (!data || !data.hourly || !Array.isArray(data.hourly.time) || data.hourly.time.length === 0) {
    throw new Error('לא התקבלו נתוני רוח תקינים מהשרת');
  }

  const hr = data.hourly;
  const idx = findClosestHourIndex(hr.time, targetDate);
  const matchedTimeIso = hr.time[idx];
  const elevationM = Number.isFinite(data.elevation) ? data.elevation : 0;
  const surfaceAltFt = Math.max(0, elevationM * M_TO_FT + 33);

  const rawPoints = [];

  // Surface 10m wind
  const s10 = hr.wind_speed_10m?.[idx];
  const d10 = hr.wind_direction_10m?.[idx];
  if (Number.isFinite(s10) && Number.isFinite(d10)) {
    rawPoints.push({
      altFt: surfaceAltFt,
      speedKts: s10,
      dirDeg: d10
    });
  }

  // Pressure level winds + geopotential heights
  for (const p of PRESSURE_LEVELS_HPA) {
    const spd = hr[`wind_speed_${p}hPa`]?.[idx];
    const dir = hr[`wind_direction_${p}hPa`]?.[idx];
    const geoM = hr[`geopotential_height_${p}hPa`]?.[idx];

    if (Number.isFinite(spd) && Number.isFinite(dir)) {
      const altFt = Number.isFinite(geoM) ? geoM * M_TO_FT : STD_ALT_FT_BY_HPA[p];
      rawPoints.push({
        altFt,
        speedKts: spd,
        dirDeg: dir
      });
    }
  }

  const layers = interpolateWindProfileTo1000FtSteps(rawPoints, Math.max(15000, maxAltFt + 3000));

  return {
    matchedTimeIso,
    layers
  };
}
