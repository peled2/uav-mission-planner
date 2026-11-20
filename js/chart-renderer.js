/**
 * UAV Mission Planner - High-DPI Canvas Cross-Section Renderers
 * 1. LosChartRenderer: Communication Ray Terrain Cross-Section (with H_UAV & H_Safety lines)
 * 2. GlideChartRenderer: Direct Aircraft -> Landing Field Glide Trajectory vs Terrain & Clearance Floor
 */

const FT_TO_M = 0.3048;

export class LosChartRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.profile = null;
    this.hoverIdx = -1;
    this._bindEvents();
  }

  setProfile(profile) {
    this.profile = profile;
    this.render();
  }

  _bindEvents() {
    this.canvas.addEventListener('mousemove', (e) => {
      if (!this.profile || !this.profile.profileSamples?.length) return;
      const rect = this.canvas.getBoundingClientRect();
      const pad = this._getPadding();
      const plotW = rect.width - pad.left - pad.right;
      const relX = e.clientX - rect.left - pad.left;
      if (plotW <= 10 || relX < 0 || relX > plotW) {
        this.hoverIdx = -1;
      } else {
        const ratio = Math.max(0, Math.min(1, relX / plotW));
        this.hoverIdx = Math.round(ratio * (this.profile.profileSamples.length - 1));
      }
      this.render();
    });

    this.canvas.addEventListener('mouseleave', () => {
      this.hoverIdx = -1;
      this.render();
    });
  }

  _getPadding() {
    return { top: 18, right: 14, bottom: 24, left: 56 };
  }

  render() {
    if (!this.canvas) return;
    const rect = this.canvas.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;

    const dpr = window.devicePixelRatio || 1;
    const targetW = Math.round(rect.width * dpr);
    const targetH = Math.round(rect.height * dpr);
    if (this.canvas.width !== targetW || this.canvas.height !== targetH) {
      this.canvas.width = targetW;
      this.canvas.height = targetH;
    }

    const ctx = this.ctx;
    ctx.save();
    ctx.scale(dpr, dpr);

    const w = rect.width;
    const h = rect.height;
    ctx.clearRect(0, 0, w, h);

    if (!this.profile || !this.profile.profileSamples?.length) {
      ctx.direction = 'rtl';
      ctx.fillStyle = '#94a3b8';
      ctx.font = '12px "Heebo", sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('ממתין לחתך גבהים...', w / 2, h / 2);
      ctx.restore();
      return;
    }

    const p = this.profile;
    const samples = p.profileSamples;
    const pad = this._getPadding();
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;
    if (plotW <= 20 || plotH <= 20) {
      ctx.restore();
      return;
    }

    const maxDistKm = Math.max(1, p.maxRangeKm);
    const inv2Re = 1.0 / (2.0 * p.Re);

    let minY = Math.min(0, p.H0);
    let maxY = Math.max(p.H0 + 200, p.hUavM + 200);

    for (const s of samples) {
      if (s.effectiveHeightM < minY) minY = s.effectiveHeightM;
      if (s.terrainAslM > maxY) maxY = s.terrainAslM;
    }
    const ySpan = Math.max(200, maxY - minY);
    minY -= ySpan * 0.08;
    maxY += ySpan * 0.12;

    const xToPx = (dKm) => pad.left + (dKm / maxDistKm) * plotW;
    const yToPx = (yM) => pad.top + plotH - ((yM - minY) / (maxY - minY)) * plotH;

    // 1. Grid lines & Y/X Labels
    ctx.direction = 'ltr';
    ctx.font = '9.5px "JetBrains Mono", monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    const yTicks = 3;
    for (let i = 0; i <= yTicks; i++) {
      const valM = minY + (i / yTicks) * (maxY - minY);
      const valFt = Math.round(valM / FT_TO_M);
      const py = yToPx(valM);
      ctx.beginPath();
      ctx.moveTo(pad.left, py);
      ctx.lineTo(pad.left + plotW, py);
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.14)';
      ctx.lineWidth = 1;
      ctx.stroke();

      ctx.fillStyle = '#94a3b8';
      ctx.fillText(`${valFt} ft`, pad.left - 5, py);
    }

    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const xTicks = 4;
    for (let i = 0; i <= xTicks; i++) {
      const dKm = Math.round((i / xTicks) * maxDistKm);
      const px = xToPx(dKm);
      ctx.beginPath();
      ctx.moveTo(px, pad.top);
      ctx.lineTo(px, pad.top + plotH);
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.10)';
      ctx.lineWidth = 1;
      ctx.stroke();

      ctx.fillStyle = '#94a3b8';
      ctx.fillText(`${dKm} km`, px, pad.top + plotH + 5);
    }

    // 2. Shadow zone shading beyond primaryRangeKm
    if (p.primaryRangeKm < maxDistKm) {
      const x0 = xToPx(p.primaryRangeKm);
      const x1 = xToPx(maxDistKm);
      ctx.fillStyle = 'rgba(239, 68, 68, 0.11)';
      ctx.fillRect(x0, pad.top, Math.max(1, x1 - x0), plotH);
    }

    // 3. Curved Terrain Polygon
    ctx.beginPath();
    ctx.moveTo(xToPx(0), yToPx(minY));
    for (const s of samples) {
      ctx.lineTo(xToPx(s.distanceKm), yToPx(s.effectiveHeightM));
    }
    ctx.lineTo(xToPx(maxDistKm), yToPx(minY));
    ctx.closePath();
    const terrGrad = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
    terrGrad.addColorStop(0, 'rgba(71, 85, 105, 0.78)');
    terrGrad.addColorStop(1, 'rgba(15, 23, 42, 0.94)');
    ctx.fillStyle = terrGrad;
    ctx.fill();

    ctx.beginPath();
    samples.forEach((s, idx) => {
      const px = xToPx(s.distanceKm);
      const py = yToPx(s.effectiveHeightM);
      if (idx === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1.4;
    ctx.stroke();

    // 4. Safety Altitude Curve (H_UAV - H_LOS_Safety) if margin > 0
    if (p.losSafetyMarginM > 1) {
      ctx.beginPath();
      samples.forEach((s, idx) => {
        const dM = s.distanceKm * 1000.0;
        const yS = p.hSafetyAltM - dM * dM * inv2Re;
        const px = xToPx(s.distanceKm);
        const py = yToPx(yS);
        if (idx === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.75)';
      ctx.lineWidth = 1.4;
      ctx.setLineDash([4, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // 5. Primary Aircraft Altitude Curve (H_UAV)
    for (let i = 1; i < samples.length; i++) {
      const s0 = samples[i - 1];
      const s1 = samples[i];
      const d0 = s0.distanceKm * 1000.0;
      const d1 = s1.distanceKm * 1000.0;
      const y0 = p.hUavM - d0 * d0 * inv2Re;
      const y1 = p.hUavM - d1 * d1 * inv2Re;

      ctx.beginPath();
      ctx.moveTo(xToPx(s0.distanceKm), yToPx(y0));
      ctx.lineTo(xToPx(s1.distanceKm), yToPx(y1));
      if (s1.isBlockedPrimary) {
        ctx.strokeStyle = 'rgba(239, 68, 68, 0.65)';
        ctx.lineWidth = 1.4;
        ctx.setLineDash([3, 3]);
      } else {
        ctx.strokeStyle = '#22c55e';
        ctx.lineWidth = 1.9;
        ctx.setLineDash([]);
      }
      ctx.stroke();
    }
    ctx.setLineDash([]);

    // 6. Direct Tangent LOS Ray from Station (0, H0) to Primary Range Endpoint
    const targetRangeKm = p.primaryRangeKm;
    const targetRangeM = targetRangeKm * 1000.0;
    const yUavAtTarget = p.hUavM - targetRangeM * targetRangeM * inv2Re;
    const gcsX = xToPx(0);
    const gcsY = yToPx(p.H0);
    const endX = xToPx(targetRangeKm);
    const endY = yToPx(yUavAtTarget);

    ctx.beginPath();
    ctx.moveTo(gcsX, gcsY);
    ctx.lineTo(endX, endY);
    ctx.strokeStyle = '#10b981';
    ctx.lineWidth = 1.8;
    ctx.stroke();

    // 7. Blocking Ridge Dot
    if (p.blockingDistKm > 0.05) {
      const bDistM = p.blockingDistKm * 1000.0;
      const bEffY = p.blockingHeightM - bDistM * bDistM * inv2Re;
      ctx.beginPath();
      ctx.arc(xToPx(p.blockingDistKm), yToPx(bEffY), 4.5, 0, Math.PI * 2);
      ctx.fillStyle = '#f59e0b';
      ctx.fill();
      ctx.strokeStyle = '#0b1118';
      ctx.lineWidth = 1.3;
      ctx.stroke();
    }

    // 8. Hover Tooltip
    if (this.hoverIdx >= 0 && this.hoverIdx < samples.length) {
      const hs = samples[this.hoverIdx];
      const hx = xToPx(hs.distanceKm);
      ctx.beginPath();
      ctx.moveTo(hx, pad.top);
      ctx.lineTo(hx, pad.top + plotH);
      ctx.strokeStyle = 'rgba(226, 232, 240, 0.45)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);

      const boxW = 178;
      const boxH = 44;
      const bx = hx + 8 + boxW > w - pad.right ? hx - boxW - 8 : hx + 8;
      const by = pad.top + 3;
      ctx.fillStyle = 'rgba(11, 17, 24, 0.92)';
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.45)';
      ctx.beginPath();
      ctx.roundRect(bx, by, boxW, boxH, 5);
      ctx.fill();
      ctx.stroke();

      const terrFt = Math.round(hs.terrainAslM / FT_TO_M);
      const minLosFt = Math.ceil((hs.minLosAslM || 0) / FT_TO_M);
      const statusTxt = hs.isBlockedPrimary ? 'BLOCKED' : 'LOS OK';

      ctx.direction = 'ltr';
      ctx.font = '10px "JetBrains Mono", monospace';
      ctx.textAlign = 'left';
      ctx.fillStyle = '#f8fafc';
      ctx.fillText(`Dist: ${hs.distanceKm.toFixed(1)} km | Terr: ${terrFt} ft`, bx + 6, by + 11);
      ctx.fillStyle = hs.isBlockedPrimary ? '#f87171' : '#34d399';
      ctx.fillText(`Min LOS: ${minLosFt} ft (${statusTxt})`, bx + 6, by + 28);
    }

    ctx.restore();
  }
}

export class GlideChartRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.glideData = null;
    this.hoverIdx = -1;
    this._bindEvents();
  }

  setGlideData(glideData) {
    this.glideData = glideData;
    this.render();
  }

  _bindEvents() {
    this.canvas.addEventListener('mousemove', (e) => {
      if (!this.glideData || !this.glideData.profileSamples?.length) return;
      const rect = this.canvas.getBoundingClientRect();
      const pad = this._getPadding();
      const plotW = rect.width - pad.left - pad.right;
      const relX = e.clientX - rect.left - pad.left;
      if (plotW <= 10 || relX < 0 || relX > plotW) {
        this.hoverIdx = -1;
      } else {
        const ratio = Math.max(0, Math.min(1, relX / plotW));
        this.hoverIdx = Math.round(ratio * (this.glideData.profileSamples.length - 1));
      }
      this.render();
    });

    this.canvas.addEventListener('mouseleave', () => {
      this.hoverIdx = -1;
      this.render();
    });
  }

  _getPadding() {
    return { top: 18, right: 14, bottom: 24, left: 56 };
  }

  render() {
    if (!this.canvas) return;
    const rect = this.canvas.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;

    const dpr = window.devicePixelRatio || 1;
    const targetW = Math.round(rect.width * dpr);
    const targetH = Math.round(rect.height * dpr);
    if (this.canvas.width !== targetW || this.canvas.height !== targetH) {
      this.canvas.width = targetW;
      this.canvas.height = targetH;
    }

    const ctx = this.ctx;
    ctx.save();
    ctx.scale(dpr, dpr);

    const w = rect.width;
    const h = rect.height;
    ctx.clearRect(0, 0, w, h);

    if (!this.glideData || !this.glideData.profileSamples?.length) {
      ctx.direction = 'rtl';
      ctx.fillStyle = '#94a3b8';
      ctx.font = '12px "Heebo", sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('ממתין לחתך גלישה מהמטוס לשדה הנחיתה...', w / 2, h / 2);
      ctx.restore();
      return;
    }

    const g = this.glideData;
    const samples = g.profileSamples;
    const pad = this._getPadding();
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;
    if (plotW <= 20 || plotH <= 20) {
      ctx.restore();
      return;
    }

    const maxDistKm = Math.max(0.5, g.distToLfKm);
    let minY = 0;
    let maxY = Math.max(g.hUavFt + 500, g.arrivalTargetFt + 500);

    for (const s of samples) {
      if (s.terrainFt < minY) minY = s.terrainFt;
      if (s.requiredFloorFt > maxY) maxY = s.requiredFloorFt;
      if (s.glideAltFt > maxY) maxY = s.glideAltFt;
      if (s.glideAltFt < minY) minY = Math.min(minY, Math.max(-1500, s.glideAltFt));
    }
    const ySpan = Math.max(500, maxY - minY);
    minY -= ySpan * 0.08;
    maxY += ySpan * 0.12;

    const xToPx = (dKm) => pad.left + (dKm / maxDistKm) * plotW;
    const yToPx = (yFt) => pad.top + plotH - ((yFt - minY) / (maxY - minY)) * plotH;

    // 1. Grid lines
    ctx.direction = 'ltr';
    ctx.font = '9.5px "JetBrains Mono", monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    const yTicks = 3;
    for (let i = 0; i <= yTicks; i++) {
      const valFt = Math.round(minY + (i / yTicks) * (maxY - minY));
      const py = yToPx(valFt);
      ctx.beginPath();
      ctx.moveTo(pad.left, py);
      ctx.lineTo(pad.left + plotW, py);
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.14)';
      ctx.lineWidth = 1;
      ctx.stroke();

      ctx.fillStyle = '#94a3b8';
      ctx.fillText(`${valFt} ft`, pad.left - 5, py);
    }

    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const xTicks = 4;
    for (let i = 0; i <= xTicks; i++) {
      const dKm = (i / xTicks) * maxDistKm;
      const px = xToPx(dKm);
      ctx.beginPath();
      ctx.moveTo(px, pad.top);
      ctx.lineTo(px, pad.top + plotH);
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.10)';
      ctx.lineWidth = 1;
      ctx.stroke();

      ctx.fillStyle = '#94a3b8';
      ctx.fillText(`${dKm.toFixed(1)} km`, px, pad.top + plotH + 5);
    }

    // 2. Terrain Polygon
    ctx.beginPath();
    ctx.moveTo(xToPx(0), yToPx(minY));
    for (const s of samples) {
      ctx.lineTo(xToPx(s.distanceKm), yToPx(s.terrainFt));
    }
    ctx.lineTo(xToPx(maxDistKm), yToPx(minY));
    ctx.closePath();
    const terrGrad = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
    terrGrad.addColorStop(0, 'rgba(71, 85, 105, 0.80)');
    terrGrad.addColorStop(1, 'rgba(15, 23, 42, 0.95)');
    ctx.fillStyle = terrGrad;
    ctx.fill();

    // 3. Emergency Ridge Clearance Floor Line (dashed amber)
    ctx.beginPath();
    samples.forEach((s, idx) => {
      const px = xToPx(s.distanceKm);
      const py = yToPx(s.requiredFloorFt);
      if (idx === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.strokeStyle = 'rgba(245, 158, 11, 0.72)';
    ctx.lineWidth = 1.3;
    ctx.setLineDash([4, 3]);
    ctx.stroke();
    ctx.setLineDash([]);

    // 4. Aircraft Glide Descent Trajectory (Green/Violet where above floor, Red where below)
    for (let i = 1; i < samples.length; i++) {
      const s0 = samples[i - 1];
      const s1 = samples[i];
      ctx.beginPath();
      ctx.moveTo(xToPx(s0.distanceKm), yToPx(s0.glideAltFt));
      ctx.lineTo(xToPx(s1.distanceKm), yToPx(s1.glideAltFt));
      if (s1.isBlocked) {
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 2.0;
        ctx.setLineDash([4, 3]);
      } else {
        ctx.strokeStyle = '#a855f7';
        ctx.lineWidth = 2.2;
        ctx.setLineDash([]);
      }
      ctx.stroke();
    }
    ctx.setLineDash([]);

    // 5. Aircraft Start Dot (left: 0 km) & Landing Field Target Dot (right: maxDistKm)
    ctx.beginPath();
    ctx.arc(xToPx(0), yToPx(g.hUavFt), 4.5, 0, Math.PI * 2);
    ctx.fillStyle = '#38bdf8';
    ctx.fill();

    ctx.beginPath();
    ctx.arc(xToPx(maxDistKm), yToPx(g.arrivalTargetFt), 4.5, 0, Math.PI * 2);
    ctx.fillStyle = g.canGlideToLf ? '#22c55e' : '#ef4444';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.2;
    ctx.stroke();

    // 6. Hover Tooltip
    if (this.hoverIdx >= 0 && this.hoverIdx < samples.length) {
      const hs = samples[this.hoverIdx];
      const hx = xToPx(hs.distanceKm);
      ctx.beginPath();
      ctx.moveTo(hx, pad.top);
      ctx.lineTo(hx, pad.top + plotH);
      ctx.strokeStyle = 'rgba(226, 232, 240, 0.45)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);

      const boxW = 184;
      const boxH = 44;
      const bx = hx + 8 + boxW > w - pad.right ? hx - boxW - 8 : hx + 8;
      const by = pad.top + 3;
      ctx.fillStyle = 'rgba(11, 17, 24, 0.92)';
      ctx.strokeStyle = 'rgba(168, 85, 247, 0.5)';
      ctx.beginPath();
      ctx.roundRect(bx, by, boxW, boxH, 5);
      ctx.fill();
      ctx.stroke();

      const marginFt = hs.glideAltFt - hs.requiredFloorFt;
      ctx.direction = 'ltr';
      ctx.font = '10px "JetBrains Mono", monospace';
      ctx.textAlign = 'left';
      ctx.fillStyle = '#f8fafc';
      ctx.fillText(
        `Dist: ${hs.distanceKm.toFixed(1)} km | Glide: ${hs.glideAltFt} ft`,
        bx + 6,
        by + 11
      );
      ctx.fillStyle = marginFt >= 0 ? '#34d399' : '#f87171';
      ctx.fillText(
        `Floor: ${hs.requiredFloorFt} ft (${marginFt >= 0 ? '+' : ''}${marginFt} ft)`,
        bx + 6,
        by + 28
      );
    }

    ctx.restore();
  }
}
