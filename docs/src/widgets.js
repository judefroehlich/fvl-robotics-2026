/* Interactive panels for CODE_EXPLAINED.
 *
 * The control classes below are direct ports of src/profile.cpp, src/pid.cpp
 * and the loop in src/chassis.cpp. Keep them in step with the C++ -- the whole
 * point is that what you see here is what the robot runs.
 */
(function () {
  'use strict';

  const MAX_VOLTAGE = 12000;
  const LOOP_MS = 10;
  const LOOP_S = LOOP_MS / 1000;
  const VELOCITY_FILTER = 0.08; // kVelocityFilter in chassis.cpp

  /* ------------------------------------------------- ports of the C++ --- */

  function clamp(value, limit) {
    limit = Math.abs(limit);
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
  }

  // src/profile.cpp
  class TrapezoidProfile {
    constructor(distance, maxVelocity, maxAcceleration) {
      this.d = distance;
      this.dir = distance < 0 ? -1 : 1;
      const d = Math.abs(distance);
      const vMax = Math.abs(maxVelocity);
      const aMax = Math.abs(maxAcceleration);

      if (d === 0 || vMax === 0 || aMax === 0) {
        this.cruiseV = 0; this.a = 0; this.accelDist = 0;
        this.tAccel = this.tCruiseEnd = this.tTotal = 0;
        this.shape = 'degenerate';
        return;
      }
      this.a = aMax;
      const triangleDistance = (vMax * vMax) / aMax;
      if (d < triangleDistance) {
        this.cruiseV = Math.sqrt(aMax * d);
        this.tAccel = this.cruiseV / aMax;
        this.accelDist = d / 2;
        this.tCruiseEnd = this.tAccel;
        this.shape = 'triangular';
      } else {
        this.cruiseV = vMax;
        this.tAccel = vMax / aMax;
        this.accelDist = (vMax * vMax) / (2 * aMax);
        this.tCruiseEnd = this.tAccel + (d - 2 * this.accelDist) / vMax;
        this.shape = 'trapezoidal';
      }
      this.tTotal = this.tCruiseEnd + this.tAccel;
    }

    at(t) {
      const s = { position: 0, velocity: 0, acceleration: 0 };
      if (this.tTotal <= 0) { s.position = this.d; return s; }
      if (t <= 0) return s;
      if (t >= this.tTotal) { s.position = this.d; return s; }

      if (t < this.tAccel) {
        s.position = 0.5 * this.a * t * t;
        s.velocity = this.a * t;
        s.acceleration = this.a;
      } else if (t < this.tCruiseEnd) {
        s.position = this.accelDist + this.cruiseV * (t - this.tAccel);
        s.velocity = this.cruiseV;
        s.acceleration = 0;
      } else {
        const remaining = this.tTotal - t;
        s.position = Math.abs(this.d) - 0.5 * this.a * remaining * remaining;
        s.velocity = this.a * remaining;
        s.acceleration = -this.a;
      }
      s.position *= this.dir;
      s.velocity *= this.dir;
      s.acceleration *= this.dir;
      return s;
    }

    isDone(t) { return t >= this.tTotal; }
  }

  // src/pid.cpp
  class PID {
    constructor(g, outputLimit) {
      this.g = g;
      this.limit = Math.abs(outputLimit === undefined ? MAX_VOLTAGE : outputLimit);
      this.integral = 0;
      this.prevError = 0;
      this.havePrev = false;
    }
    update(error, dt) {
      let derivative = 0;
      if (dt > 0) {
        if (error * this.prevError < 0) this.integral = 0;
        const inZone = this.g.iz <= 0 || Math.abs(error) < this.g.iz;
        if (inZone) this.integral += error * dt;
        if (this.g.kI !== 0) this.integral = clamp(this.integral, this.limit / Math.abs(this.g.kI));
        else this.integral = 0;
        if (this.havePrev) derivative = (error - this.prevError) / dt;
        this.prevError = error;
        this.havePrev = true;
      }
      const out = this.g.kP * error + this.g.kI * this.integral + this.g.kD * derivative;
      return clamp(out, this.limit);
    }
  }

  // src/pid.cpp
  class Settler {
    constructor(cfg) { this.cfg = cfg; this.settledMs = 0; }
    update(error, velocity, dtMs) {
      const close = Math.abs(error) <= this.cfg.tolerance;
      const slow = this.cfg.velocityTolerance <= 0 || Math.abs(velocity) <= this.cfg.velocityTolerance;
      if (close && slow) this.settledMs += dtMs; else this.settledMs = 0;
      return this.settledMs >= this.cfg.settleTimeMs;
    }
  }

  // Deterministic pseudo-noise, so a given set of sliders always draws the
  // same trace instead of shimmering.
  function pseudoNoise(step) {
    const x = Math.sin(step * 12.9898) * 43758.5453;
    return (x - Math.floor(x)) * 2 - 1;
  }

  /* --------------------------------------------- the closed-loop model --- */

  // Mirrors follow_profile() in src/chassis.cpp against a first-order
  // drivetrain with Coulomb friction.
  function simulate(cfg) {
    const profile = new TrapezoidProfile(cfg.distance, cfg.maxVelocity, cfg.maxAcceleration);
    const pid = new PID({ kP: cfg.kP, kI: cfg.kI, kD: cfg.kD, iz: cfg.iz });
    const settler = new Settler({
      tolerance: cfg.tolerance,
      velocityTolerance: cfg.velocityTolerance,
      settleTimeMs: 150,
    });

    const horizon = Math.max(profile.tTotal * 2.5 + 2.5, 3);
    const steps = Math.ceil(horizon / LOOP_S);

    let pos = 0, vel = 0, prev = 0, velEst = 0, reportedLast = 0;
    const trace = { t: [], commanded: [], actual: [], output: [], velEst: [], velTrue: [] };
    const r = {
      profile, settleTime: null, overshoot: 0, finalError: cfg.distance,
      peakTrackError: 0, saturated: false, endIndex: steps,
    };

    for (let step = 0; step < steps; step++) {
      const t = step * LOOP_S;
      const target = profile.at(t);

      // What the controller sees: last loop's position, quantised and noisy.
      const noisy = reportedLast + cfg.noise * pseudoNoise(step);
      const measured = cfg.quantum > 0 ? Math.round(noisy / cfg.quantum) * cfg.quantum : noisy;
      reportedLast = pos;

      const raw = (measured - prev) / LOOP_S;
      if (cfg.filterVelocity) velEst += VELOCITY_FILTER * (raw - velEst);
      else velEst = raw;
      prev = measured;

      let out = pid.update(target.position - measured, LOOP_S);
      out += cfg.kV * target.velocity + cfg.kA * target.acceleration;
      if (target.velocity > 0) out += cfg.kS;
      else if (target.velocity < 0) out -= cfg.kS;
      out = clamp(out, MAX_VOLTAGE);
      if (Math.abs(out) > MAX_VOLTAGE - 1) r.saturated = true;

      // Plant: first-order velocity response against friction that opposes
      // motion at every speed, which is what kS exists to cancel.
      let driving;
      if (Math.abs(vel) > 0.05) {
        driving = out - (vel > 0 ? cfg.friction : -cfg.friction);
      } else if (Math.abs(out) <= cfg.friction) {
        driving = 0; vel = 0;
      } else {
        driving = out - (out > 0 ? cfg.friction : -cfg.friction);
      }
      vel += (driving / cfg.trueKV - vel) / cfg.tau * LOOP_S;
      pos += vel * LOOP_S;

      trace.t.push(t);
      trace.commanded.push(target.position);
      trace.actual.push(pos);
      trace.output.push(out);
      trace.velEst.push(velEst);
      trace.velTrue.push(vel);

      r.peakTrackError = Math.max(r.peakTrackError, Math.abs(target.position - pos));
      const past = cfg.distance > 0 ? pos - cfg.distance : cfg.distance - pos;
      r.overshoot = Math.max(r.overshoot, past);

      if (r.settleTime === null && profile.isDone(t) &&
          settler.update(cfg.distance - measured, velEst, LOOP_MS)) {
        r.settleTime = t;
        r.finalError = cfg.distance - pos;
        r.endIndex = step;
      }
    }
    if (r.settleTime === null) r.finalError = cfg.distance - pos;
    r.trace = trace;
    return r;
  }

  /* ------------------------------------------------------- plot helper --- */

  function cssVar(name) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  }

  class Plot {
    constructor(canvas, height) {
      this.canvas = canvas;
      this.height = height;
      this.ctx = canvas.getContext('2d');
    }
    begin(xMax, yMin, yMax) {
      const dpr = window.devicePixelRatio || 1;
      const w = this.canvas.clientWidth || 600;
      this.canvas.width = Math.round(w * dpr);
      this.canvas.height = Math.round(this.height * dpr);
      const ctx = this.ctx;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, this.height);

      this.pad = { l: 46, r: 10, t: 16, b: 20 };
      this.w = w; this.xMax = xMax || 1;
      // Pad the range a little so curves never touch the frame.
      const span = (yMax - yMin) || 1;
      this.yMin = yMin - span * 0.08;
      this.yMax = yMax + span * 0.08;

      this.plotW = w - this.pad.l - this.pad.r;
      this.plotH = this.height - this.pad.t - this.pad.b;

      this.grid();
      return this;
    }
    px(t) { return this.pad.l + (t / this.xMax) * this.plotW; }
    py(v) {
      const f = (v - this.yMin) / (this.yMax - this.yMin);
      return this.pad.t + (1 - f) * this.plotH;
    }
    grid() {
      const ctx = this.ctx;
      ctx.strokeStyle = cssVar('--plot-grid');
      ctx.fillStyle = cssVar('--ink-faint');
      ctx.lineWidth = 1;
      ctx.font = '10px ui-monospace, monospace';
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';

      const ticks = niceTicks(this.yMin, this.yMax, this.plotH < 80 ? 2 : 4);
      for (const v of ticks) {
        const y = Math.round(this.py(v)) + 0.5;
        ctx.beginPath();
        ctx.moveTo(this.pad.l, y);
        ctx.lineTo(this.pad.l + this.plotW, y);
        ctx.stroke();
        ctx.fillText(fmtTick(v), this.pad.l - 6, y);
      }
      // x axis ticks in seconds
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      const xt = niceTicks(0, this.xMax, 5);
      for (const t of xt) {
        if (t < 0) continue;
        const x = Math.round(this.px(t)) + 0.5;
        ctx.fillText(t.toFixed(t < 1 ? 1 : 1) + 's', x, this.pad.t + this.plotH + 4);
      }
    }
    zero() {
      if (this.yMin > 0 || this.yMax < 0) return this;
      const ctx = this.ctx;
      ctx.strokeStyle = cssVar('--rule-strong');
      ctx.lineWidth = 1;
      const y = Math.round(this.py(0)) + 0.5;
      ctx.beginPath(); ctx.moveTo(this.pad.l, y); ctx.lineTo(this.pad.l + this.plotW, y); ctx.stroke();
      return this;
    }
    hline(v, color, dashed) {
      const ctx = this.ctx;
      ctx.save();
      ctx.strokeStyle = color; ctx.lineWidth = 1;
      if (dashed) ctx.setLineDash([4, 4]);
      const y = Math.round(this.py(v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(this.pad.l, y); ctx.lineTo(this.pad.l + this.plotW, y); ctx.stroke();
      ctx.restore();
      return this;
    }
    vband(t0, t1, color) {
      const ctx = this.ctx;
      ctx.save();
      ctx.fillStyle = color;
      ctx.fillRect(this.px(t0), this.pad.t, Math.max(this.px(t1) - this.px(t0), 0), this.plotH);
      ctx.restore();
      return this;
    }
    vline(t, color, label) {
      const ctx = this.ctx;
      ctx.save();
      ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
      const x = Math.round(this.px(t)) + 0.5;
      ctx.beginPath(); ctx.moveTo(x, this.pad.t); ctx.lineTo(x, this.pad.t + this.plotH); ctx.stroke();
      if (label) {
        ctx.setLineDash([]);
        ctx.fillStyle = color;
        ctx.font = '9px ui-monospace, monospace';
        ctx.textAlign = 'left'; ctx.textBaseline = 'top';
        ctx.fillText(label, x + 3, this.pad.t + 2);
      }
      ctx.restore();
      return this;
    }
    series(xs, ys, color, opts) {
      const o = opts || {};
      const ctx = this.ctx;
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = o.width || 1.8;
      if (o.dashed) ctx.setLineDash([5, 4]);
      ctx.lineJoin = 'round';
      ctx.beginPath();
      const n = o.limit === undefined ? xs.length : Math.min(o.limit + 1, xs.length);
      for (let i = 0; i < n; i++) {
        const x = this.px(xs[i]), y = this.py(ys[i]);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.restore();
      return this;
    }
  }

  function niceTicks(min, max, count) {
    if (!isFinite(min) || !isFinite(max) || max <= min) return [0];
    const raw = (max - min) / count;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const norm = raw / mag;
    const step = (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
    const out = [];
    for (let v = Math.ceil(min / step) * step; v <= max + step * 1e-6; v += step) {
      out.push(Math.abs(v) < step * 1e-6 ? 0 : v);
    }
    return out;
  }
  function fmtTick(v) {
    const a = Math.abs(v);
    if (a >= 1000) return (v / 1000).toFixed(a >= 10000 ? 0 : 1) + 'k';
    if (a >= 10) return v.toFixed(0);
    if (a >= 1) return v.toFixed(1);
    if (a === 0) return '0';
    return v.toFixed(2);
  }

  /* ------------------------------------------------------- UI plumbing --- */

  function el(tag, cls, html) {
    const n = document.createElement(tag);
    if (cls) n.className = cls;
    if (html !== undefined) n.innerHTML = html;
    return n;
  }

  function slider(parent, spec, onChange) {
    const wrap = el('div', 'ctrl');
    const id = 'c-' + Math.random().toString(36).slice(2, 8);
    wrap.innerHTML =
      `<label for="${id}">${spec.label}</label><output id="o-${id}"></output>` +
      `<input id="${id}" type="range" min="${spec.min}" max="${spec.max}" step="${spec.step}" value="${spec.value}">`;
    parent.appendChild(wrap);
    const input = wrap.querySelector('input');
    const out = wrap.querySelector('output');
    const show = () => { out.textContent = spec.format ? spec.format(+input.value) : input.value; };
    input.addEventListener('input', () => { show(); onChange(); });
    show();
    return {
      get: () => +input.value,
      set: (v) => { input.value = v; show(); },
    };
  }

  function group(parent, title) {
    const g = el('div', 'ctrl-group');
    if (title) g.appendChild(el('h5', null, title));
    const grid = el('div', 'ctrls');
    g.appendChild(grid);
    parent.appendChild(g);
    return grid;
  }

  function statBlock(parent, keys) {
    const dl = el('dl', 'stats');
    const map = {};
    for (const k of keys) {
      const s = el('div', 'stat');
      s.innerHTML = `<dt>${k.label}</dt><dd>—</dd>`;
      dl.appendChild(s);
      map[k.key] = s;
    }
    parent.appendChild(dl);
    return {
      set(key, text, tone) {
        const s = map[key];
        if (!s) return;
        s.querySelector('dd').textContent = text;
        s.className = 'stat' + (tone ? ' ' + tone : '');
      },
    };
  }

  function plotPanel(parent, label, height) {
    const p = el('div', 'plot');
    p.appendChild(el('div', 'plot-label', label));
    const c = el('canvas');
    p.appendChild(c);
    parent.appendChild(p);
    return new Plot(c, height);
  }

  const redrawers = [];
  function register(fn) { redrawers.push(fn); fn(); }
  window.addEventListener('resize', () => redrawers.forEach((f) => f()));
  window.__docsRedraw = () => redrawers.forEach((f) => f());

  /* ----------------------------------------------------- widget: profile -- */

  function buildProfile(host) {
    const ctrls = group(host);
    const draw = () => render();
    const dist = slider(ctrls, { label: 'distance', min: -72, max: 72, step: 0.5, value: 24, format: (v) => v.toFixed(1) + ' in' }, draw);
    const vmax = slider(ctrls, { label: 'max_velocity', min: 2, max: 40, step: 0.5, value: 24, format: (v) => v.toFixed(1) + ' in/s' }, draw);
    const amax = slider(ctrls, { label: 'max_acceleration', min: 5, max: 200, step: 5, value: 50, format: (v) => v + ' in/s²' }, draw);

    const stats = statBlock(host, [
      { key: 'shape', label: 'shape' },
      { key: 'peak', label: 'peak velocity' },
      { key: 'taccel', label: 't_accel' },
      { key: 'tcruise', label: 'cruise for' },
      { key: 'ttotal', label: 'duration' },
    ]);

    const plots = el('div', 'plots');
    host.appendChild(plots);
    const pPos = plotPanel(plots, 'position (in)', 120);
    const pVel = plotPanel(plots, 'velocity (in/s)', 120);
    const pAcc = plotPanel(plots, 'acceleration (in/s²)', 90);

    host.appendChild(el('div', 'legend',
      '<span><i style="background:var(--accent)"></i>commanded</span>' +
      '<span style="color:var(--ink-faint)"><i class="dash"></i>phase boundary</span>'));

    const verdict = el('div', 'verdict');
    host.appendChild(verdict);

    function render() {
      const d = dist.get(), v = vmax.get(), a = amax.get();
      const p = new TrapezoidProfile(d, v, a);
      const T = Math.max(p.tTotal, 0.05);
      const n = 400;
      const xs = [], ps = [], vs = [], as = [];
      for (let i = 0; i <= n; i++) {
        const t = (i / n) * T;
        const s = p.at(t);
        xs.push(t); ps.push(s.position); vs.push(s.velocity); as.push(s.acceleration);
      }
      const accent = cssVar('--accent');
      const faint = cssVar('--ink-faint');
      const band = cssVar('--accent-soft');

      const ext = (arr) => [Math.min(0, ...arr), Math.max(0, ...arr)];
      const mark = (plot) => {
        if (p.tCruiseEnd > p.tAccel) plot.vband(p.tAccel, p.tCruiseEnd, band);
        plot.zero();
        plot.vline(p.tAccel, faint, 't_accel');
        if (p.tCruiseEnd > p.tAccel) plot.vline(p.tCruiseEnd, faint, null);
      };

      let e = ext(ps);
      pPos.begin(T, e[0], e[1]); mark(pPos); pPos.series(xs, ps, accent);
      e = ext(vs);
      pVel.begin(T, e[0], e[1]); mark(pVel); pVel.series(xs, vs, accent);
      e = ext(as);
      pAcc.begin(T, e[0], e[1]); mark(pAcc); pAcc.series(xs, as, accent);

      stats.set('shape', p.shape);
      stats.set('peak', (p.cruiseV * p.dir).toFixed(1) + ' in/s');
      stats.set('taccel', p.tAccel.toFixed(3) + ' s');
      stats.set('tcruise', Math.max(p.tCruiseEnd - p.tAccel, 0).toFixed(3) + ' s');
      stats.set('ttotal', p.tTotal.toFixed(3) + ' s');

      const triangleDistance = (v * v) / a;
      if (p.shape === 'triangular') {
        verdict.className = 'verdict';
        verdict.innerHTML = `<b>Triangular.</b> This move needs ${triangleDistance.toFixed(1)} in to reach ` +
          `${v.toFixed(1)} in/s and back, but it is only ${Math.abs(d).toFixed(1)} in long — so it peaks at ` +
          `√(a·d) = <b>${p.cruiseV.toFixed(1)} in/s</b> and never cruises. There is no flat top.`;
      } else if (p.shape === 'degenerate') {
        verdict.className = 'verdict bad';
        verdict.innerHTML = '<b>Degenerate.</b> Zero distance or zero budget, so the profile has no duration ' +
          'and reports the target immediately. The PID would drive straight at the target with no plan.';
      } else {
        verdict.className = 'verdict good';
        verdict.innerHTML = `<b>Full trapezoid.</b> Longer than the ${triangleDistance.toFixed(1)} in needed to ` +
          `reach ${v.toFixed(1)} in/s and back, so it cruises for ` +
          `${(p.tCruiseEnd - p.tAccel).toFixed(2)} s. Duration = v/a + d/v = ` +
          `${(v / a).toFixed(3)} + ${(Math.abs(d) / v).toFixed(3)} = <b>${p.tTotal.toFixed(3)} s</b>.`;
      }
    }
    register(render);
  }

  /* -------------------------------------------------- widget: deadband --- */

  function buildDeadband(host) {
    const ctrls = group(host);
    const draw = () => render();
    const fric = slider(ctrls, { label: 'friction V_f', min: 200, max: 2000, step: 50, value: 900, format: (v) => v + ' mV' }, draw);
    const kp = slider(ctrls, { label: 'kP', min: 50, max: 3000, step: 50, value: 1400, format: (v) => String(v) }, draw);
    const tol = slider(ctrls, { label: 'tolerance', min: 0.1, max: 3, step: 0.1, value: 0.5, format: (v) => v.toFixed(1) + ' in' }, draw);

    const stats = statBlock(host, [
      { key: 'parks', label: 'parks short by' },
      { key: 'need', label: 'kP needed' },
    ]);

    const wrap = el('div', 'bar-wrap');
    wrap.innerHTML =
      '<div class="bar-scale"><div class="bar-track"></div><div class="bar-fill"></div><div class="bar-target"></div></div>' +
      '<div class="bar-note"></div>';
    host.appendChild(wrap);
    const fill = wrap.querySelector('.bar-fill');
    const target = wrap.querySelector('.bar-target');
    const note = wrap.querySelector('.bar-note');

    const verdict = el('div', 'verdict');
    host.appendChild(verdict);

    function render() {
      const parks = fric.get() / kp.get();
      const t = tol.get();
      const needed = fric.get() / t;
      const scale = Math.max(parks, t) * 1.6 || 1;

      fill.style.width = Math.min((parks / scale) * 100, 100) + '%';
      target.style.left = Math.min((t / scale) * 100, 100) + '%';
      note.textContent = `bar = where it parks · line = tolerance · full scale ${scale.toFixed(2)} in`;

      stats.set('parks', parks.toFixed(2) + ' in', parks <= t ? 'good' : 'bad');
      stats.set('need', Math.ceil(needed / 10) * 10 + '', null);

      if (parks <= t) {
        verdict.className = 'verdict good';
        verdict.innerHTML = `<b>Settles.</b> V_f/kP = ${fric.get()}/${kp.get()} = <b>${parks.toFixed(2)} in</b>, ` +
          `inside the ${t.toFixed(1)} in tolerance. Pure P would be enough here, though kI still removes the ` +
          `remaining error rather than living with it.`;
      } else {
        verdict.className = 'verdict bad';
        verdict.innerHTML = `<b>Never settles.</b> V_f/kP = ${fric.get()}/${kp.get()} = <b>${parks.toFixed(2)} in</b>, ` +
          `outside the ${t.toFixed(1)} in tolerance — so a pure-P controller parks there and the motion runs to ` +
          `its timeout, every time. You need kP above ${Math.ceil(needed / 10) * 10}, or a kI term.`;
      }
    }
    register(render);
  }

  /* ----------------------------------------------------- widget: units --- */

  function buildUnits(host) {
    const ctrls = group(host);
    const draw = () => render();
    const dia = slider(ctrls, { label: 'wheel_diameter', min: 2, max: 5, step: 0.05, value: 3.25, format: (v) => v.toFixed(2) + ' in' }, draw);
    const motorTeeth = slider(ctrls, { label: 'motor gear teeth', min: 12, max: 84, step: 12, value: 36, format: (v) => String(v) }, draw);
    const wheelTeeth = slider(ctrls, { label: 'wheel gear teeth', min: 12, max: 84, step: 12, value: 48, format: (v) => String(v) }, draw);
    const deg = slider(ctrls, { label: 'motor rotation', min: 0, max: 3600, step: 10, value: 360, format: (v) => v + '°' }, draw);

    const stats = statBlock(host, [
      { key: 'ratio', label: 'gear_ratio' },
      { key: 'circ', label: 'circumference' },
      { key: 'revs', label: 'wheel revs' },
      { key: 'inches', label: 'travel' },
    ]);
    const verdict = el('div', 'verdict');
    host.appendChild(verdict);

    function render() {
      const ratio = motorTeeth.get() / wheelTeeth.get();
      const circ = Math.PI * dia.get();
      const revs = (deg.get() / 360) * ratio;
      const inches = revs * circ;
      stats.set('ratio', ratio.toFixed(4));
      stats.set('circ', circ.toFixed(3) + ' in');
      stats.set('revs', revs.toFixed(3));
      stats.set('inches', inches.toFixed(3) + ' in', 'good');

      const wrong = (deg.get() / 360) * (1 / ratio) * circ;
      verdict.className = 'verdict';
      verdict.innerHTML =
        `<code>gear_ratio = ${motorTeeth.get()}/${wheelTeeth.get()} = ${ratio.toFixed(4)}</code> — ` +
        `wheel revolutions per motor revolution. Inverting it by mistake would report ` +
        `<b>${wrong.toFixed(2)} in</b> instead of <b>${inches.toFixed(2)} in</b>, an error of ` +
        `${(Math.max(wrong, inches) / Math.max(Math.min(wrong, inches), 1e-9)).toFixed(2)}×.`;
    }
    register(render);
  }

  /* ------------------------------------------------- widget: full loop --- */

  // Every preset below was checked against the simulation; the numbers in the
  // comments are what it actually produces for the default 24 in move.
  const PRESETS = {
    'Shipped defaults': {
      // settles 2.20 s, 1.37 in overshoot
      kP: 1400, kI: 800, kD: 200, iz: 3, kS: 800, kV: 350,
      trueKV: 350, tau: 0.15, friction: 900, noise: 0.01, filterVelocity: 1,
    },
    'No integral term': {
      // never settles: parks 0.60 in out, and V_f/kP = 0.64 in
      kP: 1400, kI: 0, kD: 200, iz: 3, kS: 800, kV: 350,
      trueKV: 350, tau: 0.15, friction: 900, noise: 0.01, filterVelocity: 1,
    },
    'Gains too low': {
      // never settles: V_f/kP = 1.80 in, far outside the 0.5 in tolerance
      kP: 500, kI: 0, kD: 50, iz: 3, kS: 800, kV: 350,
      trueKV: 350, tau: 0.15, friction: 900, noise: 0.01, filterVelocity: 1,
    },
    'No kD term': {
      // overshoot 1.37 -> 1.77 in
      kP: 1400, kI: 800, kD: 0, iz: 3, kS: 800, kV: 350,
      trueKV: 350, tau: 0.15, friction: 900, noise: 0.01, filterVelocity: 1,
    },
    'No feedforward': {
      // still arrives, but peak lag goes 1.37 -> 5.65 in
      kP: 1400, kI: 800, kD: 200, iz: 3, kS: 0, kV: 0,
      trueKV: 350, tau: 0.15, friction: 900, noise: 0.01, filterVelocity: 1,
    },
    'Sluggish robot': {
      // slower drivetrain than the gains assume: overshoot 1.37 -> 3.07 in
      kP: 1400, kI: 800, kD: 200, iz: 3, kS: 800, kV: 350,
      trueKV: 350, tau: 0.35, friction: 900, noise: 0.01, filterVelocity: 1,
    },
    'Unfiltered velocity': {
      // never settles even though the position is right: the 5.4 bug
      kP: 1400, kI: 800, kD: 200, iz: 3, kS: 800, kV: 350,
      trueKV: 350, tau: 0.15, friction: 900, noise: 0.04, filterVelocity: 0,
    },
  };

  function buildLoop(host) {
    const presetBar = el('div', 'presets');
    host.appendChild(presetBar);

    const gPid = group(host, 'controller (drive_tuning.pid)');
    const draw = () => { clearPreset(); render(); };
    const kP = slider(gPid, { label: 'kP', min: 0, max: 4000, step: 50, value: 1400, format: String }, draw);
    const kI = slider(gPid, { label: 'kI', min: 0, max: 4000, step: 50, value: 800, format: String }, draw);
    const kD = slider(gPid, { label: 'kD', min: 0, max: 600, step: 10, value: 200, format: String }, draw);
    const iz = slider(gPid, { label: 'integral_zone', min: 0, max: 12, step: 0.5, value: 3, format: (v) => v + ' in' }, draw);

    const gFf = group(host, 'feedforward (drive_tuning.ff)');
    const kS = slider(gFf, { label: 'kS', min: 0, max: 2500, step: 50, value: 800, format: (v) => v + ' mV' }, draw);
    const kV = slider(gFf, { label: 'kV', min: 0, max: 600, step: 10, value: 350, format: (v) => v + ' mV/(in/s)' }, draw);

    const gMove = group(host, 'the move (drive_tuning.constraints)');
    const dist = slider(gMove, { label: 'distance', min: 3, max: 72, step: 1, value: 24, format: (v) => v + ' in' }, draw);
    const vmax = slider(gMove, { label: 'max_velocity', min: 4, max: 40, step: 1, value: 24, format: (v) => v + ' in/s' }, draw);
    const amax = slider(gMove, { label: 'max_acceleration', min: 10, max: 200, step: 5, value: 50, format: (v) => v + ' in/s²' }, draw);

    const gPlant = group(host, 'the real robot (what you are tuning against)');
    const trueKV = slider(gPlant, { label: 'actual kV', min: 150, max: 700, step: 10, value: 350, format: (v) => v + ' mV/(in/s)' }, draw);
    const tau = slider(gPlant, { label: 'response time τ', min: 0.05, max: 0.5, step: 0.01, value: 0.15, format: (v) => v.toFixed(2) + ' s' }, draw);
    const friction = slider(gPlant, { label: 'friction V_f', min: 0, max: 2000, step: 50, value: 900, format: (v) => v + ' mV' }, draw);
    const noise = slider(gPlant, { label: 'sensor noise', min: 0, max: 0.08, step: 0.005, value: 0.01, format: (v) => v.toFixed(3) + ' in' }, draw);
    const filt = slider(gPlant, { label: 'filter velocity est.', min: 0, max: 1, step: 1, value: 1, format: (v) => (v ? 'on' : 'OFF') }, draw);

    const stats = statBlock(host, [
      { key: 'settle', label: 'settled' },
      { key: 'over', label: 'overshoot' },
      { key: 'final', label: 'final error' },
      { key: 'track', label: 'peak lag' },
      { key: 'sat', label: 'saturated' },
    ]);

    const plots = el('div', 'plots');
    host.appendChild(plots);
    const pPos = plotPanel(plots, 'position (in)  — commanded vs actual', 150);
    const pOut = plotPanel(plots, 'motor command (mV)', 110);
    const pVel = plotPanel(plots, 'velocity (in/s)  — true vs estimated', 110);

    host.appendChild(el('div', 'legend',
      '<span><i style="background:var(--accent)"></i>actual</span>' +
      '<span style="color:var(--ink-faint)"><i class="dash"></i>commanded by profile</span>' +
      '<span><i style="background:var(--warn)"></i>target / limits</span>'));

    const verdict = el('div', 'verdict');
    host.appendChild(verdict);

    function clearPreset() {
      presetBar.querySelectorAll('button').forEach((b) => b.classList.remove('active'));
    }
    Object.keys(PRESETS).forEach((name) => {
      const b = el('button', null, name);
      b.addEventListener('click', () => {
        const p = PRESETS[name];
        kP.set(p.kP); kI.set(p.kI); kD.set(p.kD); iz.set(p.iz);
        kS.set(p.kS); kV.set(p.kV);
        trueKV.set(p.trueKV); tau.set(p.tau); friction.set(p.friction);
        noise.set(p.noise); filt.set(p.filterVelocity);
        clearPreset();
        b.classList.add('active');
        render();
      });
      presetBar.appendChild(b);
    });
    presetBar.querySelector('button').classList.add('active');

    function render() {
      const cfg = {
        distance: dist.get(), maxVelocity: vmax.get(), maxAcceleration: amax.get(),
        kP: kP.get(), kI: kI.get(), kD: kD.get(), iz: iz.get(),
        kS: kS.get(), kV: kV.get(), kA: 0,
        tolerance: 0.5, velocityTolerance: 2.0,
        trueKV: trueKV.get(), tau: tau.get(), friction: friction.get(),
        quantum: 0.0025, noise: noise.get(),
        filterVelocity: filt.get() === 1,
      };
      const r = simulate(cfg);
      const tr = r.trace;
      const accent = cssVar('--accent');
      const faint = cssVar('--ink-faint');
      const warn = cssVar('--warn');

      // Show a little past settling so the hold is visible.
      const stop = Math.min(
        r.settleTime !== null ? r.endIndex + 40 : tr.t.length - 1,
        tr.t.length - 1
      );
      const T = Math.max(tr.t[stop], 0.1);

      const slice = (a) => a.slice(0, stop + 1);
      const pAll = slice(tr.actual).concat(slice(tr.commanded), [0, cfg.distance]);
      pPos.begin(T, Math.min(...pAll), Math.max(...pAll));
      pPos.hline(cfg.distance, warn, true);
      pPos.zero();
      if (r.settleTime !== null) pPos.vline(r.settleTime, accent, 'settled');
      pPos.vline(r.profile.tTotal, faint, 'profile ends');
      pPos.series(tr.t, tr.commanded, faint, { dashed: true, width: 1.4, limit: stop });
      pPos.series(tr.t, tr.actual, accent, { limit: stop });

      pOut.begin(T, Math.min(-MAX_VOLTAGE, ...slice(tr.output)), Math.max(MAX_VOLTAGE, ...slice(tr.output)));
      pOut.hline(MAX_VOLTAGE, warn, true);
      pOut.hline(-MAX_VOLTAGE, warn, true);
      pOut.zero();
      pOut.series(tr.t, tr.output, accent, { width: 1.3, limit: stop });

      const vAll = slice(tr.velTrue).concat(slice(tr.velEst));
      pVel.begin(T, Math.min(...vAll), Math.max(...vAll));
      pVel.zero();
      pVel.series(tr.t, tr.velEst, faint, { width: 1.1, limit: stop });
      pVel.series(tr.t, tr.velTrue, accent, { width: 1.6, limit: stop });

      const settled = r.settleTime !== null;
      stats.set('settle', settled ? r.settleTime.toFixed(2) + ' s' : 'never', settled ? 'good' : 'bad');
      stats.set('over', r.overshoot.toFixed(2) + ' in', r.overshoot > 1.5 ? 'bad' : 'good');
      stats.set('final', r.finalError.toFixed(3) + ' in', Math.abs(r.finalError) <= 0.5 ? 'good' : 'bad');
      stats.set('track', r.peakTrackError.toFixed(2) + ' in');
      stats.set('sat', r.saturated ? 'yes' : 'no', r.saturated ? 'bad' : 'good');

      const park = cfg.friction / Math.max(cfg.kP, 1);
      let msg, tone;
      if (!settled) {
        tone = 'bad';
        if (!cfg.filterVelocity) {
          msg = `<b>Never settles — and the position is fine.</b> Final error is only ` +
            `${Math.abs(r.finalError).toFixed(2)} in. The velocity estimate is unfiltered, so sensor noise ` +
            `reads as tens of in/s of motion that is not there, and the settle check's velocity condition can ` +
            `never pass. This is the bug described in 5.4 — turn the filter back on.`;
        } else if (cfg.kI === 0 && park > cfg.tolerance) {
          msg = `<b>Never settles — parked in the friction deadband.</b> With kI = 0, P stops producing motion ` +
            `at V_f/kP = ${cfg.friction}/${cfg.kP} = <b>${park.toFixed(2)} in</b>, outside the ` +
            `${cfg.tolerance} in tolerance. Add a kI term, or raise kP above ${Math.ceil(cfg.friction / cfg.tolerance)}.`;
        } else {
          msg = `<b>Never settles.</b> Final error ${r.finalError.toFixed(2)} in, tolerance ${cfg.tolerance} in. ` +
            `On the robot this motion would run until its timeout.`;
        }
      } else if (r.peakTrackError > 3 && r.overshoot < 1.0) {
        // Behind the plan the whole way, as opposed to sailing past the end:
        // a large tracking error with no overshoot is a feedforward problem.
        tone = 'bad';
        msg = `<b>Arrives, but lags ${r.peakTrackError.toFixed(2)} in behind the plan.</b> ` +
          (cfg.kV === 0
            ? `With kV = 0 there is no feedforward, so the PID has to discover the whole voltage from the ` +
              `error — it only ever pushes once it is already behind. Note it still settles and barely ` +
              `overshoots: the cost of dropping feedforward is tracking accuracy, not arrival.`
            : `The robot cannot keep up with this profile. Lower max_velocity or max_acceleration, or ` +
              `check that kV matches the robot.`);
      } else if (r.overshoot > 1.5) {
        tone = 'bad';
        const over = cfg.kV > cfg.trueKV;
        const slow = cfg.tau > 0.25;
        msg = `<b>Settles in ${r.settleTime.toFixed(2)} s, but overshoots ${r.overshoot.toFixed(2)} in.</b> ` +
          (slow
            ? `This drivetrain responds in ${cfg.tau.toFixed(2)} s, slower than the gains assume, so every ` +
              `correction arrives late and carries the robot past the target. Raise kD, or lower ` +
              `max_acceleration so there is less momentum to arrest.`
            : over
            ? `kV is ${(cfg.kV / cfg.trueKV).toFixed(2)}× the robot's actual value, so the feedforward is ` +
              `overdriving and the PID has to cancel its own feedforward. Lower kV.`
            : `Raise kD — overshoot responds to it more than to any other gain.`);
      } else {
        tone = 'good';
        msg = `<b>Settles in ${r.settleTime.toFixed(2)} s</b> with ${r.overshoot.toFixed(2)} in of overshoot and ` +
          `${Math.abs(r.finalError).toFixed(3)} in of final error. ` +
          (r.saturated
            ? `It does hit the 12 V limit, so there is no headroom left for bigger corrections — consider ` +
              `lowering max_velocity.`
            : `Peak command stays inside the 12 V limit, so the PID has headroom to correct with.`);
      }
      verdict.className = 'verdict ' + tone;
      verdict.innerHTML = msg;
    }
    register(render);
  }

  /* --------------------------------------------------------------- boot -- */

  const BUILDERS = {
    'w-profile': buildProfile,
    'w-deadband': buildDeadband,
    'w-units': buildUnits,
    'w-loop': buildLoop,
  };

  // Exposed so docs/test-port.mjs can check these against the C++ reference
  // values without needing a browser.
  window.__docsInternals = { TrapezoidProfile, PID, Settler, simulate, clamp };

  document.addEventListener('DOMContentLoaded', () => {
    document.querySelectorAll('[data-widget]').forEach((host) => {
      const fn = BUILDERS[host.dataset.widget];
      if (fn) {
        try { fn(host); }
        catch (err) {
          host.innerHTML = '<p class="verdict bad">This panel failed to load: ' + err.message + '</p>';
          console.error(err);
        }
      }
    });
  });
})();
