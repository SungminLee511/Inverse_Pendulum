// ui/panel.js — parameter panel builder + presets + per-link sliders.
//
// Sections (collapsible) populated dynamically:
//   • Physical / Cart       — fixed sliders (g, m0, cart friction)
//   • Per-link (variable)   — one sub-panel per active link with m, L, l, I,
//                             joint friction. Rebuilt on mode-change.
//   • Sensor / Actuator     — F_max, slew, τ, force σ, angle σ, cart σ,
//                             quant bits, sensor T, sensor delay
//   • Controller            — ctrl_mode, Q diag (sized by mode), R, control T,
//                             handover θ/ω
//   • Simulation            — integrator, dt_sim
//
// Presets: hard-coded library + save/load via localStorage. The preset
// dropdown in index.html drives both apply + save/load.
//
// Sign / unit conventions match the rest of the codebase (angles from up, CCW).

import { state, setParam, getParam, on, setMode } from '../state.js';

const groups = {
  physical:          document.querySelector('.panel-group[data-group="physical"] .panel-body'),
  'sensor-actuator': document.querySelector('.panel-group[data-group="sensor-actuator"] .panel-body'),
  controller:        document.querySelector('.panel-group[data-group="controller"] .panel-body'),
  sim:               document.querySelector('.panel-group[data-group="sim"] .panel-body'),
};

// Per-link sliders are torn down + rebuilt on mode-change. They go into the
// physical group under a sub-divider so the layout stays tight.
let _perLinkRoot = null;

// Track sliders so a state-level setParam (e.g. preset load) refreshes the UI
const sliderRegistry = [];   // { path, slider, number, digits }

function fmt(v, digits) {
  if (Number.isInteger(v) && digits === 0) return String(v);
  return Number(v).toFixed(digits);
}

export function addSlider(host, label, path, { min, max, step = 0.001, digits = 3 } = {}) {
  if (typeof host === 'string') host = groups[host];
  if (!host) { console.warn(`panel: missing host`); return; }
  const row = document.createElement('div');
  row.className = 'slider-row';

  const lab = document.createElement('label');
  lab.textContent = label;
  lab.title = path;

  const slider = document.createElement('input');
  slider.type = 'range';
  slider.min = String(min);
  slider.max = String(max);
  slider.step = String(step);
  slider.value = String(getParam(path));

  const num = document.createElement('input');
  num.type = 'number';
  num.min = String(min);
  num.max = String(max);
  num.step = String(step);
  num.value = fmt(getParam(path), digits);

  function applyFromSlider() {
    const v = Number(slider.value);
    num.value = fmt(v, digits);
    setParam(path, v);
  }
  function applyFromNumber() {
    let v = Number(num.value);
    if (Number.isNaN(v)) return;
    v = Math.max(min, Math.min(max, v));
    slider.value = String(v);
    setParam(path, v);
  }
  slider.addEventListener('input', applyFromSlider);
  num.addEventListener('change', applyFromNumber);

  row.append(lab, slider, num);
  host.append(row);

  const entry = { path, slider, number: num, digits };
  sliderRegistry.push(entry);
  return { row, slider, number: num, entry };
}

export function addSelect(host, label, path, options) {
  if (typeof host === 'string') host = groups[host];
  if (!host) return;
  const row = document.createElement('div');
  row.className = 'slider-row';
  const lab = document.createElement('label');
  lab.textContent = label;
  const sel = document.createElement('select');
  sel.style.gridColumn = '2 / span 2';
  for (const opt of options) {
    const o = document.createElement('option');
    o.value = String(opt.value);
    o.textContent = opt.label;
    if (getParam(path) === opt.value) o.selected = true;
    sel.append(o);
  }
  sel.addEventListener('change', () => {
    const v = (typeof options[0].value === 'number') ? Number(sel.value) : sel.value;
    setParam(path, v);
  });
  row.append(lab, sel);
  host.append(row);
  return { row, select: sel };
}

// Refresh UI when params change outside the panel (e.g. preset load)
on('param-change', ({ path, value }) => {
  for (const r of sliderRegistry) {
    if (r.path === path) {
      r.slider.value = String(value);
      r.number.value = fmt(value, r.digits);
    }
  }
});

// ---------- Per-link sub-panel ----------
function destroyPerLinkSliders() {
  if (_perLinkRoot) _perLinkRoot.remove();
  _perLinkRoot = null;
  // Drop per-link entries from the slider registry (paths like 'links.X.Y').
  for (let i = sliderRegistry.length - 1; i >= 0; i--) {
    if (sliderRegistry[i].path.startsWith('links.')) sliderRegistry.splice(i, 1);
  }
}

function buildPerLinkSliders() {
  destroyPerLinkSliders();
  const root = document.createElement('div');
  root.className = 'per-link-root';
  groups.physical.append(root);
  _perLinkRoot = root;
  for (let i = 0; i < state.n; i++) {
    const subhead = document.createElement('div');
    subhead.className = 'panel-subhead';
    subhead.textContent = `Link ${i + 1}`;
    root.append(subhead);
    addSlider(root, `m${i+1} [kg]`,         `links.${i}.m`,            { min: 0.01, max: 2.0, step: 0.01, digits: 2 });
    addSlider(root, `L${i+1} [m]`,          `links.${i}.L`,            { min: 0.05, max: 1.0, step: 0.01, digits: 2 });
    addSlider(root, `l_c${i+1} [m]`,        `links.${i}.l`,            { min: 0.02, max: 0.5, step: 0.01, digits: 2 });
    addSlider(root, `I${i+1} [kg·m²]`,      `links.${i}.I`,            { min: 1e-5, max: 0.1, step: 1e-5, digits: 5 });
    addSlider(root, `joint visc ${i+1}`,    `links.${i}.joint_viscous`,{ min: 0, max: 0.05, step: 0.0001, digits: 4 });
    addSlider(root, `joint Coul ${i+1}`,    `links.${i}.joint_coulomb`,{ min: 0, max: 0.5, step: 0.001, digits: 3 });
  }
}

// Controller Q-diag sliders also depend on n (4 entries for n=1, 6 for n=2, 8 for n=3).
let _qDiagRoot = null;
function buildQDiagSliders() {
  if (_qDiagRoot) _qDiagRoot.remove();
  for (let i = sliderRegistry.length - 1; i >= 0; i--) {
    if (sliderRegistry[i].path.startsWith('Q_diag.')) sliderRegistry.splice(i, 1);
  }
  const nq = state.n + 1;
  const root = document.createElement('div');
  root.className = 'qdiag-root';
  groups.controller.append(root);
  _qDiagRoot = root;
  addSlider(root, 'Q[x]', 'Q_diag.0', { min: 0.1, max: 100, step: 0.1, digits: 1 });
  for (let i = 1; i < nq; i++) {
    addSlider(root, `Q[θ_${i}]`, `Q_diag.${i}`, { min: 1, max: 5000, step: 1, digits: 0 });
  }
  addSlider(root, 'Q[xdot]', `Q_diag.${nq}`, { min: 0.1, max: 100, step: 0.1, digits: 1 });
  for (let i = 1; i < nq; i++) {
    addSlider(root, `Q[θ̇_${i}]`, `Q_diag.${nq + i}`, { min: 0.1, max: 500, step: 0.1, digits: 1 });
  }
}

// ---------- Preset library ----------
// Built-in presets shipped with the page. User overrides land in localStorage
// under key `pendulum.presets.user.<name>`. Save/Load uses the active preset
// dropdown selection.
const BUILTIN_PRESETS = {
  'default-1': {
    n: 1, F_max: 30, ctrl_mode: 'auto', integrator: 'rk4',
    cart_visc: 0.1, cart_coulomb: 0,
    angle_noise: 0.001745, cart_noise: 1e-3, quant_bits: 12,
    sensor_period: 2e-3, sensor_delay: 2e-3, motor_tau: 5e-3, slew_max: 5000,
  },
  'default-2': { n: 2, F_max: 30, ctrl_mode: 'auto', integrator: 'rk4' },
  'default-3': { n: 3, F_max: 50, ctrl_mode: 'auto', integrator: 'rk4' },
  'noisy':     {
    angle_noise: 0.01, cart_noise: 5e-3, force_noise: 0.5,
    sensor_delay: 0.008, motor_tau: 0.01, slew_max: 500,
  },
  'fast-motor': {
    F_max: 60, motor_tau: 0.5e-3, slew_max: 50000,
  },
  'stiff-triple': {
    n: 3, F_max: 80, ctrl_mode: 'auto',
    cart_visc: 0.3, motor_tau: 1e-3, sensor_delay: 1e-3,
    angle_noise: 5e-4, quant_bits: 14,
  },
};

const USER_PRESET_KEY_PREFIX = 'pendulum.presets.user.';

function applyPreset(p) {
  if ('n' in p && p.n !== state.n) {
    setMode(p.n);
  }
  for (const [k, v] of Object.entries(p)) {
    if (k === 'n') continue;
    if (k === 'Q_diag' && Array.isArray(v)) {
      // Q_diag is an array; setParam writes each entry by index path.
      for (let i = 0; i < v.length; i++) setParam(`Q_diag.${i}`, v[i]);
      continue;
    }
    if (k === 'links' && Array.isArray(v)) {
      for (let i = 0; i < v.length && i < state.params.links.length; i++) {
        for (const [lk, lv] of Object.entries(v[i])) setParam(`links.${i}.${lk}`, lv);
      }
      continue;
    }
    setParam(k, v);
  }
}

function getUserPreset(name) {
  try { return JSON.parse(localStorage.getItem(USER_PRESET_KEY_PREFIX + name) || 'null'); }
  catch { return null; }
}

function saveUserPreset(name, obj) {
  localStorage.setItem(USER_PRESET_KEY_PREFIX + name, JSON.stringify(obj));
}

function currentSnapshot() {
  const p = state.params;
  // Shallow snapshot of the tunable scalar params + per-link arrays.
  return {
    n: state.n,
    g: p.g, m0: p.m0, cart_visc: p.cart_visc, cart_coulomb: p.cart_coulomb,
    F_max: p.F_max, motor_tau: p.motor_tau, slew_max: p.slew_max,
    force_noise: p.force_noise,
    angle_noise: p.angle_noise, cart_noise: p.cart_noise,
    quant_bits: p.quant_bits, sensor_period: p.sensor_period, sensor_delay: p.sensor_delay,
    ctrl_mode: p.ctrl_mode, R: p.R, control_period: p.control_period,
    handover_theta: p.handover_theta, handover_omega: p.handover_omega,
    Q_diag: p.Q_diag.slice(),
    integrator: p.integrator, dt_sim: p.dt_sim,
    links: p.links.map(l => ({ m: l.m, L: l.L, l: l.l, I: l.I,
      joint_viscous: l.joint_viscous, joint_coulomb: l.joint_coulomb })),
  };
}

function wirePresets() {
  const sel = document.getElementById('preset-select');
  const btnSave = document.getElementById('preset-save');
  const btnLoad = document.getElementById('preset-load');
  if (!sel) return;

  btnLoad && btnLoad.addEventListener('click', () => {
    const name = sel.value;
    let p = getUserPreset(name) || BUILTIN_PRESETS[name];
    if (!p) return;
    applyPreset(p);
  });

  btnSave && btnSave.addEventListener('click', () => {
    const name = sel.value;
    saveUserPreset(name, currentSnapshot());
  });
}

// ---------- Disturbance: kick ----------
function wireKick() {
  const btn = document.getElementById('btn-kick');
  const mag = document.getElementById('kick-mag');
  if (!btn) return;
  btn.addEventListener('click', () => doKick(Number(mag?.value || 5)));
  // Keyboard 'K' shortcut wired in main.js → calls window.__pendulum.kick()
}

/** Apply an instantaneous impulse on link 1: θ̇_1 += magnitude·sign. */
export function doKick(magnitude = 5, sign = +1) {
  if (state.n < 1 || !state.qdot) return;
  state.qdot[1] = (state.qdot[1] || 0) + sign * magnitude;
}

// ---------- Build everything ----------
export function buildPanels() {
  // Physical / Cart
  addSlider('physical', 'gravity g',     'g',            { min: 0, max: 20, step: 0.1, digits: 2 });
  addSlider('physical', 'cart m₀',       'm0',           { min: 0.1, max: 5, step: 0.05, digits: 2 });
  addSlider('physical', 'cart viscous',  'cart_visc',    { min: 0, max: 2, step: 0.01, digits: 3 });
  addSlider('physical', 'cart Coulomb',  'cart_coulomb', { min: 0, max: 5, step: 0.05, digits: 2 });

  // Per-link block (rebuilds on mode-change).
  buildPerLinkSliders();

  // Sensor / Actuator
  addSlider('sensor-actuator', 'F_max [N]',        'F_max',         { min: 1, max: 100, step: 0.5, digits: 1 });
  addSlider('sensor-actuator', 'slew_max [N/s]',   'slew_max',      { min: 50, max: 50000, step: 50, digits: 0 });
  addSlider('sensor-actuator', 'τ motor [s]',      'motor_tau',     { min: 0.0005, max: 0.05, step: 0.0005, digits: 4 });
  addSlider('sensor-actuator', 'force σ',          'force_noise',   { min: 0, max: 5, step: 0.01, digits: 3 });
  addSlider('sensor-actuator', 'angle σ [rad]',    'angle_noise',   { min: 0, max: 0.05, step: 0.0001, digits: 4 });
  addSlider('sensor-actuator', 'cart σ [m]',       'cart_noise',    { min: 0, max: 0.02, step: 0.0001, digits: 4 });
  addSlider('sensor-actuator', 'quant bits',       'quant_bits',    { min: 4, max: 20, step: 1, digits: 0 });
  addSlider('sensor-actuator', 'sensor delay [s]', 'sensor_delay',  { min: 0, max: 0.02, step: 0.0005, digits: 4 });
  addSlider('sensor-actuator', 'sensor T [s]',     'sensor_period', { min: 0.001, max: 0.02, step: 0.0005, digits: 4 });

  // Controller mode + per-mode params.
  addSelect('controller', 'mode', 'ctrl_mode', [
    { value: 'auto',    label: 'Auto (swing-up → LQR)' },
    { value: 'swingup', label: 'Swing-up only' },
    { value: 'lqr',     label: 'LQR only' },
    { value: 'off',     label: 'Off (manual u_cmd)' },
  ]);
  buildQDiagSliders();
  addSlider('controller', 'R (LQR)',          'R',                { min: 0.001, max: 1, step: 0.001, digits: 3 });
  addSlider('controller', 'control T [s]',    'control_period',   { min: 0.001, max: 0.02, step: 0.0005, digits: 4 });
  addSlider('controller', 'handover θ thresh','handover_theta',   { min: 0.05, max: 1.0, step: 0.01, digits: 2 });
  addSlider('controller', 'handover ω thresh','handover_omega',   { min: 0.1, max: 10, step: 0.1, digits: 2 });

  // Sim
  addSelect('sim', 'integrator', 'integrator', [
    { value: 'euler',     label: 'Forward Euler' },
    { value: 'si_euler',  label: 'Semi-implicit Euler' },
    { value: 'rk4',       label: 'RK4 (default)' },
  ]);
  addSlider('sim', 'dt_sim [s]', 'dt_sim', { min: 0.00005, max: 0.005, step: 0.00005, digits: 5 });

  wirePresets();
  wireKick();
}

// Rebuild dynamic sub-panels when the mode changes.
on('mode-change', () => {
  buildPerLinkSliders();
  buildQDiagSliders();
});

// Backwards-compatible alias (some tests import buildStubPanels by name).
export { buildPanels as buildStubPanels };
