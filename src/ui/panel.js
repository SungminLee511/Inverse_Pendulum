// ui/panel.js — parameter panel builder.
// Phase 1: provides addSlider/addNumber API and seeds a few stub controls so
// the panel isn't visually empty. Phase 7 fills it out with the full param set.

import { state, setParam, getParam, on } from '../state.js';

const groups = {
  physical:        document.querySelector('.panel-group[data-group="physical"] .panel-body'),
  'sensor-actuator': document.querySelector('.panel-group[data-group="sensor-actuator"] .panel-body'),
  controller:      document.querySelector('.panel-group[data-group="controller"] .panel-body'),
  sim:             document.querySelector('.panel-group[data-group="sim"] .panel-body'),
};

// Track sliders so a state-level setParam (e.g. preset load) refreshes the UI
const sliderRegistry = [];   // { path, slider, number }

function fmt(v, digits) {
  if (Number.isInteger(v) && digits === 0) return String(v);
  return Number(v).toFixed(digits);
}

export function addSlider(groupKey, label, path, { min, max, step = 0.001, digits = 3 } = {}) {
  const host = groups[groupKey];
  if (!host) { console.warn(`panel: unknown group '${groupKey}'`); return; }
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

  sliderRegistry.push({ path, slider, number: num, digits });
  return { row, slider, number: num };
}

export function addSelect(groupKey, label, path, options) {
  const host = groups[groupKey];
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

// --- Stub scaffolding so the panel is non-empty ---
// (full set added in Phase 7)
export function buildStubPanels() {
  // Physical / Cart
  addSlider('physical', 'gravity g',  'g',  { min: 0, max: 20, step: 0.1, digits: 2 });
  addSlider('physical', 'cart m₀',    'm0', { min: 0.1, max: 5, step: 0.05, digits: 2 });

  // Sensor / Actuator
  addSlider('sensor-actuator', 'F_max',    'F_max',       { min: 1, max: 100, step: 0.5, digits: 1 });
  addSlider('sensor-actuator', 'τ motor',  'motor_tau',   { min: 0.0005, max: 0.05, step: 0.0005, digits: 4 });
  addSlider('sensor-actuator', 'angle σ',  'angle_noise', { min: 0, max: 0.05, step: 0.0001, digits: 4 });

  // Controller
  addSlider('controller', 'R (LQR)',          'R',                { min: 0.001, max: 1, step: 0.001, digits: 3 });
  addSlider('controller', 'handover θ thresh','handover_theta',   { min: 0.05, max: 1.0, step: 0.01, digits: 2 });

  // Sim
  addSelect('sim', 'integrator', 'integrator', [
    { value: 'euler',     label: 'Forward Euler' },
    { value: 'si_euler',  label: 'Semi-implicit Euler' },
    { value: 'rk4',       label: 'RK4 (default)' },
  ]);
  addSlider('sim', 'dt_sim [ms]', 'dt_sim', { min: 0.00005, max: 0.005, step: 0.00005, digits: 5 });
}
