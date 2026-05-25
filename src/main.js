// main.js — entry point. Wires DOM events to state and starts the rAF loop.
// Physics, sensor, control steps register via loop.setStep() in later phases.

import {
  state,
  setMode,
  setRunning,
  setSpeed,
  reset,
  on,
} from './state.js';
import { setStep, onFrame, start } from './loop.js';
import { buildPanels, doKick } from './ui/panel.js';
import { getEOM } from './physics/index.js';
import { step as stepInt } from './physics/integrator.js';
import { initCanvas, render as renderCanvas } from './ui/canvas.js';
import { initSensors, sensorTick, setVelocityCutoff } from './sensors.js';
import { initActuator, actuatorTick, _resetActuator } from './actuator.js';
import { initController, controllerTick, markDirty as markKDirty, getK } from './control/controller.js';
import { initPlots, plotsSampleTick, renderPlots, _internal as _plotsInternal } from './ui/plots.js';

// --- DOM refs ---
const canvas = document.getElementById('pendulum-canvas');
const ctx = canvas.getContext('2d');
const hudMode = document.getElementById('hud-mode');
const hudT = document.getElementById('hud-t');
const hudFps = document.getElementById('hud-fps');
const btnPlayPause = document.getElementById('btn-playpause');
const btnReset = document.getElementById('btn-reset');
const speedSlider = document.getElementById('speed-slider');
const speedVal = document.getElementById('speed-val');

// --- Mode + global controls ---
document.querySelectorAll('.mode-btn').forEach(b => {
  b.addEventListener('click', () => setMode(Number(b.dataset.mode)));
});
btnPlayPause.addEventListener('click', () => setRunning(!state.running));
btnReset.addEventListener('click', () => reset());
speedSlider.addEventListener('input', e => setSpeed(e.target.value));

on('mode-change', n => {
  document.querySelectorAll('.mode-btn').forEach(b => {
    b.classList.toggle('active', Number(b.dataset.mode) === n);
  });
  hudMode.textContent = `n=${n}`;
});
on('running-change', running => { btnPlayPause.textContent = running ? '⏸' : '▶'; });
on('speed-change', s => { speedVal.textContent = s.toFixed(1) + '×'; });
on('reset', () => { hudT.textContent = 't = 0.00 s'; });

// Keyboard shortcuts. K kicks pendulum link 1 with the magnitude slider value.
window.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  switch (e.key) {
    case ' ': e.preventDefault(); setRunning(!state.running); break;
    case 'r': case 'R': reset(); break;
    case 'k': case 'K': {
      const mag = Number(document.getElementById('kick-mag')?.value || 5);
      doKick(mag);
      break;
    }
    case '1': setMode(1); break;
    case '2': setMode(2); break;
    case '3': setMode(3); break;
  }
});

// Collapsible panel groups
document.querySelectorAll('.panel-title').forEach(t => {
  t.addEventListener('click', () => {
    const g = t.parentElement;
    g.classList.toggle('collapsed');
    t.textContent = t.textContent.replace(/^[▾▸]/, g.classList.contains('collapsed') ? '▸' : '▾');
  });
});

// --- Canvas (DPI + draw routines extracted to ui/canvas.js) ---
initCanvas(canvas);

// --- Real physics step. For modes where the EOM module is still a placeholder
//     (n=2 until Phase 8, n=3 until Phase 11), getEOM returns a damped-oscillation
//     stub so the canvas stays visibly alive.
setStep('physics', dt_sim => {
  const eom = getEOM(state.n);
  const integrator = state.params.integrator || 'rk4';
  // Use u_effective (post-actuator). Falls back to u_applied if actuator not initialised.
  const u = (state.u_effective !== undefined) ? state.u_effective : state.u_applied;
  const [qn, qdn] = stepInt(integrator, state.q, state.qdot, u, dt_sim, state.params, eom);
  for (let i = 0; i < state.q.length; i++) {
    state.q[i] = qn[i]; state.qdot[i] = qdn[i];
  }
});

// Sensor step + plot sampling at sensor cadence — plots throttle internally so
// changing sensor_period doesn't blow up the ring buffer.
setStep('sensor', () => {
  sensorTick();
  plotsSampleTick();
});

// Control step (Phase 4+): controller writes u_cmd → actuator → u_effective.
setStep('control', () => {
  controllerTick();
  actuatorTick(state.params.control_period);
});

setStep('render', renderCanvas);

// HUD updates each frame; plots redraw on the same wall-time tick (throttled
// to ~30 Hz inside renderPlots).
onFrame((now, dt) => {
  hudT.textContent = `t = ${state.t.toFixed(2)} s`;
  hudFps.textContent = state.fps.toFixed(0) + ' fps';
  renderPlots(now);
});

// Initial DOM sync
hudMode.textContent = `n=${state.n}`;
btnPlayPause.textContent = state.running ? '⏸' : '▶';
speedVal.textContent = state.speed.toFixed(1) + '×';
hudT.textContent = 't = 0.00 s';

buildPanels();
initSensors();
initActuator();
initController();
initPlots();

start();

// Debug handle
import { setParam } from './state.js';
window.__pendulum = { state, setParam, markKDirty, getK, _resetActuator, doKick,
  getPlotBuffers: () => _plotsInternal.getBuffers(),
  setVelocityCutoff };
