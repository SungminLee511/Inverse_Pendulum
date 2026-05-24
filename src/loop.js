// loop.js — multi-rate sim loop (PLAN §3.1).
// Phase 1: scaffold only. physicsStep / sensorStep / controlStep are no-ops here
// and will be filled in by later phases via setStep().
//
// One requestAnimationFrame tick drives the whole simulation:
//   wallDt   = clamped wall-clock since last frame
//   simAdvance = wallDt * speed
//   while simAdvance > 0:
//     physicsStep(dt_sim)
//     if sensorAccum  >= dt_sensor:  sensorStep();  sensorAccum  -= dt_sensor
//     if controlAccum >= dt_control: controlStep(); controlAccum -= dt_control
//     simAdvance -= dt_sim
//   render()

import { state, emit } from './state.js';

const noop = () => {};
const steps = {
  physics: noop,    // (dt_sim) => void
  sensor: noop,     // ()       => void
  control: noop,    // ()       => void
  render: noop,     // (W, H, ctx) => void  -- set by canvas module
};

export function setStep(name, fn) {
  if (!(name in steps)) throw new Error(`unknown step '${name}'`);
  steps[name] = fn || noop;
}

let lastFrame = 0;
let sensorAccum = 0;
let controlAccum = 0;

let fpsAccum = 0, fpsCount = 0, lastFpsUpdate = 0;

let frameHook = noop;   // (now, dt_wall) => void  -- for HUD etc.
export function onFrame(fn) { frameHook = fn || noop; }

let _running = true;
function pumpRunningFromState() { _running = state.running; }
pumpRunningFromState();
// keep _running in sync via state pub/sub
import { on } from './state.js';
on('running-change', pumpRunningFromState);
on('reset', () => { sensorAccum = 0; controlAccum = 0; });

function tick(now) {
  if (lastFrame === 0) lastFrame = now;
  const wallDt = (now - lastFrame) / 1000;
  lastFrame = now;

  // FPS bookkeeping
  fpsAccum += wallDt; fpsCount += 1;
  if (now - lastFpsUpdate > 500) {
    state.fps = fpsCount / Math.max(fpsAccum, 1e-6);
    fpsAccum = 0; fpsCount = 0; lastFpsUpdate = now;
  }

  frameHook(now, wallDt);

  if (_running) {
    const maxFrame = (state.params.max_frame_ms || 50) / 1000;
    let simAdvance = Math.min(wallDt, maxFrame) * state.speed;
    const dt_sim = state.params.dt_sim;
    const dt_sensor = state.params.sensor_period;
    const dt_control = state.params.control_period;

    // Hard safety: cap iterations so a buggy dt_sim can't freeze the tab
    const MAX_SUBSTEPS = 200000;
    let iter = 0;
    while (simAdvance > 0 && iter++ < MAX_SUBSTEPS) {
      const step = Math.min(dt_sim, simAdvance);
      steps.physics(step);
      sensorAccum += step;
      controlAccum += step;
      if (sensorAccum >= dt_sensor) {
        steps.sensor();
        sensorAccum -= dt_sensor;
        // clamp to prevent unbounded debt
        if (sensorAccum > dt_sensor) sensorAccum = 0;
      }
      if (controlAccum >= dt_control) {
        steps.control();
        controlAccum -= dt_control;
        if (controlAccum > dt_control) controlAccum = 0;
      }
      state.t += step;
      simAdvance -= step;
    }
  }

  steps.render();
  requestAnimationFrame(tick);
}

export function start() {
  requestAnimationFrame(tick);
  emit('loop-start');
}

// Diagnostics (for tests / debug)
export const _diag = {
  get accumulators() { return { sensorAccum, controlAccum }; },
};
