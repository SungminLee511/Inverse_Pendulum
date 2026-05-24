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
import { buildStubPanels } from './ui/panel.js';
import { getEOM, isReal } from './physics/index.js';
import { step as stepInt, totalEnergy } from './physics/integrator.js';

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

// Keyboard shortcuts
window.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  switch (e.key) {
    case ' ': e.preventDefault(); setRunning(!state.running); break;
    case 'r': case 'R': reset(); break;
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

// --- Canvas DPI ---
function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.floor(rect.width * dpr);
  canvas.height = Math.floor(rect.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// --- Real physics step. For modes where the EOM module is still a placeholder
//     (n=2 until Phase 8, n=3 until Phase 11), getEOM returns a damped-oscillation
//     stub so the canvas stays visibly alive.
setStep('physics', dt_sim => {
  const eom = getEOM(state.n);
  const integrator = state.params.integrator || 'rk4';
  const [qn, qdn] = stepInt(integrator, state.q, state.qdot, state.u_applied, dt_sim, state.params, eom);
  // overwrite in place to keep references stable
  for (let i = 0; i < state.q.length; i++) {
    state.q[i] = qn[i]; state.qdot[i] = qdn[i];
  }
});

// --- Placeholder render ---
function drawScene() {
  const W = canvas.getBoundingClientRect().width;
  const H = canvas.getBoundingClientRect().height;
  ctx.clearRect(0, 0, W, H);
  const trackY = H * 0.7;
  ctx.strokeStyle = '#30363d'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(20, trackY); ctx.lineTo(W - 20, trackY); ctx.stroke();
  ctx.strokeStyle = '#21262d'; ctx.lineWidth = 1;
  for (let x = 40; x < W - 20; x += 60) {
    ctx.beginPath(); ctx.moveTo(x, trackY); ctx.lineTo(x, trackY + 6); ctx.stroke();
  }
  const px_per_m = 200;
  const cartX = W / 2 + state.q[0] * px_per_m;
  const cartY = trackY - 20;
  const cartW = 60, cartH = 28;
  ctx.fillStyle = '#1d242d'; ctx.strokeStyle = '#58a6ff'; ctx.lineWidth = 1.5;
  ctx.fillRect(cartX - cartW / 2, cartY - cartH / 2, cartW, cartH);
  ctx.strokeRect(cartX - cartW / 2, cartY - cartH / 2, cartW, cartH);
  ctx.fillStyle = '#79c0ff';
  ctx.beginPath(); ctx.arc(cartX, cartY - cartH / 2, 3, 0, Math.PI * 2); ctx.fill();

  let px = cartX, py = cartY - cartH / 2;
  const palette = ['#58a6ff', '#f0883e', '#3fb950'];
  for (let i = 0; i < state.n; i++) {
    const L = state.params.links[i].L;
    const theta = state.q[i + 1];
    const nx = px + L * Math.sin(theta) * px_per_m;
    const ny = py - L * Math.cos(theta) * px_per_m;
    ctx.strokeStyle = palette[i] || '#fff'; ctx.lineWidth = 4;
    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(nx, ny); ctx.stroke();
    ctx.fillStyle = '#e6edf3';
    ctx.beginPath(); ctx.arc(nx, ny, 5, 0, Math.PI * 2); ctx.fill();
    px = nx; py = ny;
  }

  ctx.fillStyle = '#30363d';
  ctx.font = '10px ui-monospace, monospace';
  const real = isReal(state.n);
  const E = real ? totalEnergy(state.q, state.qdot, state.params).toFixed(4) : '—';
  const label = real
    ? `Phase 2: real n=${state.n} EOM, integrator=${state.params.integrator}, E=${E} J`
    : `Phase 2: n=${state.n} EOM placeholder (filled in Phase ${state.n === 2 ? 8 : 11})`;
  ctx.fillText(label, 8, H - 8);
  state.energy = real ? Number(E) : NaN;
}
setStep('render', drawScene);

// HUD updates each frame
onFrame((now, dt) => {
  hudT.textContent = `t = ${state.t.toFixed(2)} s`;
  hudFps.textContent = state.fps.toFixed(0) + ' fps';
});

// Initial DOM sync
hudMode.textContent = `n=${state.n}`;
btnPlayPause.textContent = state.running ? '⏸' : '▶';
speedVal.textContent = state.speed.toFixed(1) + '×';
hudT.textContent = 't = 0.00 s';

buildStubPanels();

start();

// Debug handle
window.__pendulum = { state };
