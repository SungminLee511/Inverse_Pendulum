// main.js — entry point.
// Phase 1 skeleton: state-driven mode selector + play/pause/reset/speed.
// Draws a placeholder cart + n-link "always upright" stick figure. Physics in P2.

import {
  state,
  setMode,
  setRunning,
  setSpeed,
  reset,
  on,
} from './state.js';

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

// --- Mode buttons → state.setMode ---
document.querySelectorAll('.mode-btn').forEach(b => {
  b.addEventListener('click', () => setMode(Number(b.dataset.mode)));
});

// State → mode button styling
on('mode-change', n => {
  document.querySelectorAll('.mode-btn').forEach(b => {
    b.classList.toggle('active', Number(b.dataset.mode) === n);
  });
  hudMode.textContent = `n=${n}`;
});

// --- Play / pause / reset / speed ---
btnPlayPause.addEventListener('click', () => setRunning(!state.running));
btnReset.addEventListener('click', () => reset());
speedSlider.addEventListener('input', e => setSpeed(e.target.value));

on('running-change', running => {
  btnPlayPause.textContent = running ? '⏸' : '▶';
});
on('speed-change', s => { speedVal.textContent = s.toFixed(1) + '×'; });
on('reset', () => { hudT.textContent = 't = 0.00 s'; });

// --- Keyboard shortcuts (final polish in P15 but cheap to wire now) ---
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

// --- Placeholder draw ---
function drawPlaceholder(W, H) {
  ctx.clearRect(0, 0, W, H);
  const trackY = H * 0.7;
  ctx.strokeStyle = '#30363d';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(20, trackY); ctx.lineTo(W - 20, trackY); ctx.stroke();
  ctx.strokeStyle = '#21262d'; ctx.lineWidth = 1;
  for (let x = 40; x < W - 20; x += 60) {
    ctx.beginPath(); ctx.moveTo(x, trackY); ctx.lineTo(x, trackY + 6); ctx.stroke();
  }
  // Cart at centre + state.q[0] (always 0 here)
  const cartX = W / 2 + (state.q ? state.q[0] * 100 : 0);
  const cartY = trackY - 20;
  const cartW = 60, cartH = 28;
  ctx.fillStyle = '#1d242d'; ctx.strokeStyle = '#58a6ff'; ctx.lineWidth = 1.5;
  ctx.fillRect(cartX - cartW / 2, cartY - cartH / 2, cartW, cartH);
  ctx.strokeRect(cartX - cartW / 2, cartY - cartH / 2, cartW, cartH);
  ctx.fillStyle = '#79c0ff';
  ctx.beginPath(); ctx.arc(cartX, cartY - cartH / 2, 3, 0, Math.PI * 2); ctx.fill();

  // Links from current state.q (still placeholder: just draw straight up)
  let px = cartX, py = cartY - cartH / 2;
  const px_per_m = 200;
  const palette = ['#58a6ff', '#f0883e', '#3fb950'];
  for (let i = 0; i < state.n; i++) {
    const L = state.params.links[i].L;
    const theta = state.q ? state.q[i + 1] : 0;
    // theta from up, CCW positive. Screen Y is down → "up" is -y.
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
  ctx.fillText('Phase 1 skeleton — physics not yet wired', 8, H - 8);
}

// --- rAF loop ---
let lastFrame = 0, fpsAccum = 0, fpsCount = 0, lastFpsUpdate = 0;
function tick(now) {
  if (lastFrame === 0) lastFrame = now;
  const dt = (now - lastFrame) / 1000;
  lastFrame = now;

  fpsAccum += dt; fpsCount += 1;
  if (now - lastFpsUpdate > 500) {
    state.fps = fpsCount / fpsAccum;
    hudFps.textContent = state.fps.toFixed(0) + ' fps';
    fpsAccum = 0; fpsCount = 0; lastFpsUpdate = now;
  }

  if (state.running) {
    state.t += dt * state.speed;
    hudT.textContent = `t = ${state.t.toFixed(2)} s`;
  }

  const rect = canvas.getBoundingClientRect();
  drawPlaceholder(rect.width, rect.height);
  requestAnimationFrame(tick);
}

// Wake the canvas/HUD with the default mode
hudMode.textContent = `n=${state.n}`;
btnPlayPause.textContent = state.running ? '⏸' : '▶';
speedVal.textContent = state.speed.toFixed(1) + '×';

requestAnimationFrame(tick);

// Expose for debugging
window.__pendulum = { state };
