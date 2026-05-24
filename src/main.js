// main.js — entry point. Phase 1 skeleton: wires mode buttons and play/pause/reset
// to dummy state, draws a placeholder frame on the canvas. Physics and controls
// come in later phases.

const state = {
  n: 1,
  running: true,
  speed: 1.0,
  t: 0,
  lastFrame: 0,
};

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

function setMode(n) {
  state.n = n;
  document.querySelectorAll('.mode-btn').forEach(b => {
    b.classList.toggle('active', Number(b.dataset.mode) === n);
  });
  hudMode.textContent = `n=${n}`;
}

document.querySelectorAll('.mode-btn').forEach(b => {
  b.addEventListener('click', () => setMode(Number(b.dataset.mode)));
});

btnPlayPause.addEventListener('click', () => {
  state.running = !state.running;
  btnPlayPause.textContent = state.running ? '⏸' : '▶';
});

btnReset.addEventListener('click', () => {
  state.t = 0;
  hudT.textContent = 't = 0.00 s';
});

speedSlider.addEventListener('input', e => {
  state.speed = Number(e.target.value);
  speedVal.textContent = state.speed.toFixed(1) + '×';
});

// Collapsible panel groups
document.querySelectorAll('.panel-title').forEach(t => {
  t.addEventListener('click', () => {
    const g = t.parentElement;
    g.classList.toggle('collapsed');
    t.textContent = t.textContent.replace(/^[▾▸]/, g.classList.contains('collapsed') ? '▸' : '▾');
  });
});

// --- Canvas DPI handling ---
function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.floor(rect.width * dpr);
  canvas.height = Math.floor(rect.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// --- Render loop (Phase 1: placeholder track + cart drawing) ---
let fpsAccum = 0, fpsCount = 0, lastFpsUpdate = 0;

function drawPlaceholder(rectW, rectH) {
  ctx.clearRect(0, 0, rectW, rectH);

  // Track
  const trackY = rectH * 0.7;
  ctx.strokeStyle = '#30363d';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(20, trackY);
  ctx.lineTo(rectW - 20, trackY);
  ctx.stroke();

  // Track ticks
  ctx.strokeStyle = '#21262d';
  ctx.lineWidth = 1;
  for (let x = 40; x < rectW - 20; x += 60) {
    ctx.beginPath();
    ctx.moveTo(x, trackY);
    ctx.lineTo(x, trackY + 6);
    ctx.stroke();
  }

  // Cart (centered) — placeholder
  const cartX = rectW / 2;
  const cartY = trackY - 20;
  const cartW = 60, cartH = 28;
  ctx.fillStyle = '#1d242d';
  ctx.strokeStyle = '#58a6ff';
  ctx.lineWidth = 1.5;
  ctx.fillRect(cartX - cartW / 2, cartY - cartH / 2, cartW, cartH);
  ctx.strokeRect(cartX - cartW / 2, cartY - cartH / 2, cartW, cartH);

  // Pivot
  ctx.fillStyle = '#79c0ff';
  ctx.beginPath();
  ctx.arc(cartX, cartY - cartH / 2, 3, 0, Math.PI * 2);
  ctx.fill();

  // Placeholder pendulum: n links, all pointing up
  let px = cartX, py = cartY - cartH / 2;
  const linkLen = 90;
  for (let i = 0; i < state.n; i++) {
    const nx = px;
    const ny = py - linkLen;
    ctx.strokeStyle = ['#58a6ff', '#f0883e', '#3fb950'][i] || '#fff';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(nx, ny);
    ctx.stroke();
    ctx.fillStyle = '#e6edf3';
    ctx.beginPath();
    ctx.arc(nx, ny, 5, 0, Math.PI * 2);
    ctx.fill();
    px = nx; py = ny;
  }

  // Watermark
  ctx.fillStyle = '#30363d';
  ctx.font = '10px ui-monospace, monospace';
  ctx.fillText('Phase 1 skeleton — physics not yet wired', 8, rectH - 8);
}

function tick(now) {
  if (state.lastFrame === 0) state.lastFrame = now;
  const dt = (now - state.lastFrame) / 1000;
  state.lastFrame = now;

  // FPS
  fpsAccum += dt;
  fpsCount += 1;
  if (now - lastFpsUpdate > 500) {
    const fps = fpsCount / fpsAccum;
    hudFps.textContent = fps.toFixed(0) + ' fps';
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

// Init
setMode(1);
requestAnimationFrame(tick);

// Expose for debugging / tests
window.__pendulum = { state };
