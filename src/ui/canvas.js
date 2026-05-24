// ui/canvas.js — pendulum animation renderer.
// Draws the cart + n-link arm with proper world→screen transform, ground line,
// scale ticks, pivot dot, joint dots, and HUD overlays.

import { state } from '../state.js';
import { isReal } from '../physics/index.js';
import { totalEnergy } from '../physics/integrator.js';

const PX_PER_M = 200;

const PALETTE = [
  '#58a6ff',   // link 1 — blue
  '#f0883e',   // link 2 — orange
  '#3fb950',   // link 3 — green
];

const COLORS = {
  bg_top:   '#0a0d12',
  bg_bot:   '#11161d',
  ground:   '#30363d',
  ticks:    '#21262d',
  cart_fill:'#1d242d',
  cart_edge:'#58a6ff',
  pivot:    '#79c0ff',
  joint:    '#e6edf3',
  text:     '#8b949e',
  text_hi:  '#e6edf3',
  warn:     '#d29922',
  energy_pos:'#3fb950',
  energy_neg:'#f85149',
};

/** Compute world→screen transform. World x=0 is in the centre of the canvas;
 *  world y=0 is on the ground line at H*0.7. World y grows UP. */
function makeXform(W, H) {
  const groundY = H * 0.7;
  return {
    groundY,
    toScreen: (xWorld, yWorld) => [W / 2 + xWorld * PX_PER_M, groundY - yWorld * PX_PER_M],
  };
}

function drawGround(ctx, W, H, xform) {
  // Vertical bg gradient is in CSS (canvas backgroundColor); we just paint elements.
  ctx.strokeStyle = COLORS.ground;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(20, xform.groundY);
  ctx.lineTo(W - 20, xform.groundY);
  ctx.stroke();

  // Ticks every 0.2 m, labels every 0.5 m
  ctx.strokeStyle = COLORS.ticks;
  ctx.lineWidth = 1;
  ctx.fillStyle = COLORS.text;
  ctx.font = '10px ui-monospace, monospace';
  ctx.textAlign = 'center';
  const xMaxM = (W / 2 - 20) / PX_PER_M;
  for (let xm = -Math.ceil(xMaxM); xm <= Math.ceil(xMaxM); xm += 0.2) {
    const sx = W / 2 + xm * PX_PER_M;
    if (sx < 20 || sx > W - 20) continue;
    ctx.beginPath();
    ctx.moveTo(sx, xform.groundY);
    ctx.lineTo(sx, xform.groundY + (Math.abs(xm % 0.5) < 0.01 ? 8 : 4));
    ctx.stroke();
    if (Math.abs(xm % 0.5) < 0.01) {
      ctx.fillText(`${xm.toFixed(1)}`, sx, xform.groundY + 22);
    }
  }
  ctx.textAlign = 'start';
}

function drawCart(ctx, xform) {
  const [cx, cy] = xform.toScreen(state.q[0], 0);
  const cartW = 60, cartH = 28;
  // Cart body
  ctx.fillStyle = COLORS.cart_fill;
  ctx.strokeStyle = COLORS.cart_edge;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  const r = 4;
  const x0 = cx - cartW / 2, y0 = cy - cartH;
  ctx.moveTo(x0 + r, y0);
  ctx.lineTo(x0 + cartW - r, y0);
  ctx.quadraticCurveTo(x0 + cartW, y0, x0 + cartW, y0 + r);
  ctx.lineTo(x0 + cartW, y0 + cartH - r);
  ctx.quadraticCurveTo(x0 + cartW, y0 + cartH, x0 + cartW - r, y0 + cartH);
  ctx.lineTo(x0 + r, y0 + cartH);
  ctx.quadraticCurveTo(x0, y0 + cartH, x0, y0 + cartH - r);
  ctx.lineTo(x0, y0 + r);
  ctx.quadraticCurveTo(x0, y0, x0 + r, y0);
  ctx.closePath();
  ctx.fill(); ctx.stroke();

  // Wheels
  ctx.fillStyle = '#161b22';
  ctx.strokeStyle = '#30363d';
  ctx.lineWidth = 1;
  for (const xw of [cx - cartW / 4, cx + cartW / 4]) {
    ctx.beginPath();
    ctx.arc(xw, cy + 2, 5, 0, Math.PI * 2);
    ctx.fill(); ctx.stroke();
  }

  // Pivot (top of cart)
  ctx.fillStyle = COLORS.pivot;
  ctx.beginPath();
  ctx.arc(cx, cy - cartH, 4, 0, Math.PI * 2);
  ctx.fill();

  return [cx, cy - cartH];
}

function drawLinks(ctx, pivotPx) {
  let [px, py] = pivotPx;
  for (let i = 0; i < state.n; i++) {
    const L = state.params.links[i].L;
    const theta = state.q[i + 1];
    const nx = px + L * Math.sin(theta) * PX_PER_M;
    const ny = py - L * Math.cos(theta) * PX_PER_M;

    // Link line
    ctx.strokeStyle = PALETTE[i] || '#fff';
    ctx.lineWidth = 4;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(nx, ny);
    ctx.stroke();

    // CoM marker (smaller dot at l*sin, l*cos along the link)
    const l = state.params.links[i].l;
    const lx = px + l * Math.sin(theta) * PX_PER_M;
    const ly = py - l * Math.cos(theta) * PX_PER_M;
    ctx.fillStyle = COLORS.text_hi;
    ctx.globalAlpha = 0.4;
    ctx.beginPath(); ctx.arc(lx, ly, 3, 0, Math.PI * 2); ctx.fill();
    ctx.globalAlpha = 1;

    // Joint dot at far end
    ctx.fillStyle = COLORS.joint;
    ctx.beginPath();
    ctx.arc(nx, ny, 5, 0, Math.PI * 2);
    ctx.fill();

    px = nx; py = ny;
  }
}

function drawHud(ctx, W, H) {
  const real = isReal(state.n);
  ctx.font = '11px ui-monospace, monospace';
  ctx.fillStyle = COLORS.text;

  // Sign-convention legend (top-right)
  ctx.textAlign = 'right';
  ctx.fillText('θ measured from ↑, CCW positive', W - 12, 16);
  ctx.textAlign = 'start';

  // Energy badge (bottom-left)
  if (real) {
    const E = totalEnergy(state.q, state.qdot, state.params);
    state.energy = E;
    ctx.fillStyle = E >= 0 ? COLORS.energy_pos : COLORS.energy_neg;
    ctx.fillText(`E = ${E.toFixed(4)} J`, 12, H - 28);
  } else {
    state.energy = NaN;
  }

  // Status line
  ctx.fillStyle = COLORS.text;
  const status = real
    ? `n=${state.n} · integrator=${state.params.integrator} · dt_sim=${(state.params.dt_sim*1e3).toFixed(2)} ms`
    : `n=${state.n} EOM placeholder — fills in at Phase ${state.n === 2 ? 8 : 11}`;
  ctx.fillText(status, 12, H - 12);
}

let _ctx = null;
let _canvas = null;

export function initCanvas(canvasEl) {
  _canvas = canvasEl;
  _ctx = canvasEl.getContext('2d');
  resize();
  window.addEventListener('resize', resize);
}

function resize() {
  const dpr = window.devicePixelRatio || 1;
  const rect = _canvas.getBoundingClientRect();
  _canvas.width = Math.floor(rect.width * dpr);
  _canvas.height = Math.floor(rect.height * dpr);
  _ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

export function render() {
  const rect = _canvas.getBoundingClientRect();
  const W = rect.width, H = rect.height;
  _ctx.clearRect(0, 0, W, H);
  const xform = makeXform(W, H);
  drawGround(_ctx, W, H, xform);
  const pivot = drawCart(_ctx, xform);
  drawLinks(_ctx, pivot);
  drawHud(_ctx, W, H);
}
