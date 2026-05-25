// tests/ui/plots_live.test.js — Phase 6 plot canvases come alive in the browser.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { chromium } from 'playwright';
import { startServer } from '../_static_server.js';

async function setup() {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const page = await (await browser.newContext()).newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });
  return { srv, browser, page };
}

// Walk the four plot canvases and count distinct colours present.
// A "drawn" plot has axes + a polyline + grid; expect ≥ 5 distinct colours.
async function distinctColors(page, id) {
  return await page.evaluate((cid) => {
    const cv = document.getElementById(cid);
    const ctx = cv.getContext('2d');
    const im = ctx.getImageData(0, 0, cv.width, cv.height);
    const set = new Set();
    for (let i = 0; i < im.data.length; i += 4) {
      const a = im.data[i+3];
      if (a < 8) continue;
      // pack RGB into a single number, with a tiny tolerance bucket so AA
      // doesn't blow the distinct count out of proportion.
      const r = im.data[i] >> 3;
      const g = im.data[i+1] >> 3;
      const b = im.data[i+2] >> 3;
      set.add((r << 10) | (g << 5) | b);
    }
    return set.size;
  }, id);
}

test('All four plot canvases have content after ~2 s sim', async () => {
  const { srv, browser, page } = await setup();
  // Kick pendulum off hanging so velocities + force are non-zero.
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'auto';
    s.params.F_max = 12;
    s.params.cart_visc = 0.05;
    s.q[1] = Math.PI - 0.4;   // perturb so plots fill with motion
    s.qdot[1] = 0;
    s.speed = 5.0;
  });
  await page.waitForTimeout(2000);
  for (const id of ['plot-angles', 'plot-velocities', 'plot-phase', 'plot-force']) {
    const n = await distinctColors(page, id);
    assert.ok(n >= 5, `${id}: distinct colours ≥ 5 after 2 s (got ${n})`);
  }
  await browser.close(); await srv.close();
});

test('Force plot shows ±F_max reference lines', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'auto';
    s.params.F_max = 12;
    s.q[1] = Math.PI - 0.4;
    s.speed = 5.0;
  });
  await page.waitForTimeout(1500);
  // The dashed amber reference (#d29922 = rgb 210, 153, 34) gets blended by
  // canvas antialiasing into dimmer variants — same hue, lower brightness.
  // Match by hue: R > G > B, R−B ≥ 60, R ≈ 1.4·G (within 50%).
  const hits = await page.evaluate(() => {
    const cv = document.getElementById('plot-force');
    const ctx = cv.getContext('2d');
    const im = ctx.getImageData(0, 0, cv.width, cv.height);
    let n = 0;
    for (let i = 0; i < im.data.length; i += 4) {
      const r = im.data[i], g = im.data[i+1], b = im.data[i+2];
      if (r > g && g > b && r - b >= 60 && Math.abs(r - 1.4 * g) < 0.5 * g) n++;
    }
    return n;
  });
  assert.ok(hits > 20, `amber F_max reference present (got ${hits} pixels)`);
  await browser.close(); await srv.close();
});

test('Phase plot trail accumulates over time (more bright px later)', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'swingup';
    s.params.F_max = 12;
    s.q[1] = Math.PI - 0.4;
    s.speed = 5.0;
  });
  // Count only "data" pixels — blue trail dots, not background.
  const countTrail = async () => page.evaluate(() => {
    const cv = document.getElementById('plot-phase');
    const ctx = cv.getContext('2d');
    const im = ctx.getImageData(0, 0, cv.width, cv.height);
    let n = 0;
    for (let i = 0; i < im.data.length; i += 4) {
      const r = im.data[i], g = im.data[i+1], b = im.data[i+2];
      // blue-ish: B > R, B − R > 30
      if (b > r && b - r > 30 && b > 80) n++;
    }
    return n;
  });
  await page.waitForTimeout(400);
  const early = await countTrail();
  await page.waitForTimeout(2500);
  const late = await countTrail();
  assert.ok(late > early + 20,
    `phase trail accumulates (early=${early} → late=${late} blue px)`);
  await browser.close(); await srv.close();
});

test('Plot buffers clear on mode change', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'auto';
    s.q[1] = Math.PI - 0.4;
    s.speed = 5.0;
  });
  await page.waitForTimeout(800);
  await page.locator('.mode-btn[data-mode="2"]').click();
  // Right after a mode change, buffers should be empty → very few coloured px.
  await page.waitForTimeout(40);
  const px = await page.evaluate(() => {
    const cv = document.getElementById('plot-angles');
    const ctx = cv.getContext('2d');
    const im = ctx.getImageData(0, 0, cv.width, cv.height);
    let n = 0;
    // Count only "data" pixels — bright blue/orange/green ribbons.
    for (let i = 0; i < im.data.length; i += 4) {
      const r = im.data[i], g = im.data[i+1], b = im.data[i+2];
      // bright blue line ≈ (88, 166, 255) — match with tolerance
      if (Math.abs(r - 88) < 40 && Math.abs(g - 166) < 40 && Math.abs(b - 255) < 40) n++;
    }
    return n;
  });
  assert.ok(px < 20, `angles buffer drained on mode-change (got ${px} blue px)`);
  await browser.close(); await srv.close();
});
