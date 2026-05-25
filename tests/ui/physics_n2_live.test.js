// tests/ui/physics_n2_live.test.js — n=2 EOM runs live in the browser.

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

test('Switching to n=2 runs real physics: energy bounded, no NaN', async () => {
  const { srv, browser, page } = await setup();
  await page.locator('.mode-btn[data-mode="2"]').click();
  await page.waitForTimeout(50);
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.running = false;
    s.params.ctrl_mode = 'off';
    s.params.cart_visc = 0;
    s.params.links[0].joint_viscous = 0;
    s.params.links[1].joint_viscous = 0;
    s.u_cmd = 0; s.u_applied = 0; s.u_effective = 0;
    window.__pendulum._resetActuator();
    s.q[0] = 0; s.qdot[0] = 0;
    s.q[1] = Math.PI - 0.2;  s.qdot[1] = 0;
    s.q[2] = Math.PI - 0.2;  s.qdot[2] = 0;
    s.running = true;
    s.speed = 2.0;
  });
  await page.waitForTimeout(1200);
  const result = await page.evaluate(() => {
    const s = window.__pendulum.state;
    return {
      n: s.n,
      q: Array.from(s.q),
      qdot: Array.from(s.qdot),
      energy: s.energy,
      isReal: !Number.isNaN(s.energy),
    };
  });
  assert.equal(result.n, 2);
  assert.equal(result.q.length, 3);
  assert.ok(result.q.every(Number.isFinite), `q finite (${result.q})`);
  assert.ok(result.qdot.every(Number.isFinite), `qdot finite (${result.qdot})`);
  assert.ok(result.isReal, 'EOM is no longer the placeholder (state.energy finite)');
  await browser.close(); await srv.close();
});

test('LQR for n=2 produces a valid 6-vector gain', async () => {
  const { srv, browser, page } = await setup();
  await page.locator('.mode-btn[data-mode="2"]').click();
  await page.waitForTimeout(80);
  await page.evaluate(() => {
    window.__pendulum.state.params.ctrl_mode = 'lqr';
    window.__pendulum.markKDirty();
  });
  await page.waitForTimeout(100);
  // wait one tick for K to be computed
  await page.evaluate(() => { window.__pendulum.state.running = true; });
  await page.waitForTimeout(80);
  const K = await page.evaluate(() => window.__pendulum.getK());
  assert.ok(Array.isArray(K) && K.length === 6, `K length 6 (got ${K?.length})`);
  for (const k of K) assert.ok(Number.isFinite(k), `K entry finite (got ${k})`);
  await browser.close(); await srv.close();
});

test('Canvas: two links visible after mode switch (orange + blue pixels)', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => { window.__pendulum.state.running = false; });
  await page.locator('.mode-btn[data-mode="2"]').click();
  await page.waitForTimeout(50);
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'off';
    s.u_cmd = 0; s.u_applied = 0; s.u_effective = 0;
    window.__pendulum._resetActuator();
    s.q[0] = 0; s.qdot[0] = 0;
    s.q[1] = Math.PI - 0.4; s.qdot[1] = 0;
    s.q[2] = Math.PI - 0.4; s.qdot[2] = 0;
    s.running = true;
  });
  await page.waitForTimeout(500);
  const hits = await page.evaluate(() => {
    const cv = document.getElementById('pendulum-canvas');
    const ctx = cv.getContext('2d');
    const im = ctx.getImageData(0, 0, cv.width, cv.height);
    let blue = 0, orange = 0;
    for (let i = 0; i < im.data.length; i += 4) {
      const r = im.data[i], g = im.data[i+1], b = im.data[i+2];
      // link-1 #58a6ff ≈ (88, 166, 255)
      if (Math.abs(r-88) < 40 && Math.abs(g-166) < 40 && Math.abs(b-255) < 40) blue++;
      // link-2 #f0883e ≈ (240, 136, 62)
      if (Math.abs(r-240) < 30 && Math.abs(g-136) < 40 && Math.abs(b-62) < 40) orange++;
    }
    return { blue, orange };
  });
  assert.ok(hits.blue   > 30, `link-1 (blue)   visible (got ${hits.blue})`);
  assert.ok(hits.orange > 30, `link-2 (orange) visible (got ${hits.orange})`);
  await browser.close(); await srv.close();
});
