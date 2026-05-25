// tests/ui/physics_n3_live.test.js — n=3 EOM runs live in the browser.

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

test('Switching to n=3 runs real physics; q/qdot stay finite', async () => {
  const { srv, browser, page } = await setup();
  await page.locator('.mode-btn[data-mode="3"]').click();
  await page.waitForTimeout(50);
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.running = false;
    s.params.ctrl_mode = 'off';
    s.u_cmd = 0; s.u_applied = 0; s.u_effective = 0;
    window.__pendulum._resetActuator();
    s.params.cart_visc = 0.05;
    for (const l of s.params.links) l.joint_viscous = 0.001;
    s.q[0] = 0; s.qdot[0] = 0;
    s.q[1] = Math.PI - 0.2; s.qdot[1] = 0;
    s.q[2] = Math.PI - 0.2; s.qdot[2] = 0;
    s.q[3] = Math.PI - 0.2; s.qdot[3] = 0;
    s.running = true; s.speed = 2.0;
  });
  await page.waitForTimeout(1500);
  const result = await page.evaluate(() => {
    const s = window.__pendulum.state;
    return { n: s.n, q: Array.from(s.q), qdot: Array.from(s.qdot), energy: s.energy };
  });
  assert.equal(result.n, 3);
  assert.equal(result.q.length, 4);
  assert.ok(result.q.every(Number.isFinite), `q finite (${result.q})`);
  assert.ok(result.qdot.every(Number.isFinite), `qdot finite (${result.qdot})`);
  assert.ok(Number.isFinite(result.energy), `energy finite (got ${result.energy})`);
  await browser.close(); await srv.close();
});

test('LQR for n=3 produces a valid 8-vector gain', async () => {
  const { srv, browser, page } = await setup();
  await page.locator('.mode-btn[data-mode="3"]').click();
  await page.waitForTimeout(80);
  await page.evaluate(() => {
    window.__pendulum.state.params.ctrl_mode = 'lqr';
    window.__pendulum.markKDirty();
  });
  await page.waitForTimeout(100);
  await page.evaluate(() => { window.__pendulum.state.running = true; });
  await page.waitForTimeout(80);
  const K = await page.evaluate(() => window.__pendulum.getK());
  assert.ok(Array.isArray(K) && K.length === 8, `K length 8 (got ${K?.length})`);
  for (const k of K) assert.ok(Number.isFinite(k), `K entry finite`);
  await browser.close(); await srv.close();
});

test('Canvas: three links visible after mode switch (blue + orange + green px)', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => { window.__pendulum.state.running = false; });
  await page.locator('.mode-btn[data-mode="3"]').click();
  await page.waitForTimeout(50);
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'off';
    s.u_cmd = 0; s.u_applied = 0; s.u_effective = 0;
    window.__pendulum._resetActuator();
    // Spread angles so the three links fan out and all stay on-canvas.
    s.q[0] = 0; s.qdot[0] = 0;
    s.q[1] = 2.2;  s.qdot[1] = 0;     // ~126°
    s.q[2] = 1.5;  s.qdot[2] = 0;     // ~86°
    s.q[3] = 0.8;  s.qdot[3] = 0;     // ~46°
  });
  // Freeze the sim so gravity doesn't immediately collapse the spread.
  await page.locator('#btn-playpause').click();
  await page.waitForTimeout(150);
  // Resume just so render fires once with up-to-date state, then pause again.
  await page.evaluate(() => { window.__pendulum.state.running = false; });
  // Give the rAF render a chance to paint the latest q.
  await page.evaluate(() => new Promise(r => requestAnimationFrame(() => r(null))));
  await page.waitForTimeout(50);
  const hits = await page.evaluate(() => {
    const cv = document.getElementById('pendulum-canvas');
    const ctx = cv.getContext('2d');
    const im = ctx.getImageData(0, 0, cv.width, cv.height);
    let blueish = 0, orangeish = 0, greenish = 0;
    for (let i = 0; i < im.data.length; i += 4) {
      const r = im.data[i], g = im.data[i+1], b = im.data[i+2];
      if (b > r + 20 && b > g) blueish++;
      else if (r > g + 5 && r > b + 30 && g > b) orangeish++;
      else if (g > r && g > b + 20) greenish++;
    }
    return { blueish, orangeish, greenish };
  });
  assert.ok(hits.blueish   > 30, `link-1 (blue)   (${hits.blueish})`);
  assert.ok(hits.orangeish > 30, `link-2 (orange) (${hits.orangeish})`);
  assert.ok(hits.greenish  > 30, `link-3 (green)  (${hits.greenish})`);
  await browser.close(); await srv.close();
});
