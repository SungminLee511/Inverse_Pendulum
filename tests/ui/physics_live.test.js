// tests/ui/physics_live.test.js — verify the real n=1 EOM runs inside the browser.
// Sanity: energy stays bounded over a few seconds of in-browser sim.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { chromium } from 'playwright';
import { startServer } from '../_static_server.js';

test('n=1 physics runs live and energy stays bounded', async () => {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const page = await (await browser.newContext()).newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });

  // Disable friction so we can check conservation
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.cart_visc = 0;
    s.params.links[0].joint_viscous = 0;
    // perturb to give the pendulum some swing
    s.q[1] = Math.PI - 0.5;
    s.qdot[1] = 0;
  });
  // Let it run for ~1.5s real time. With dt_sim=1e-4 the loop will advance up to
  // 50 ms wallclock per frame = thousands of sim steps; over 1.5s wall the sim
  // covers ~1.5 s of sim time.
  await page.waitForTimeout(1500);

  const samples = await page.evaluate(async () => {
    const out = [];
    for (let i = 0; i < 20; i++) {
      out.push(window.__pendulum.state.energy);
      await new Promise(r => setTimeout(r, 50));
    }
    return out;
  });

  for (const s of samples) assert.ok(Number.isFinite(s), `energy is finite (${s})`);
  const E0 = samples[0], Emax = Math.max(...samples), Emin = Math.min(...samples);
  const drift = Math.abs(Emax - Emin) / Math.max(Math.abs(E0), 1e-6);
  // Live browser timing isn't exactly RK4-pure (variable wall dt), so allow 2%.
  assert.ok(drift < 0.02, `energy drift ${drift.toFixed(5)} < 2% in browser run`);

  await browser.close(); await srv.close();
});

test('mode switching keeps physics step alive (no NaN)', async () => {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const page = await (await browser.newContext()).newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });

  for (const n of [2, 3, 1]) {
    await page.locator(`.mode-btn[data-mode="${n}"]`).click();
    await page.waitForTimeout(200);
    const finite = await page.evaluate(() => {
      const s = window.__pendulum.state;
      return s.q.every(Number.isFinite) && s.qdot.every(Number.isFinite);
    });
    assert.ok(finite, `q/qdot stay finite after switching to n=${n}`);
  }
  await browser.close(); await srv.close();
});
