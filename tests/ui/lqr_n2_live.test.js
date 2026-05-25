// tests/ui/lqr_n2_live.test.js — LQR mode stabilises an n=2 perturbation in
// the browser end-to-end (controller + actuator + sensor chain).

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

test('n=2 LQR stabilises θ_1=0.08, θ_2=0.05 (ideal sensors/actuator) in ≤8 s', async () => {
  const { srv, browser, page } = await setup();
  await page.locator('.mode-btn[data-mode="2"]').click();
  await page.waitForTimeout(80);
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.running = false;
    s.params.ctrl_mode = 'off';
    s.u_cmd = 0; s.u_applied = 0; s.u_effective = 0;
    window.__pendulum._resetActuator();
    // Idealise the transducers.
    s.params.angle_noise = 0; s.params.cart_noise = 0;
    s.params.sensor_delay = 0; s.params.quant_bits = 0;
    s.params.motor_tau = 0; s.params.slew_max = 1e9;
    s.params.force_noise = 0; s.params.cart_coulomb = 0;
    s.params.cart_visc = 0;
    s.params.links[0].joint_viscous = 0;
    s.params.links[1].joint_viscous = 0;
    s.params.F_max = 50;
    s.params.sensor_period = 1e-3;
    s.params.control_period = 1e-3;
    // IC: small perturbation around upright.
    s.q[0] = 0;     s.qdot[0] = 0;
    s.q[1] = 0.08;  s.qdot[1] = 0;
    s.q[2] = 0.05;  s.qdot[2] = 0;
    // Make Q a bit more aggressive so 8 s is enough.
    window.__pendulum.setParam('Q_diag.1', 500);
    window.__pendulum.setParam('Q_diag.2', 500);
    window.__pendulum.setParam('R', 0.05);
    window.__pendulum.markKDirty();
    s.params.ctrl_mode = 'lqr';
    s.speed = 4.0;
    s.running = true;
  });

  const samples = await page.evaluate(async () => {
    const out = [];
    for (let i = 0; i < 80; i++) {
      const s = window.__pendulum.state;
      out.push({ tsim: s.t, th1: s.q[1], th2: s.q[2], u: s.u_cmd });
      await new Promise(r => setTimeout(r, 50));
    }
    return out;
  });

  const tail = samples.slice(-10);
  const t1 = tail.reduce((a, s) => a + Math.abs(s.th1), 0) / tail.length;
  const t2 = tail.reduce((a, s) => a + Math.abs(s.th2), 0) / tail.length;
  assert.ok(t1 < 0.05, `tail |θ_1| < 0.05 (got ${t1.toFixed(4)})`);
  assert.ok(t2 < 0.05, `tail |θ_2| < 0.05 (got ${t2.toFixed(4)})`);

  await browser.close(); await srv.close();
});

test('n=2 Q[θ_2] slider live-changes the |K[2]| gain', async () => {
  const { srv, browser, page } = await setup();
  await page.locator('.mode-btn[data-mode="2"]').click();
  await page.waitForTimeout(80);

  async function gainAtQ(qTheta2) {
    await page.evaluate((q) => {
      const s = window.__pendulum.state;
      s.running = false;
      s.params.ctrl_mode = 'lqr';
      window.__pendulum.setParam('Q_diag.2', q);
      window.__pendulum.markKDirty();
    }, qTheta2);
    await page.evaluate(() => { window.__pendulum.state.running = true; });
    await page.waitForTimeout(60);
    await page.evaluate(() => { window.__pendulum.state.running = false; });
    return await page.evaluate(() => window.__pendulum.getK());
  }
  const K_low  = await gainAtQ(50);
  const K_high = await gainAtQ(8000);
  assert.ok(Math.abs(K_high[2]) > Math.abs(K_low[2]) * 1.3,
    `|K[2]| grows with Q[θ_2] (low=${K_low[2].toFixed(2)} → high=${K_high[2].toFixed(2)})`);
  await browser.close(); await srv.close();
});
