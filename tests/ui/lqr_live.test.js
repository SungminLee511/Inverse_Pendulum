// tests/ui/lqr_live.test.js — LQR stabilises a small perturbation in the browser.

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

test('LQR mode stabilises a 0.15 rad tilt back to ~upright within 6 s (ideal sensors/actuator)', async () => {
  const { srv, browser, page } = await setup();
  // 1) pause sim, set ctrl off, place IC, zero forces, idealize all transducers
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.running = false;
    s.params.ctrl_mode = 'off';
    s.u_cmd = 0; s.u_applied = 0; s.u_effective = 0;
    s.params.angle_noise = 0; s.params.cart_noise = 0;
    s.params.sensor_delay = 0; s.params.quant_bits = 0;
    s.params.motor_tau = 0; s.params.slew_max = 1e9;
    s.params.force_noise = 0; s.params.cart_coulomb = 0;
    s.params.cart_visc = 0;
    s.params.links[0].joint_viscous = 0;
    s.params.F_max = 50;
    s.params.sensor_period = 1e-3;
    s.params.control_period = 1e-3;
    s.q[0] = 0; s.q[1] = 0.15;
    s.qdot[0] = 0; s.qdot[1] = 0;
    s.speed = 3.0;
    window.__pendulum.markKDirty();
  });
  // 2) enable LQR + resume
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'lqr';
    s.running = true;
  });

  const samples = await page.evaluate(async () => {
    const out = [];
    for (let i = 0; i < 80; i++) {
      out.push({ t: window.__pendulum.state.t, theta: window.__pendulum.state.q[1], u: window.__pendulum.state.u_cmd });
      await new Promise(r => setTimeout(r, 50));
    }
    return out;
  });

  const tail = samples.slice(-10).map(s => Math.abs(s.theta));
  const tailMean = tail.reduce((a, b) => a + b) / tail.length;
  const peak = Math.max(...samples.map(s => Math.abs(s.theta)));
  assert.ok(tailMean < 0.05,
    `LQR brought |θ| down (peak ${peak.toFixed(3)}, tail mean ${tailMean.toFixed(4)}, first10=[${samples.slice(0,10).map(s=>s.theta.toFixed(3)).join(',')}])`);

  await browser.close(); await srv.close();
});

test('Increasing Q[θ] slider produces a larger |K[θ]| gain', async () => {
  const { srv, browser, page } = await setup();

  async function gainAtQ(qTheta) {
    await page.evaluate((q) => {
      const s = window.__pendulum.state;
      s.running = false;
      s.params.ctrl_mode = 'lqr';
      window.__pendulum.setParam('Q_diag.1', q);
      window.__pendulum.markKDirty();
    }, qTheta);
    // Resume one control tick so K is recomputed in-place
    await page.evaluate(() => { window.__pendulum.state.running = true; });
    await page.waitForTimeout(40);
    await page.evaluate(() => { window.__pendulum.state.running = false; });
    return await page.evaluate(() => window.__pendulum.getK());
  }

  const K_low  = await gainAtQ(20);
  const K_high = await gainAtQ(2000);
  assert.ok(Math.abs(K_high[1]) > Math.abs(K_low[1]) * 1.5,
    `|K[1]| grows with Q[θ] (K_low[1]=${K_low[1].toFixed(2)}, K_high[1]=${K_high[1].toFixed(2)})`);

  await browser.close(); await srv.close();
});
