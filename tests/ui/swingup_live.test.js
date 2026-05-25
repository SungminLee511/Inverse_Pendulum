// tests/ui/swingup_live.test.js — full swing-up → LQR catch in the browser.

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

test('Auto mode swings pendulum up from hanging and LQR holds it (≤45 s, |θ|<10°)', async () => {
  const { srv, browser, page } = await setup();

  // Pause, reset to clean hanging start, idealise transducers, resume.
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.running = false;
    s.params.ctrl_mode = 'auto';
    s.params.angle_noise = 0; s.params.cart_noise = 0;
    s.params.sensor_delay = 0; s.params.quant_bits = 0;
    s.params.motor_tau = 0; s.params.slew_max = 1e9;
    s.params.force_noise = 0;
    s.params.cart_visc = 0.05; s.params.cart_coulomb = 0;
    s.params.links[0].joint_viscous = 0.001;
    s.params.F_max = 12;
    s.params.sensor_period = 1e-3;
    s.params.control_period = 1e-3;
    s.q[0] = 0; s.q[1] = Math.PI;
    s.qdot[0] = 0; s.qdot[1] = 0;
    s.u_cmd = 0; s.u_applied = 0; s.u_effective = 0;
    window.__pendulum._resetActuator();
    window.__pendulum.markKDirty();
    s.speed = 10.0;     // run sim 10× wallclock so 45 s sim ≈ 4.5 s wall
    s.running = true;
  });

  // Watch sim time and θ. We sample every 100 ms wall = 1 s sim. Bail once
  // sim time exceeds 45 s.
  const samples = await page.evaluate(async () => {
    const out = [];
    while (true) {
      const s = window.__pendulum.state;
      let th = s.q[1];
      while (th >  Math.PI) th -= 2 * Math.PI;
      while (th < -Math.PI) th += 2 * Math.PI;
      out.push({ tsim: s.t, theta: th, x: s.q[0], u: s.u_cmd });
      if (s.t > 45) break;
      await new Promise(r => setTimeout(r, 100));
      if (out.length > 200) break;  // hard cap
    }
    return out;
  });

  const last5 = samples.slice(-5).map(s => Math.abs(s.theta));
  const tail = last5.reduce((a, b) => a + b) / last5.length;
  assert.ok(tail < 0.175,    // ~10°
    `swing-up + LQR stabilises within 45 s, tail |θ| < 10° (got ${(tail*180/Math.PI).toFixed(1)}°, samples=${samples.length})`);
  await browser.close(); await srv.close();
});

test('Manual swing-up mode keeps energy near E* without LQR catching', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.running = false;
    s.params.ctrl_mode = 'swingup';
    s.params.cart_visc = 0.05;
    s.params.angle_noise = 0; s.params.cart_noise = 0;
    s.params.sensor_delay = 0; s.params.quant_bits = 0;
    s.params.motor_tau = 0; s.params.slew_max = 1e9;
    s.params.F_max = 12;
    s.q[0] = 0; s.q[1] = Math.PI;
    s.qdot[0] = 0; s.qdot[1] = 0;
    s.u_cmd = 0; s.u_applied = 0; s.u_effective = 0;
    window.__pendulum._resetActuator();
    s.speed = 10.0;
    s.running = true;
  });

  // After ~10 s sim time, pendulum energy should be in the neighbourhood of E*.
  // Sample (E, E*) via window — expose totalEnergy/pendulumEnergy if needed.
  const result = await page.evaluate(async () => {
    let tail = [];
    while (true) {
      const s = window.__pendulum.state;
      if (s.t > 12) break;
      if (s.t > 6) tail.push({ q: Array.from(s.q), qdot: Array.from(s.qdot) });
      await new Promise(r => setTimeout(r, 50));
      if (tail.length > 200) break;
    }
    return tail;
  });
  // Compute pendulum energy on the test side (same formula as swingup.js).
  const params_l = 0.25, params_m = 0.2, params_g = 9.81, params_I = 0.2*0.5*0.5/12;
  const Estar = params_m * params_g * params_l;
  const Es = result.map(s => {
    const th = s.q[1], thd = s.qdot[1];
    return 0.5 * (params_I + params_m * params_l * params_l) * thd * thd
         + params_m * params_g * params_l * Math.cos(th);
  });
  const Emean = Es.reduce((a, b) => a + b, 0) / Es.length;
  assert.ok(Math.abs(Emean - Estar) < 0.6 * Estar,
    `swingup-only keeps E≈E* (mean ${Emean.toFixed(3)} vs E*=${Estar.toFixed(3)}, n=${Es.length})`);
  await browser.close(); await srv.close();
});
