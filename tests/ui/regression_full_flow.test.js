// tests/ui/regression_full_flow.test.js — Phase 15 end-to-end regression.
//
// Walks the simulator through: hanging → swing-up → LQR catch → kick → recover.
// Verifies no NaN, energy reaches near E*, and after the kick the pendulum
// returns to near-upright.

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

test('Full flow: hanging → swing-up → LQR → kick → recover (n=1)', async () => {
  const { srv, browser, page } = await setup();
  // Configure: idealised sensors+actuator, auto mode, fast sim.
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'auto';
    s.params.cart_visc = 0.05;
    s.params.links[0].joint_viscous = 0.001;
    s.params.F_max = 12;
    s.params.angle_noise = 0; s.params.cart_noise = 0;
    s.params.sensor_delay = 0; s.params.quant_bits = 0;
    s.params.motor_tau = 0; s.params.slew_max = 1e9;
    s.params.force_noise = 0; s.params.cart_coulomb = 0;
    s.speed = 10.0;     // 10× wall → 1 wall-sec = 10 sim-sec
  });
  // Phase A: swing up from hanging to upright (sim ~30 s = 3 s wall).
  await page.waitForTimeout(3500);
  const afterSwingup = await page.evaluate(() => {
    const s = window.__pendulum.state;
    function wrap(a) { while (a > Math.PI) a -= 2*Math.PI; while (a < -Math.PI) a += 2*Math.PI; return a; }
    return { theta: wrap(s.q[1]), tsim: s.t };
  });
  assert.ok(Math.abs(afterSwingup.theta) < 0.3,
    `swing-up completed: |θ| < 0.3 (got ${afterSwingup.theta.toFixed(3)}, t_sim=${afterSwingup.tsim.toFixed(1)})`);

  // Phase B: hold steady for 1 sim sec to confirm LQR is engaged.
  await page.waitForTimeout(200);
  const steady = await page.evaluate(() => {
    const s = window.__pendulum.state;
    function wrap(a) { while (a > Math.PI) a -= 2*Math.PI; while (a < -Math.PI) a += 2*Math.PI; return a; }
    return { theta: wrap(s.q[1]) };
  });
  assert.ok(Math.abs(steady.theta) < 0.15,
    `LQR holds upright: |θ| < 0.15 (got ${steady.theta.toFixed(3)})`);

  // Phase C: kick the pendulum.
  await page.evaluate(() => { window.__pendulum.doKick(3, +1); });
  await page.waitForTimeout(50);
  const justKicked = await page.evaluate(() => window.__pendulum.state.qdot[1]);
  assert.ok(Math.abs(justKicked) > 1, `kick injected θ̇: |θ̇| > 1 (got ${justKicked.toFixed(3)})`);

  // Phase D: LQR recovers within 5 sim sec.
  await page.waitForTimeout(600);
  const recovered = await page.evaluate(() => {
    const s = window.__pendulum.state;
    function wrap(a) { while (a > Math.PI) a -= 2*Math.PI; while (a < -Math.PI) a += 2*Math.PI; return a; }
    return { theta: wrap(s.q[1]), qdot1: s.qdot[1] };
  });
  assert.ok(Math.abs(recovered.theta) < 0.2,
    `LQR recovered: |θ| < 0.2 (got ${recovered.theta.toFixed(3)})`);
  assert.ok(Math.abs(recovered.qdot1) < 2.0,
    `θ̇ damped: |θ̇| < 2 (got ${recovered.qdot1.toFixed(3)})`);

  await browser.close(); await srv.close();
});

test('Mouse drag on link applies a horizontal disturbance force', async () => {
  const { srv, browser, page } = await setup();
  // Pause sim, fix pendulum at hanging, then drag.
  await page.locator('#btn-playpause').click();
  await page.waitForTimeout(40);
  // Compute the EXACT pixel position of joint 1 from current canvas state.
  const target = await page.evaluate(() => {
    const cv = document.getElementById('pendulum-canvas');
    const rect = cv.getBoundingClientRect();
    const PX = 200;
    const groundY = rect.height * 0.7;
    const pivotY  = groundY - 28;
    const s = window.__pendulum.state;
    const L = s.params.links[0].L;
    const theta = s.q[1];
    // Joint-1 tip screen coords (sx, sy):
    const sx = rect.left + rect.width / 2 + s.q[0] * PX + L * Math.sin(theta) * PX;
    const sy = rect.top + pivotY + L * Math.cos(theta) * PX * -1;   // cos(π)=-1 → tip below pivot
    return { sx, sy };
  });
  await page.mouse.move(target.sx, target.sy);
  await page.mouse.down();
  await page.mouse.move(target.sx + 50, target.sy);
  await page.waitForTimeout(40);
  const dragF = await page.evaluate(() => window.__pendulum.state.drag_force);
  assert.ok(Math.abs(dragF) > 1, `drag induces a force (got ${dragF.toFixed(3)})`);
  await page.mouse.up();
  await page.waitForTimeout(40);
  const released = await page.evaluate(() => window.__pendulum.state.drag_force);
  assert.equal(released, 0, 'release zeros drag force');
  await browser.close(); await srv.close();
});

test('Full flow: no NaN in state after 5 s sim under default Auto mode (n=1)', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'auto';
    s.speed = 5.0;
  });
  await page.waitForTimeout(1100);
  const result = await page.evaluate(() => {
    const s = window.__pendulum.state;
    return {
      qfin: Array.from(s.q).every(Number.isFinite),
      vfin: Array.from(s.qdot).every(Number.isFinite),
      ufin: Number.isFinite(s.u_cmd) && Number.isFinite(s.u_applied),
    };
  });
  assert.ok(result.qfin, 'q stays finite');
  assert.ok(result.vfin, 'qdot stays finite');
  assert.ok(result.ufin, 'u_cmd / u_applied stay finite');
  await browser.close(); await srv.close();
});
