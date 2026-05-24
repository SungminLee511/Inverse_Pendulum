// tests/ui/panel_live.test.js — slider mutations actually change behaviour.

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

test('changing F_max slider changes saturation behaviour', async () => {
  const { srv, browser, page } = await setup();
  // Find the F_max number input
  const numIn = page.locator('.panel-group[data-group="sensor-actuator"] .slider-row').filter({ hasText: 'F_max' }).locator('input[type=number]');
  await numIn.fill('8');
  await numIn.dispatchEvent('change');
  await page.evaluate(() => { window.__pendulum.state.u_cmd = 1000; });
  await page.waitForTimeout(400);
  const u_applied = await page.evaluate(() => window.__pendulum.state.u_applied);
  assert.ok(Math.abs(u_applied - 8) < 0.5, `F_max slider 8 limits u_applied to ~8 (got ${u_applied})`);
  await browser.close(); await srv.close();
});

test('Bumping integrator dropdown to Euler still keeps sim alive', async () => {
  const { srv, browser, page } = await setup();
  const sel = page.locator('.panel-group[data-group="sim"] select');
  await sel.selectOption('euler');
  await page.evaluate(() => { window.__pendulum.state.q[1] = Math.PI - 0.6; });
  await page.waitForTimeout(500);
  const finite = await page.evaluate(() => {
    const s = window.__pendulum.state;
    return s.q.every(Number.isFinite) && s.qdot.every(Number.isFinite);
  });
  assert.ok(finite, 'Euler integrator keeps state finite');
  await browser.close(); await srv.close();
});

test('Lowering angle_noise σ to 0 zeroes noise variance on sensor channel', async () => {
  const { srv, browser, page } = await setup();
  const numIn = page.locator('.panel-group[data-group="sensor-actuator"] .slider-row').filter({ hasText: 'angle σ' }).locator('input[type=number]');
  await numIn.fill('0');
  await numIn.dispatchEvent('change');
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.cart_visc = 0;
    s.params.links[0].joint_viscous = 0;
    s.q[1] = Math.PI;
    s.qdot[1] = 0;
  });
  await page.waitForTimeout(300);
  // Pendulum hanging at rest with zero noise → sensor_last[1] should be ~π every tick
  const samples = await page.evaluate(async () => {
    const out = [];
    for (let i = 0; i < 20; i++) {
      out.push(Array.from(window.__pendulum.state.sensor_last));
      await new Promise(r => setTimeout(r, 30));
    }
    return out;
  });
  const angle_samples = samples.map(s => s[1]);
  const mean = angle_samples.reduce((a,b)=>a+b)/angle_samples.length;
  const std = Math.sqrt(angle_samples.reduce((a,b)=>a+(b-mean)*(b-mean),0)/angle_samples.length);
  // After setting σ=0, the dominant remaining variation is the quantization LSB (~0.0015 rad at 12-bit).
  assert.ok(std < 0.005, `sensor angle std small when σ=0 (got ${std.toExponential(2)})`);
  await browser.close(); await srv.close();
});

test('All expected sliders are present', async () => {
  const { srv, browser, page } = await setup();
  const sliders = await page.locator('.slider-row input[type=range]').count();
  assert.ok(sliders >= 16, `at least 16 sliders present (got ${sliders})`);
  await browser.close(); await srv.close();
});
