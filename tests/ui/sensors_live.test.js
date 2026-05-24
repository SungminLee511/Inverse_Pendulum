// tests/ui/sensors_live.test.js — sensor module runs inside browser.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { chromium } from 'playwright';
import { startServer } from '../_static_server.js';

test('sensor_last + sensor_vel_est exist after ~1s of sim', async () => {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const page = await (await browser.newContext()).newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });
  // Perturb the pendulum so velocity > 0
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.q[1] = Math.PI - 0.6;
    s.qdot[1] = 1.0;
  });
  await page.waitForTimeout(1000);
  const snap = await page.evaluate(() => {
    const s = window.__pendulum.state;
    return {
      sensor_last: s.sensor_last && Array.from(s.sensor_last),
      sensor_vel_est: s.sensor_vel_est && Array.from(s.sensor_vel_est),
      qdot: Array.from(s.qdot),
      n: s.n,
    };
  });
  assert.ok(Array.isArray(snap.sensor_last), 'sensor_last populated');
  assert.equal(snap.sensor_last.length, snap.n + 1);
  for (const v of snap.sensor_last) assert.ok(Number.isFinite(v));
  for (const v of snap.sensor_vel_est) assert.ok(Number.isFinite(v));
  await browser.close(); await srv.close();
});
