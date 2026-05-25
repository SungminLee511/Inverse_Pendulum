// tests/ui/actuator_live.test.js — actuator wiring in browser.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { chromium } from 'playwright';
import { startServer } from '../_static_server.js';

test('state.u_cmd → state.u_applied propagates through actuator (controller off)', async () => {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const page = await (await browser.newContext()).newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });

  // Disable the controller (it leaves u_cmd alone) and set u_cmd directly.
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'off';
    s.u_cmd = 5;
  });
  await page.waitForTimeout(400);
  const u_applied = await page.evaluate(() => window.__pendulum.state.u_applied);
  assert.ok(Number.isFinite(u_applied), 'u_applied is finite');
  assert.ok(Math.abs(u_applied - 5) < 0.5, `u_applied tracks command (got ${u_applied})`);

  await browser.close(); await srv.close();
});

test('Saturation clips u_cmd above F_max', async () => {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const page = await (await browser.newContext()).newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });

  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'off';
    s.params.F_max = 10;
    s.u_cmd = 1000;
  });
  await page.waitForTimeout(400);
  const u_applied = await page.evaluate(() => window.__pendulum.state.u_applied);
  assert.ok(Math.abs(u_applied - 10) < 0.5, `clipped to F_max=10 (got ${u_applied})`);

  await browser.close(); await srv.close();
});
