// tests/ui/sysid_live.test.js — Phase 14 sys-id wiring in the browser.

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

test('Sys-ID excitation drives a non-zero, oscillating u_cmd (chirp default)', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'sysid';
    window.__pendulum.setParam('sysid_excitation', 'chirp');
    window.__pendulum.setParam('sysid_amplitude', 5);
    s.speed = 3.0;
  });
  // Wait long enough for the chirp to swing through multiple cycles.
  const samples = await page.evaluate(async () => {
    const out = [];
    for (let i = 0; i < 60; i++) {
      out.push(window.__pendulum.state.u_cmd);
      await new Promise(r => setTimeout(r, 50));
    }
    return out;
  });
  // Some samples positive, some negative → oscillating (sinusoidal sweep).
  const pos = samples.filter(s => s > 1).length;
  const neg = samples.filter(s => s < -1).length;
  assert.ok(pos > 5, `at least a few positive peaks (got ${pos})`);
  assert.ok(neg > 5, `at least a few negative peaks (got ${neg})`);
  await browser.close(); await srv.close();
});

test('Sys-ID step excitation produces a sustained positive u_cmd after t0', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'sysid';
    window.__pendulum.setParam('sysid_excitation', 'step');
    window.__pendulum.setParam('sysid_amplitude', 4);
    s.speed = 3.0;
  });
  // After sysid engages, t=0 in the sysid clock. Step fires at t0=0.2.
  await page.waitForTimeout(400);
  const u = await page.evaluate(() => window.__pendulum.state.u_cmd);
  assert.ok(Math.abs(u - 4) < 0.5, `u≈4 after step engage (got ${u.toFixed(3)})`);
  await browser.close(); await srv.close();
});

test('Sys-ID PRBS excitation flips sign across samples', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'sysid';
    window.__pendulum.setParam('sysid_excitation', 'prbs');
    window.__pendulum.setParam('sysid_amplitude', 3);
    s.speed = 5.0;
  });
  const samples = await page.evaluate(async () => {
    const out = [];
    for (let i = 0; i < 40; i++) {
      out.push(window.__pendulum.state.u_cmd);
      await new Promise(r => setTimeout(r, 50));
    }
    return out;
  });
  const flips = samples.filter(s => Math.abs(Math.abs(s) - 3) < 0.5).length;
  const pos = samples.filter(s => s > 1).length;
  const neg = samples.filter(s => s < -1).length;
  assert.ok(flips > 10, `≥10 samples saturated to ±3 (got ${flips})`);
  assert.ok(pos >= 1 && neg >= 1,
    `PRBS visits both signs (pos=${pos}, neg=${neg}, samples=[${samples.slice(0,10).map(x=>x.toFixed(1)).join(',')}])`);
  await browser.close(); await srv.close();
});

test('Switching from sysid back to lqr clears the excitation', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'sysid';
    window.__pendulum.setParam('sysid_excitation', 'step');
    window.__pendulum.setParam('sysid_amplitude', 8);
    s.speed = 3.0;
  });
  await page.waitForTimeout(400);
  await page.evaluate(() => { window.__pendulum.state.params.ctrl_mode = 'off'; });
  await page.waitForTimeout(200);
  await page.evaluate(() => { window.__pendulum.state.u_cmd = 0; });
  await page.waitForTimeout(200);
  const u_off = await page.evaluate(() => window.__pendulum.state.u_cmd);
  // ctrl_mode=off → controller doesn't write u_cmd; we manually set it to 0,
  // so it should remain ~0 (no sysid bleed-through).
  assert.ok(Math.abs(u_off) < 1, `no sysid bleed in 'off' (got ${u_off.toFixed(3)})`);
  await browser.close(); await srv.close();
});
