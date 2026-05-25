// tests/ui/panel_full.test.js — Phase 7 panel + presets + keyboard tests.

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

test('Per-link sliders appear for n=1 (6 entries × 1 link)', async () => {
  const { srv, browser, page } = await setup();
  const labels = await page.locator('.per-link-root .slider-row label').allTextContents();
  const link1 = labels.filter(l => l.endsWith('1') || l.includes('1 '));
  assert.ok(link1.length >= 6, `link-1 has ≥ 6 sliders (got ${link1.length}: ${labels.join('|')})`);
  await browser.close(); await srv.close();
});

test('Switching to n=2 rebuilds per-link block (12 entries)', async () => {
  const { srv, browser, page } = await setup();
  await page.locator('.mode-btn[data-mode="2"]').click();
  await page.waitForTimeout(50);
  const count = await page.locator('.per-link-root .slider-row').count();
  assert.equal(count, 12, `n=2 has 12 link sliders (got ${count})`);
  await browser.close(); await srv.close();
});

test('Switching to n=3 rebuilds per-link block (18 entries) and 8 Q-diag sliders', async () => {
  const { srv, browser, page } = await setup();
  await page.locator('.mode-btn[data-mode="3"]').click();
  await page.waitForTimeout(50);
  const linkRows = await page.locator('.per-link-root .slider-row').count();
  assert.equal(linkRows, 18, `n=3 has 18 link sliders (got ${linkRows})`);
  const qRows = await page.locator('.qdiag-root .slider-row').count();
  assert.equal(qRows, 8, `n=3 has 8 Q-diag sliders (got ${qRows})`);
  await browser.close(); await srv.close();
});

test('Per-link slider edit propagates to state.params.links[i].field', async () => {
  const { srv, browser, page } = await setup();
  // Pick the m₁ numeric input and set it to 0.5.
  const numIn = page.locator('.per-link-root .slider-row')
    .filter({ hasText: 'm1' })
    .locator('input[type=number]');
  await numIn.fill('0.5');
  await numIn.dispatchEvent('change');
  await page.waitForTimeout(50);
  const m1 = await page.evaluate(() => window.__pendulum.state.params.links[0].m);
  assert.ok(Math.abs(m1 - 0.5) < 1e-6, `m₁ slider edit → state.params.links[0].m (got ${m1})`);
  await browser.close(); await srv.close();
});

test('Preset Load applies "stiff-triple" preset (n=3, F_max=80, sensor_delay=1e-3)', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => {
    document.getElementById('preset-select').value = 'stiff-triple';
  });
  await page.locator('#preset-load').click();
  await page.waitForTimeout(80);
  const snap = await page.evaluate(() => {
    const s = window.__pendulum.state;
    return { n: s.n, F_max: s.params.F_max, sensor_delay: s.params.sensor_delay };
  });
  assert.equal(snap.n, 3);
  assert.ok(Math.abs(snap.F_max - 80) < 0.5, `F_max=80 (got ${snap.F_max})`);
  assert.ok(Math.abs(snap.sensor_delay - 1e-3) < 1e-6, `sensor_delay=1e-3 (got ${snap.sensor_delay})`);
  await browser.close(); await srv.close();
});

test('Preset Save → Load round-trip preserves a modified F_max', async () => {
  const { srv, browser, page } = await setup();
  // Set F_max = 42 and save under "default-1".
  await page.evaluate(() => {
    window.__pendulum.setParam('F_max', 42);
    document.getElementById('preset-select').value = 'default-1';
  });
  await page.locator('#preset-save').click();
  // Now scramble F_max, then click Load.
  await page.evaluate(() => { window.__pendulum.setParam('F_max', 3); });
  await page.waitForTimeout(20);
  await page.locator('#preset-load').click();
  await page.waitForTimeout(80);
  const F = await page.evaluate(() => window.__pendulum.state.params.F_max);
  assert.ok(Math.abs(F - 42) < 0.5, `round-tripped F_max=42 (got ${F})`);
  await browser.close(); await srv.close();
});

test('Keyboard K applies a kick (qdot[1] changes by ~magnitude)', async () => {
  const { srv, browser, page } = await setup();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.running = false;
    s.params.ctrl_mode = 'off';
    s.u_cmd = 0; s.u_applied = 0;
    s.q[1] = Math.PI; s.qdot[1] = 0;
    document.getElementById('kick-mag').value = '5';
  });
  // Body needs focus for keydown — click on the canvas first.
  await page.locator('#pendulum-canvas').click();
  await page.keyboard.press('k');
  await page.waitForTimeout(20);
  const qd1 = await page.evaluate(() => window.__pendulum.state.qdot[1]);
  // Allow modest sim drift while the event flow plays out — the kick adds 5,
  // any background dynamics shouldn't move qdot by more than ~1 rad/s.
  assert.ok(Math.abs(qd1 - 5) < 1.0, `qdot[1] ≈ +5 from K kick (got ${qd1})`);
  await browser.close(); await srv.close();
});

test('Keyboard Space toggles running', async () => {
  const { srv, browser, page } = await setup();
  await page.locator('#pendulum-canvas').click();
  const r1 = await page.evaluate(() => window.__pendulum.state.running);
  await page.keyboard.press('Space');
  await page.waitForTimeout(20);
  const r2 = await page.evaluate(() => window.__pendulum.state.running);
  assert.notEqual(r1, r2, 'Space toggles state.running');
  await browser.close(); await srv.close();
});

test('Keyboard R resets sim time', async () => {
  const { srv, browser, page } = await setup();
  await page.waitForTimeout(300);    // let sim run for a bit
  await page.locator('#pendulum-canvas').click();
  await page.keyboard.press('r');
  await page.waitForTimeout(20);
  const t = await page.evaluate(() => window.__pendulum.state.t);
  assert.ok(t < 0.05, `R reset sim time → ~0 (got ${t})`);
  await browser.close(); await srv.close();
});

test('Total slider count ≥ 22 (mode 1)', async () => {
  const { srv, browser, page } = await setup();
  const n = await page.locator('.slider-row input[type=range]').count();
  assert.ok(n >= 22, `Total sliders ≥ 22 (got ${n})`);
  await browser.close(); await srv.close();
});
