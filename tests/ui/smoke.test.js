// tests/ui/smoke.test.js — Phase 1 UI smoke check via Playwright.
// Asserts: page loads, mode buttons toggle, canvas renders pixels, no console errors.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { chromium } from 'playwright';
import { startServer } from '../_static_server.js';

test('page loads without console errors', async () => {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const ctx = await browser.newContext();
  const page = await ctx.newPage();
  const errors = [];
  page.on('pageerror', e => errors.push(`pageerror: ${e.message}`));
  page.on('console', m => { if (m.type() === 'error') errors.push(`console.error: ${m.text()}`); });

  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });
  // give the rAF loop a tick
  await page.waitForTimeout(300);

  const title = await page.title();
  assert.match(title, /Inverted Pendulum/, 'title contains expected text');
  assert.deepEqual(errors, [], 'no console / page errors');

  await browser.close(); await srv.close();
});

test('mode buttons exist and toggle active class', async () => {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const page = await (await browser.newContext()).newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });

  const btns = page.locator('.mode-btn');
  assert.equal(await btns.count(), 3, 'three mode buttons');

  await btns.nth(1).click();   // n=2
  await page.waitForTimeout(50);
  assert.equal(await btns.nth(0).getAttribute('class'), 'mode-btn', '1-link no longer active');
  assert.match(await btns.nth(1).getAttribute('class'), /active/, '2-link is active');
  const hudMode = await page.locator('#hud-mode').textContent();
  assert.equal(hudMode.trim(), 'n=2', 'HUD reflects n=2');

  await btns.nth(2).click();   // n=3
  await page.waitForTimeout(50);
  const hudMode3 = await page.locator('#hud-mode').textContent();
  assert.equal(hudMode3.trim(), 'n=3', 'HUD reflects n=3');

  await browser.close(); await srv.close();
});

test('canvas has non-trivial pixel content after rAF tick', async () => {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const page = await (await browser.newContext()).newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });
  await page.waitForTimeout(500);

  const distinctPixels = await page.evaluate(() => {
    const c = document.getElementById('pendulum-canvas');
    const ctx = c.getContext('2d');
    const w = c.width, h = c.height;
    const data = ctx.getImageData(0, 0, w, h).data;
    const colors = new Set();
    // sample every 41st pixel to keep this cheap
    for (let i = 0; i < data.length; i += 41 * 4) {
      colors.add((data[i] << 16) | (data[i + 1] << 8) | data[i + 2]);
      if (colors.size > 30) break;
    }
    return colors.size;
  });
  assert.ok(distinctPixels >= 3, `canvas has > 3 distinct colors (got ${distinctPixels}) — rendering`);

  await browser.close(); await srv.close();
});

test('play/pause button toggles', async () => {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const page = await (await browser.newContext()).newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });
  const btn = page.locator('#btn-playpause');
  assert.equal((await btn.textContent()).trim(), '⏸', 'initially playing');
  await btn.click();
  await page.waitForTimeout(50);
  assert.equal((await btn.textContent()).trim(), '▶', 'paused after click');
  await btn.click();
  await page.waitForTimeout(50);
  assert.equal((await btn.textContent()).trim(), '⏸', 'playing again');
  await browser.close(); await srv.close();
});

test('parameter panel groups are populated', async () => {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const page = await (await browser.newContext()).newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });

  const groups = await page.locator('.panel-group').count();
  assert.equal(groups, 4, 'four panel groups');
  const sliders = await page.locator('.slider-row input[type=range]').count();
  assert.ok(sliders >= 5, `at least 5 sliders scaffolded (got ${sliders})`);

  await browser.close(); await srv.close();
});

test('keyboard shortcut R resets sim time', async () => {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const page = await (await browser.newContext()).newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });
  await page.waitForTimeout(800);
  const tBefore = await page.locator('#hud-t').textContent();
  // Should be > 0 by now
  await page.locator('body').click();
  await page.keyboard.press('r');
  await page.waitForTimeout(50);
  const tAfter = await page.locator('#hud-t').textContent();
  assert.match(tAfter, /^t = 0\.0/, `time reset to ~0 (was ${tBefore} → ${tAfter})`);
  await browser.close(); await srv.close();
});
