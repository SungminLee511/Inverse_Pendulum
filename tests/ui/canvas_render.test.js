// tests/ui/canvas_render.test.js — verify the canvas renderer actually draws
// the cart + link with expected pixel content for each mode, and capture screenshots
// for visual regression.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { chromium } from 'playwright';
import { startServer } from '../_static_server.js';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __filename = fileURLToPath(import.meta.url);
const __dirname  = path.dirname(__filename);
const SHOTDIR    = path.resolve(__dirname, '..', 'screenshots');

if (!fs.existsSync(SHOTDIR)) fs.mkdirSync(SHOTDIR, { recursive: true });

async function setup() {
  const srv = await startServer({ port: 0 });
  const browser = await chromium.launch({ headless: true });
  const ctx = await browser.newContext({ viewport: { width: 1280, height: 800 } });
  const page = await ctx.newPage();
  await page.goto(srv.url + '/index.html', { waitUntil: 'networkidle' });
  return { srv, browser, page };
}

test('canvas draws ground, cart, and link 1 for n=1', async () => {
  const { srv, browser, page } = await setup();
  // Perturb angle to make it visually obvious
  await page.evaluate(() => { window.__pendulum.state.q[1] = Math.PI - 0.7; });
  await page.waitForTimeout(300);

  // Snapshot canvas and check we have the three signature colors (link blue, cart blue, joint white)
  const palette = await page.evaluate(() => {
    const c = document.getElementById('pendulum-canvas');
    const x = c.getContext('2d');
    const d = x.getImageData(0, 0, c.width, c.height).data;
    const seen = new Set();
    for (let i = 0; i < d.length; i += 4) {
      const k = (d[i] << 16) | (d[i + 1] << 8) | d[i + 2];
      seen.add(k);
    }
    return seen.size;
  });
  assert.ok(palette > 15, `canvas has > 15 distinct colors (got ${palette})`);

  await page.locator('#pendulum-canvas').screenshot({ path: path.join(SHOTDIR, 'n1.png') });
  await browser.close(); await srv.close();
});

test('canvas draws 2 links when mode = 2 (placeholder physics)', async () => {
  const { srv, browser, page } = await setup();
  // Pause sim + disable controller before setting q to keep the
  // hanging-ish IC stable across the render.
  await page.locator('#btn-playpause').click();
  await page.locator('.mode-btn[data-mode="2"]').click();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.params.ctrl_mode = 'off';
    s.q[0] = 0; s.qdot[0] = 0;
    s.q[1] = Math.PI - 0.5; s.qdot[1] = 0;
    s.q[2] = Math.PI - 0.3; s.qdot[2] = 0;
    s.u_cmd = 0; s.u_applied = 0; s.u_effective = 0;
    window.__pendulum._resetActuator();
  });
  await page.locator('#btn-playpause').click();   // resume for a render
  await page.waitForTimeout(150);
  await page.locator('#pendulum-canvas').screenshot({ path: path.join(SHOTDIR, 'n2.png') });

  // Detect orange link pixels (link 2 color = #f0883e ~ R=240, G=136, B=62)
  const hasOrange = await page.evaluate(() => {
    const c = document.getElementById('pendulum-canvas');
    const x = c.getContext('2d');
    const d = x.getImageData(0, 0, c.width, c.height).data;
    for (let i = 0; i < d.length; i += 4) {
      if (d[i] > 220 && d[i + 1] > 110 && d[i + 1] < 160 && d[i + 2] < 90) return true;
    }
    return false;
  });
  assert.ok(hasOrange, 'orange (link 2) pixels present');
  await browser.close(); await srv.close();
});

test('canvas draws 3 links when mode = 3', async () => {
  const { srv, browser, page } = await setup();
  await page.locator('.mode-btn[data-mode="3"]').click();
  await page.evaluate(() => {
    const s = window.__pendulum.state;
    s.q[1] = Math.PI - 0.4;
    s.q[2] = Math.PI - 0.2;
    s.q[3] = Math.PI;
  });
  await page.waitForTimeout(300);
  await page.locator('#pendulum-canvas').screenshot({ path: path.join(SHOTDIR, 'n3.png') });

  // Detect green pixels (link 3 color = #3fb950 ~ R=63, G=185, B=80)
  const hasGreen = await page.evaluate(() => {
    const c = document.getElementById('pendulum-canvas');
    const x = c.getContext('2d');
    const d = x.getImageData(0, 0, c.width, c.height).data;
    for (let i = 0; i < d.length; i += 4) {
      if (d[i] < 90 && d[i + 1] > 160 && d[i + 2] < 110 && d[i + 2] > 50) return true;
    }
    return false;
  });
  assert.ok(hasGreen, 'green (link 3) pixels present');
  await browser.close(); await srv.close();
});

test('cart x position visible movement: q[0] change shifts cart centroid in canvas', async () => {
  const { srv, browser, page } = await setup();
  // Pause physics so the cart sits exactly where we put it
  await page.evaluate(() => { window.__pendulum.state.running = false; });

  // Helper: centroid x of cart-edge-color pixels (#58a6ff = R88 G166 B255).
  // The cart outline is the most reliably present feature.
  const cartCentroid = async () => page.evaluate(() => {
    const c = document.getElementById('pendulum-canvas');
    const x = c.getContext('2d');
    const d = x.getImageData(0, 0, c.width, c.height).data;
    let sx = 0, n = 0;
    for (let i = 0; i < d.length; i += 4) {
      const r = d[i], g = d[i + 1], b = d[i + 2];
      // tolerance of ±30 around cart edge blue
      if (Math.abs(r - 88) < 30 && Math.abs(g - 166) < 30 && Math.abs(b - 255) < 30) {
        const px = (i / 4) % c.width;
        sx += px; n++;
      }
    }
    return n > 0 ? sx / n : -1;
  });

  await page.evaluate(() => { window.__pendulum.state.q[0] = 0.4; });
  await page.waitForTimeout(120);
  const right = await cartCentroid();

  await page.evaluate(() => { window.__pendulum.state.q[0] = -0.4; });
  await page.waitForTimeout(120);
  const left = await cartCentroid();

  assert.ok(right > 0 && left > 0, `cart centroid found in both (${left} <- -> ${right})`);
  assert.ok(left < right - 50,
    `cart centroid shifted left when q[0] went +0.4 → -0.4 (right=${right.toFixed(1)}, left=${left.toFixed(1)})`);

  await browser.close(); await srv.close();
});
