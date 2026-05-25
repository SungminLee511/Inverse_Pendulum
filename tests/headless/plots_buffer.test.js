// tests/headless/plots_buffer.test.js — TimeSeries ring-buffer correctness.
//
// We can't import the full plots module from Node (it touches the DOM in
// initPlots), so we shim `document` + `window` first, then import. The
// exposed TimeSeries class is then exercised directly.

import { test } from 'node:test';
import assert from 'node:assert/strict';

globalThis.window = { devicePixelRatio: 1 };
globalThis.document = {
  getElementById: () => null,    // initPlots won't be called in these tests
};

const plots = await import('../../src/ui/plots.js');
const { TimeSeries, wrap } = plots._internal;

test('TimeSeries: empty initial state', () => {
  const ts = new TimeSeries(8);
  assert.equal(ts.n, 0);
  assert.equal(ts.head, 0);
});

test('TimeSeries: push appends rows and grows n up to cap', () => {
  const ts = new TimeSeries(4);
  ts.push(0.0, [1]); ts.push(0.1, [2]); ts.push(0.2, [3]);
  assert.equal(ts.n, 3);
  // forEach should yield (0.0,1)(0.1,2)(0.2,3) in order
  const seen = [];
  ts.forEach((t, row) => seen.push([t, row[0]]));
  assert.deepEqual(seen, [[0, 1], [0.1, 2], [0.2, 3]]);
});

test('TimeSeries: ring wraps when cap exceeded', () => {
  const ts = new TimeSeries(3);
  for (let k = 0; k < 5; k++) ts.push(k * 0.1, [k]);
  assert.equal(ts.n, 3, 'capped at cap');
  const seen = [];
  ts.forEach((t, row) => seen.push(row[0]));
  // Last 3 values written are 2,3,4 — in time order
  assert.deepEqual(seen, [2, 3, 4]);
});

test('TimeSeries: forEach is chronological after wrap', () => {
  const ts = new TimeSeries(5);
  for (let k = 0; k < 10; k++) ts.push(k * 0.1, [k]);
  let prev = -Infinity;
  let mono = true;
  ts.forEach((t) => { if (t < prev) mono = false; prev = t; });
  assert.ok(mono, 'forEach yields rows in increasing time');
});

test('TimeSeries: clear empties without resizing', () => {
  const ts = new TimeSeries(3);
  ts.push(0.0, [1]); ts.push(0.1, [2]);
  ts.clear();
  assert.equal(ts.n, 0);
  assert.equal(ts.head, 0);
  // can still push after clear
  ts.push(0.5, [9]);
  assert.equal(ts.n, 1);
});

test('TimeSeries: trange reports oldest/newest after wrap', () => {
  const ts = new TimeSeries(3);
  ts.push(1.0, [10]); ts.push(2.0, [20]); ts.push(3.0, [30]);
  let [lo, hi] = ts.trange();
  assert.equal(lo, 1);
  assert.equal(hi, 3);
  ts.push(4.0, [40]);
  [lo, hi] = ts.trange();
  assert.equal(lo, 2);
  assert.equal(hi, 4);
});

test('TimeSeries: multi-channel push lazily allocates channels', () => {
  const ts = new TimeSeries(4);
  ts.push(0.0, [1, 2, 3]);
  ts.push(0.1, [4, 5, 6]);
  const seen = [];
  ts.forEach((t, row) => seen.push(row.slice()));
  assert.deepEqual(seen[0], [1, 2, 3]);
  assert.deepEqual(seen[1], [4, 5, 6]);
});

test('wrap(): maps any real angle into (−π, π]', () => {
  assert.equal(wrap(0),      0);
  assert.equal(wrap(Math.PI), Math.PI);
  assert.ok(Math.abs(wrap(3 * Math.PI) - Math.PI) < 1e-12);
  assert.ok(Math.abs(wrap(-3 * Math.PI) - Math.PI) < 1e-12);
  assert.ok(Math.abs(wrap(2 * Math.PI)) < 1e-12);
});
