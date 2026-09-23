// MultiPipe2 planner tests. Run from the repo root: node --test
const test = require('node:test');
const assert = require('node:assert');
const { plan, objLines, checkImport } = require('../../../scripts/multipipe2/MultiPipe2Planner.js');
const scenes = require('../scenes/scenes.json');
const v1Scenes = require('../../multipipe/scenes/scenes.json');

const R = 2;
const W = R / 0.93;
const opts = { radius: R, nodeSize: 1.6, tolerance: 0.001 };
const lines = (spec) => spec.map((c) => ({ kind: 'line', start: c.pts[0], end: c.pts[c.pts.length - 1] }));
const run = (name, o) => plan(lines(scenes[name]), { ...opts, ...o });
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

// Every edge used by exactly two faces, once in each direction; positive volume means it winds outward.
function assertClosedAndWound(cage, name) {
  const directed = new Set(), count = new Map();
  let vol = 0;
  for (const f of cage.faces) {
    f.forEach((a, i) => {
      const b = f[(i + 1) % f.length];
      assert.ok(!directed.has(a + '>' + b), name + ': edge ' + a + '>' + b + ' twice in one direction');
      directed.add(a + '>' + b);
      const k = Math.min(a, b) + '_' + Math.max(a, b);
      count.set(k, (count.get(k) || 0) + 1);
    });
    for (let t = 1; t + 1 < f.length; t++) vol += dot(cage.vertices[f[0]], cross(cage.vertices[f[t]], cage.vertices[f[t + 1]]));
  }
  for (const [k, n] of count) assert.strictEqual(n, 2, name + ': edge ' + k + ' used ' + n + ' times');
  assert.ok(vol > 0, name + ': winds inward');
}

// Distance from p to the nearest scene line.
function toCurves(p, spec) {
  let best = Infinity;
  for (const c of spec) {
    const a = c.pts[0], ab = sub(c.pts[1], a), t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / dot(ab, ab)));
    const q = sub(p, [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t]);
    best = Math.min(best, Math.sqrt(dot(q, q)));
  }
  return best;
}

test('scenes give closed, outward-wound cages close to their curves', () => {
  const want = { line: [1, 0, 2], straight: [1, 1, 2], bend90: [1, 1, 2], y3ortho: [1, 1, 3], x4planar: [1, 1, 4],
    cubeframe: [1, 8, 0], twobends: [2, 2, 4] };
  for (const name of Object.keys(want)) {
    const cage = run(name), r = cage.report;
    assert.deepStrictEqual(r.errors, [], name);
    assert.deepStrictEqual([r.pipeFrames, r.nodes, r.freeEnds], want[name], name);
    assert.strictEqual(r.struts, scenes[name].length, name);
    assertClosedAndWound(cage, name);
    for (const v of cage.vertices) assert.ok(toCurves(v, scenes[name]) <= W * Math.SQRT2 + 1e-9, name + ': vertex far from the curves');
  }
});

test('a lone strut: rings of half-width radius / 0.93 at both ends and 1 x radius in, both ends capped', () => {
  const cage = run('line');
  const xs = [...new Set(cage.vertices.map((v) => +v[0].toFixed(9)))].sort((a, b) => a - b);
  assert.deepStrictEqual(xs, [0, R, 30 - R, 30]);
  for (const v of cage.vertices) assert.ok(Math.abs(Math.abs(v[1]) - W) < 1e-9 && Math.abs(Math.abs(v[2]) - W) < 1e-9);
  // Two cap faces: one quad at each end.
  const caps = cage.faces.filter((f) => f.every((i) => cage.vertices[i][0] === 0) || f.every((i) => cage.vertices[i][0] === 30));
  assert.strictEqual(caps.length, 2);
  assert.deepStrictEqual(cage.box.map((x) => +x.toFixed(9)), [0, -W, -W, 30, W, W].map((x) => +x.toFixed(9)));
});

test('rings sit nodeSize x radius from a node', () => {
  const cage = run('straight');
  const xs = [...new Set(cage.vertices.map((v) => +v[0].toFixed(9)))].sort((a, b) => a - b);
  assert.deepStrictEqual(xs, [0, R, 30 - 1.6 * R, 30 + 1.6 * R, 60 - R, 60].map((x) => +x.toFixed(9)));
});

test('cap off: exactly four open edges at each free end, none anywhere else; no free ends stays closed', () => {
  const frames = { line: 1, bend90: 1, twobends: 2, polyframe: 1, cubeframe: 1, hairpin30: 1 };
  for (const name of Object.keys(frames)) {
    const spec = scenes[name], cage = plan(v1Input(spec), { ...opts, cap: false }), r = cage.report;
    assert.deepStrictEqual(r.errors, [], name);
    assert.strictEqual(r.pipeFrames, frames[name], name);
    // Free ends: segment ends that appear once.
    const ends = spec.flatMap((c) => c.pts.slice(1).flatMap((p, i) => [c.pts[i], p]));
    const free = ends.filter((p) => ends.filter((q) => Math.hypot(...sub(p, q)) < 1e-9).length === 1);
    assert.strictEqual(r.freeEnds, free.length, name);
    if (!free.length) { assertClosedAndWound(cage, name); continue; }
    // Edges: each used twice in opposite directions, except the open ones, used once.
    const directed = new Set(), count = new Map();
    for (const f of cage.faces) f.forEach((a, i) => {
      const b = f[(i + 1) % f.length], k = Math.min(a, b) + '_' + Math.max(a, b);
      assert.ok(!directed.has(a + '>' + b), name + ': edge ' + a + '>' + b + ' twice in one direction');
      directed.add(a + '>' + b);
      count.set(k, (count.get(k) || 0) + 1);
    });
    const perEnd = free.map(() => 0);
    for (const [k, n] of count) {
      if (n === 2) continue;
      assert.strictEqual(n, 1, name + ': edge ' + k + ' used ' + n + ' times');
      // An open edge's two vertices both sit on one free end's ring, W * sqrt2 from the end point.
      const [a, b] = k.split('_').map((i) => cage.vertices[+i]);
      const e = free.findIndex((p) => Math.abs(Math.hypot(...sub(a, p)) - W * Math.SQRT2) < 1e-9 &&
        Math.abs(Math.hypot(...sub(b, p)) - W * Math.SQRT2) < 1e-9);
      assert.ok(e >= 0, name + ': open edge ' + k + ' away from every free end');
      perEnd[e]++;
    }
    assert.deepStrictEqual(perEnd, free.map(() => 4), name);
  }
});

test('endpoints within tolerance meet at one node', () => {
  const p = plan([{ kind: 'line', start: [0, 0, 0], end: [30, 0, 0] }, { kind: 'line', start: [0, 0.0005, 0], end: [0, 30, 0] }], opts);
  assert.deepStrictEqual([p.report.nodes, p.report.freeEnds, p.report.pipeFrames], [1, 2, 1]);
});

test('errors: bad options, no curves, zero length, not a line', () => {
  assert.match(run('line', { radius: 0 }).report.errors[0], /Radius/);
  assert.match(run('line', { nodeSize: 0.9 }).report.errors[0], /Node size/);
  assert.match(plan([], opts).report.errors[0], /at least one curve/);
  assert.match(plan([{ kind: 'line', start: [1, 1, 1], end: [1, 1, 1] }], opts).report.errors[0], /zero length/);
  assert.match(plan([{ kind: 'curve' }], opts).report.errors[0], /not a straight line or polyline/);
  assert.match(plan([{ kind: 'polyline', points: [[1, 1, 1], [1, 1, 1]] }], opts).report.errors[0], /zero length/);
});

test('OBJ is Y-up (x, z, -y), vertices then 1-based faces, no groups or materials', () => {
  const cage = { vertices: [[1, 2, 3], [4, 5, 6], [7, 8, 9]], faces: [[0, 1, 2]] };
  assert.deepStrictEqual(objLines(cage), ['v 1.000000 3.000000 -2.000000', 'v 4.000000 6.000000 -5.000000',
    'v 7.000000 9.000000 -8.000000', 'f 1 2 3']);
  assert.ok(objLines(run('cubeframe')).every((l) => /^[vf] /.test(l)));
});

test('import check: inside the cage box and at least 80% of it on every axis', () => {
  const box = [0, 0, 0, 10, 10, 10];
  assert.strictEqual(checkImport(box, [[0.5, 0.5, 0.5, 9.5, 9.5, 9.5]], 0.01), '');
  assert.strictEqual(checkImport(box, [[0, 0, 0, 5, 10, 10], [5, 0, 0, 10, 10, 10]], 0.01), '');
  assert.match(checkImport(box, [[0, 0, 0, 20, 20, 20]], 0.01), /ScaleFactor/);
  assert.match(checkImport(box, [[2, 2, 2, 8, 8, 8]], 0.01), /ScaleFactor/);
  assert.match(checkImport(box, [[0, 0, 0, 10, 10, 10.1]], 0.01), /ScaleFactor/);
});

test('tight angles grow the node: one closed cage, reach as a factor of radius', () => {
  const want = { hairpin30: [1, 5.96], k5skew: [1, 2.39], d8: [1, 2.26], roofTruss: [6, 4.55], cubeframe: [0, 0] };
  for (const name of Object.keys(want)) {
    const cage = run(name), r = cage.report;
    assert.deepStrictEqual(r.errors, [], name);
    assert.strictEqual(r.pipeFrames, 1, name);
    assert.strictEqual(r.grownNodes, want[name][0], name);
    assert.ok(Math.abs(r.largestReach - want[name][1]) < 0.01, name + ': reach ' + r.largestReach);
    assert.strictEqual(r.shortStruts, 0, name);
    assertClosedAndWound(cage, name);
  }
});

test('a strut shorter than its two end offsets is counted and still built', () => {
  // A 30-degree hairpin with an 8-long arm: the node needs about 11.9, the free end 2.
  const p = plan([{ kind: 'line', start: [0, 0, 0], end: [30, 0, 0] },
    { kind: 'line', start: [0, 0, 0], end: [8 * Math.cos(Math.PI / 6), 8 * Math.sin(Math.PI / 6), 0] }], opts);
  assert.deepStrictEqual(p.report.errors, []);
  assert.strictEqual(p.report.shortStruts, 1);
  assertClosedAndWound(p, 'short strut');
});

test('two struts in the same direction at a node are an error, not a cage', () => {
  const p = plan([{ kind: 'line', start: [0, 0, 0], end: [30, 0, 0] }, { kind: 'line', start: [0, 0, 0], end: [20, 0, 0] }], opts);
  assert.match(p.report.errors[0], /same direction/);
  assert.strictEqual(p.faces.length, 0);
});

// v1's shared scenes, at v1's scale (radius 0.5), as MultiPipe2 input.
const v1Opts = { radius: 0.5, nodeSize: 1.6, tolerance: 0.001 };
const v1Input = (spec) => spec.map((c) => c.type === 'polyline' ? { kind: 'polyline', points: c.pts }
  : { kind: 'line', start: c.pts[0], end: c.pts[c.pts.length - 1] });
const v1Run = (name, o) => plan(v1Input(v1Scenes[name]), { ...v1Opts, ...o });
const line = (start, end) => ({ kind: 'line', start, end });

test('a polyline gives one strut per segment and a node at every corner, nearly straight ones too', () => {
  const cage = v1Run('polyline'), r = cage.report;
  assert.deepStrictEqual(r.errors, []);
  assert.deepStrictEqual([r.pipeFrames, r.struts, r.nodes, r.freeEnds, r.crossings], [1, 5, 4, 2, 0]);
  assertClosedAndWound(cage, 'polyline');
  // A closed polyline: its last point meets its first, so every corner is a node. A zero-length segment is skipped.
  const sq = plan([{ kind: 'polyline', points: [[0, 0, 0], [40, 0, 0], [40, 40, 0], [40, 40, 0], [0, 40, 0], [0, 0, 0]] }], opts);
  assert.deepStrictEqual([sq.report.struts, sq.report.nodes, sq.report.freeEnds, sq.report.pipeFrames], [4, 4, 0, 1]);
  assertClosedAndWound(sq, 'closed polyline');
});

test('a frame drawn as one polyline plus loose lines is one pipe frame', () => {
  const cage = plan(v1Input(scenes.polyframe), opts), r = cage.report;
  assert.deepStrictEqual(r.errors, []);
  assert.deepStrictEqual([r.pipeFrames, r.struts, r.nodes, r.freeEnds], [1, 8, 4, 4]);
  assertClosedAndWound(cage, 'table');
});

test('duplicates: exact and reversed lines dropped and counted, as in v1', () => {
  const r = v1Run('duplicates').report;
  assert.deepStrictEqual(r.errors, []);
  assert.deepStrictEqual([r.duplicatesDropped, r.struts, r.nodes, r.freeEnds], [2, 2, 1, 2]);
  // A doubled polyline, whole or split into lines, drops every segment.
  const poly = { kind: 'polyline', points: [[0, 0, 0], [10, 0, 0], [10, 10, 0]] };
  assert.strictEqual(plan([poly, poly], v1Opts).report.duplicatesDropped, 2);
  assert.strictEqual(plan([poly, line([10, 10, 0], [10, 0, 0]), line([10, 0, 0], [0, 0, 0])], v1Opts).report.duplicatesDropped, 2);
  // A near-duplicate outside tolerance is kept.
  assert.strictEqual(plan([line([0, 0, 0], [10, 0, 0]), line([0, 0, 0], [10, 0.002, 0])], v1Opts).report.duplicatesDropped, 0);
});

test('crossing scene: one crossing warned about and left unjoined, as in v1', () => {
  const r = v1Run('crossing').report;
  assert.deepStrictEqual(r.errors, []);
  assert.strictEqual(r.crossings, 1);
  assert.deepStrictEqual(r.warnings, ['Curves cross without sharing an end near (0.000, 0.000, 0.000); left unjoined.']);
  // Unjoined: the vertical line stays its own pipe frame; the corner at (10, 0, 0) is the only node.
  assert.deepStrictEqual([r.struts, r.nodes, r.freeEnds, r.pipeFrames], [3, 1, 4, 2]);
});

test('struts that touch only at a shared node or a tip are not crossings, at any scale', () => {
  for (const name of ['y', 't', 'node6tight', 'acute', 'polyline', 'freeends', 'straight']) {
    assert.strictEqual(v1Run(name).report.crossings, 0, name);
  }
  // A tip landing on another strut's side is a touch.
  assert.strictEqual(plan([line([-10, 0, 0], [10, 0, 0]), line([0, 0, 0], [0, 10, 0])], v1Opts).report.crossings, 0);
  const tiny = { ...v1Opts, radius: 0.0005, tolerance: 1e-6 };
  assert.strictEqual(plan([line([0, 0, 0], [0.01, 0, 0]), line([0.005, -0.005, 0], [0.005, 0.005, 0])], tiny).report.crossings, 1);
});
