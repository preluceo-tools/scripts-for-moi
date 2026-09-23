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
  assert.match(plan([{ kind: 'curve' }], opts).report.errors[0], /not a line, polyline or smooth curve/);
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

test('divisions: N extra rings on every strut, evenly spaced between its end rings, after any free-end ring', () => {
  // Rings per strut: two end rings, one more at each free end, plus N. Joints add no vertices, so rings = vertices / 4.
  for (const n of [0, 1, 3]) {
    for (const name of ['line', 'straight', 'bend90', 'cubeframe', 'twobends', 'polyframe']) {
      const spec = scenes[name], cage = plan(v1Input(spec), { ...opts, divisions: n }), r = cage.report;
      assert.deepStrictEqual(r.errors, [], name);
      const ends = spec.flatMap((c) => c.pts.slice(1).flatMap((p, i) => [c.pts[i], p]));
      const free = ends.filter((p) => ends.filter((q) => Math.hypot(...sub(p, q)) < 1e-9).length === 1).length;
      assert.strictEqual(cage.vertices.length / 4, r.struts * (2 + n) + free, name + ' N=' + n);
      assertClosedAndWound(cage, name + ' N=' + n);
    }
  }
  const xs = (cage) => [...new Set(cage.vertices.map((v) => +v[0].toFixed(9)))].sort((a, b) => a - b);
  // A lone strut 0-30: free-end rings at R and 30 - R, three rings between them 6.5 apart.
  assert.deepStrictEqual(xs(run('line', { divisions: 3 })), [0, R, 8.5, 15, 21.5, 30 - R, 30].map((x) => +x.toFixed(9)));
  // Two struts through a node at 30: one ring halfway between the free-end ring and the node's ring.
  const d = 1.6 * R;
  assert.deepStrictEqual(xs(run('straight', { divisions: 1 })),
    [0, R, (R + 30 - d) / 2, 30 - d, 30 + d, (30 + d + 60 - R) / 2, 60 - R, 60].map((x) => +x.toFixed(9)));
});

test('divisions Auto: straight struts get the same cage as N = 0', () => {
  for (const name of ['line', 'bend90', 'cubeframe', 'roofTruss']) {
    assert.deepStrictEqual(run(name, { divisions: 'auto' }), run(name, { divisions: 0 }), name);
    assert.deepStrictEqual(run(name), run(name, { divisions: 0 }), name);
  }
});

test('round joints off by default: same cage as ticket 07 baseline', () => {
  for (const name of ['roofTruss', 'cubeframe']) {
    assert.deepStrictEqual(run(name), run(name, { roundJoints: false, allNodes: false }), name);
  }
});

test('round joints: collar only at grown nodes unless allNodes', () => {
  // A grown (30-degree) node far from an ungrown (90-degree) one, so both are multi-strut but only one qualifies.
  const hp = [{ kind: 'line', start: [0, 0, 0], end: [30, 0, 0] }, { kind: 'line', start: [0, 0, 0], end: [25.980762, 15, 0] }];
  const bend = [{ kind: 'line', start: [1000, 0, 0], end: [1030, 0, 0] }, { kind: 'line', start: [1000, 0, 0], end: [1000, 30, 0] }];
  const curves = [...hp, ...bend];
  const base = plan(curves, opts);
  assert.strictEqual(base.report.grownNodes, 1);
  const rj = plan(curves, { ...opts, roundJoints: true });
  const rjAll = plan(curves, { ...opts, roundJoints: true, allNodes: true });
  assert.deepStrictEqual([rj.report.errors, rjAll.report.errors], [[], []]);
  assertClosedAndWound(rj, 'roundJoints grown only');
  assertClosedAndWound(rjAll, 'roundJoints allNodes');
  // The grown node has degree 2: one collar ring per strut = 2 rings = 8 vertices with roundJoints alone.
  assert.strictEqual(rj.vertices.length - base.vertices.length, 2 * 4);
  // Both nodes are degree 2: 4 rings = 16 vertices with allNodes too.
  assert.strictEqual(rjAll.vertices.length - base.vertices.length, 4 * 4);
});

test('round joints: allNodes adds a collar at every multi-strut node even when none grew', () => {
  const base = run('cubeframe');
  assert.strictEqual(base.report.grownNodes, 0);
  const rj = run('cubeframe', { roundJoints: true });
  assert.strictEqual(rj.vertices.length, base.vertices.length);
  const all = run('cubeframe', { roundJoints: true, allNodes: true });
  assert.deepStrictEqual(all.report.errors, []);
  // Every strut end at a multi-strut node gains a collar: sum of node degree = 2 x struts - free ends.
  assert.strictEqual(all.vertices.length - base.vertices.length, 4 * (2 * base.report.struts - base.report.freeEnds));
  assertClosedAndWound(all, 'cubeframe allNodes');
});

test('round joints: collar vertices stay within the ring bound, roofTruss stays closed with both options on', () => {
  const cage = run('roofTruss', { roundJoints: true, allNodes: true });
  assert.deepStrictEqual(cage.report.errors, []);
  assertClosedAndWound(cage, 'roofTruss round joints');
  for (const v of cage.vertices) assert.ok(toCurves(v, scenes.roofTruss) <= W * Math.SQRT2 + 1e-9, 'vertex far from the curves');
});

test('round joints: a strut too short for its collar skips it silently, no crash', () => {
  const p = plan([{ kind: 'line', start: [0, 0, 0], end: [30, 0, 0] },
    { kind: 'line', start: [0, 0, 0], end: [8 * Math.cos(Math.PI / 6), 8 * Math.sin(Math.PI / 6), 0] }],
    { ...opts, roundJoints: true, allNodes: true });
  assert.deepStrictEqual(p.report.errors, []);
  assert.strictEqual(p.report.shortStruts, 1);
  assertClosedAndWound(p, 'short strut round joints');
});

test('round joints: collar offset scales with growth, not a flat w (ticket 10)', () => {
  // roofTruss's bottom-left corner in isolation: same two directions, same acute-angle growth as the scene,
  // reproducing the collar-overshoot bug's node on its own so the reach and collar offset are unambiguous.
  const corner = [{ kind: 'line', start: [0, 0, 0], end: [40, 0, 0] }, { kind: 'line', start: [0, 0, 0], end: [20, 0, 25] }];
  const base = plan(corner, opts);
  assert.strictEqual(base.report.grownNodes, 1);
  const reach = base.report.largestReach * R, d0 = 1.6 * R;
  assert.ok(reach - d0 > W, 'fixture must grow well past nodeSize + w or this test proves nothing');
  const cage = plan(corner, { ...opts, roundJoints: true });
  assert.strictEqual(cage.report.shortStruts, 0);
  const d = (c) => Math.hypot(...sub(c, [0, 0, 0]));
  const rings = ringsOf(cage).map((g) => d(g.c)).sort((a, b) => a - b);
  // Innermost ring on each strut is the joint ring, at `reach`; the collar ring sits `max(w, reach - d0)`
  // further inboard, not a flat `w` further inboard (the old, overshooting behaviour).
  const jointRing = rings.filter((x) => Math.abs(x - reach) < 1e-6);
  const collarRing = rings.filter((x) => Math.abs(x - (reach + Math.max(W, reach - d0))) < 1e-6);
  const flatW = rings.filter((x) => Math.abs(x - (reach + W)) < 1e-6);
  assert.strictEqual(jointRing.length, 2, 'one joint ring per strut, at reach');
  assert.strictEqual(collarRing.length, 2, 'one collar ring per strut, at reach + max(w, reach - d0)');
  assert.strictEqual(flatW.length, 0, 'collar must not sag at the old flat-w offset once the node has grown past it');
});

test('divisions must be a whole number of 0 or more', () => {
  for (const bad of [-1, 1.5, NaN, Infinity, '3', null]) {
    assert.match(run('line', { divisions: bad }).report.errors[0], /Divisions must be a whole number of 0 or more/, String(bad));
  }
});

// Smooth curves, sampled densely by arc length as the command does. An arc in the XY plane about c, radius r, from
// angle a0 to a1 (radians); a1 - a0 of 2 pi gives a closed ring.
const unit3 = (a) => { const l = Math.hypot(...a); return a.map((x) => x / l); };
function arc(c, r, a0, a1, n = 64) {
  const at = (a) => [c[0] + r * Math.cos(a), c[1] + r * Math.sin(a), c[2]];
  const tan = (a) => [-Math.sin(a) * Math.sign(a1 - a0), Math.cos(a) * Math.sign(a1 - a0), 0];
  const samples = [];
  for (let k = 0; k <= n; k++) samples.push(at(a0 + (a1 - a0) * k / n));
  return { kind: 'smooth', samples, startTangent: tan(a0), endTangent: tan(a1) };
}
const smooth = (n, f) => ({ kind: 'smooth', samples: Array.from({ length: n + 1 }, (_, k) => f(k / n)) });
// The rings of a cage in build order: centre, frame vector r (ring corner 0 minus corner 1), and tangent.
function ringsOf(cage) {
  const out = [];
  for (let i = 0; i + 3 < cage.vertices.length; i += 4) {
    const q = cage.vertices.slice(i, i + 4), c = [0, 1, 2].map((k) => (q[0][k] + q[1][k] + q[2][k] + q[3][k]) / 4);
    const r = unit3(sub(q[0], q[1])), e2 = unit3(sub(q[0], q[3]));
    out.push({ c, r, t: cross(r, e2) });
  }
  return out;
}
// Twist between two rings: r carried by the smallest rotation taking t to the next t, then its angle to the next r.
function twist(a, b) {
  const axis = cross(a.t, b.t), s = Math.hypot(...axis), cs = dot(a.t, b.t);
  let r = a.r;
  if (s > 1e-12) {
    const k = axis.map((x) => x / s), kr = cross(k, r), kd = dot(k, r);
    r = r.map((x, i) => x * cs + kr[i] * s + k[i] * kd * (1 - cs));
  }
  return Math.atan2(dot(cross(r, b.r), b.t), dot(r, b.r));
}
const deg = (x) => x * 180 / Math.PI;

test('an arc strut: rings on the curve, perpendicular to it, and no twist from ring to ring', () => {
  const cage = plan([arc([0, 0, 0], 20, 0, Math.PI / 2)], opts), r = cage.report, rs = ringsOf(cage);
  assert.deepStrictEqual(r.errors, []);
  assert.deepStrictEqual([r.pipeFrames, r.struts, r.nodes, r.freeEnds], [1, 1, 0, 2]);
  assertClosedAndWound(cage, 'arc');
  for (const g of rs) {
    assert.ok(Math.abs(Math.hypot(g.c[0], g.c[1]) - 20) < 0.01 * R && Math.abs(g.c[2]) < 1e-9, 'ring centre off the arc');
    assert.ok(Math.abs(dot(g.t, unit3([-g.c[1], g.c[0], 0]))) > 0.9999, 'ring not perpendicular to the arc');
  }
  for (const v of cage.vertices) assert.ok(Math.abs(Math.hypot(Math.hypot(v[0], v[1]) - 20, v[2]) - W * Math.SQRT2) < 0.01 * R);
  for (let i = 0; i + 1 < rs.length; i++) assert.ok(Math.abs(deg(twist(rs[i], rs[i + 1]))) < 0.1, 'twist ' + deg(twist(rs[i], rs[i + 1])));
  // A 3D curve too: a helix turn. The carried frame twists by no more than a hair between rings.
  const hr = ringsOf(plan([smooth(256, (u) => [20 * Math.cos(2 * Math.PI * u), 20 * Math.sin(2 * Math.PI * u), 32 * u])], opts));
  for (let i = 0; i + 1 < hr.length; i++) assert.ok(Math.abs(deg(twist(hr[i], hr[i + 1]))) < 0.5, 'helix twist ' + deg(twist(hr[i], hr[i + 1])));
});

test('divisions Auto: a quarter arc gets more rings than a straight strut, the turn between rings within 15 degrees', () => {
  const quarter = plan([arc([0, 0, 0], 20, 0, Math.PI / 2)], opts), straight = plan([line([0, 0, 0], [10 * Math.PI, 0, 0])], opts);
  const rs = ringsOf(quarter);
  assert.ok(rs.length > ringsOf(straight).length, rs.length + ' rings');
  for (let i = 0; i + 1 < rs.length; i++) assert.ok(deg(Math.acos(Math.min(1, dot(rs[i].t, rs[i + 1].t)))) <= 15 + 1e-6);
  // A manual N counts as on a straight strut: two end rings, one more at each free end, plus N.
  for (const n of [0, 2, 7]) assert.strictEqual(ringsOf(plan([arc([0, 0, 0], 20, 0, Math.PI / 2)], { ...opts, divisions: n })).length, 4 + n);
});

test('a closed smooth curve is a closed tube: no caps, no nodes, its last ring meeting its first untwisted', () => {
  const ring = plan([arc([0, 0, 0], 10, 0, 2 * Math.PI)], opts), r = ring.report;
  assert.deepStrictEqual(r.errors, []);
  assert.deepStrictEqual([r.pipeFrames, r.struts, r.nodes, r.freeEnds], [1, 1, 0, 0]);
  assertClosedAndWound(ring, 'ring');
  assert.strictEqual(ringsOf(ring).length, 24);
  assert.strictEqual(ringsOf(plan([arc([0, 0, 0], 10, 0, 2 * Math.PI)], { ...opts, divisions: 5 })).length, 6);
  assert.strictEqual(ringsOf(plan([arc([0, 0, 0], 10, 0, 2 * Math.PI)], { ...opts, divisions: 0 })).length, 3);
  // A saddle loop: the frame carried round comes back turned, and that turn is spread evenly along the loop.
  const cage = plan([smooth(256, (u) => [20 * Math.cos(2 * Math.PI * u), 20 * Math.sin(2 * Math.PI * u), 8 * Math.cos(4 * Math.PI * u)])], opts);
  const rs = ringsOf(cage);
  assert.deepStrictEqual([cage.report.nodes, cage.report.freeEnds, cage.report.pipeFrames], [0, 0, 1]);
  assertClosedAndWound(cage, 'saddle');
  const steps = rs.map((g, i) => twist(g, rs[(i + 1) % rs.length]));
  for (const s of steps) assert.ok(Math.abs(deg(s)) < 3, 'twist ' + deg(s));
  // Something else at the seam makes it a node like any other.
  const tied = plan([arc([0, 0, 0], 10, 0, 2 * Math.PI), line([10, 0, 0], [30, 0, 0])], opts);
  assert.deepStrictEqual([tied.report.nodes, tied.report.freeEnds, tied.report.pipeFrames], [1, 1, 1]);
  assertClosedAndWound(tied, 'ring with a tail');
});

test('smooth curves meeting lines and polylines at nodes give closed cages', () => {
  // A D: a half circle closed by its diameter, at N = 0, 3 and Auto.
  for (const n of ['auto', 0, 3]) {
    const d = plan([arc([0, 0, 0], 20, 0, Math.PI), line([-20, 0, 0], [20, 0, 0])], { ...opts, divisions: n });
    assert.deepStrictEqual([d.report.errors.length, d.report.nodes, d.report.freeEnds, d.report.pipeFrames], [0, 2, 0, 1]);
    assertClosedAndWound(d, 'D N=' + n);
  }
  // A square frame: an arc bowing out of one side, a polyline round the other three, and a 3D half circle standing
  // on the far side.
  const frame = plan([arc([20, 0, 0], 20, Math.PI, 2 * Math.PI), { kind: 'polyline', points: [[40, 0, 0], [40, 40, 0], [0, 40, 0], [0, 0, 0]] },
    smooth(64, (u) => [20 + 20 * Math.cos(Math.PI * u), 40, 20 * Math.sin(Math.PI * u)])], opts);
  assert.deepStrictEqual(frame.report.errors, []);
  assert.deepStrictEqual([frame.report.nodes, frame.report.freeEnds, frame.report.pipeFrames], [4, 0, 1]);
  assertClosedAndWound(frame, 'frame');
});

test('smooth duplicates and crossings behave as lines do', () => {
  const a = arc([0, 0, 0], 10, 0, Math.PI), back = { kind: 'smooth', samples: a.samples.slice().reverse() };
  const dup = plan([a, back, a, arc([0, 0, 0], 10, 0, -Math.PI)], v1Opts).report;
  assert.deepStrictEqual([dup.duplicatesDropped, dup.struts], [2, 2]);
  // Two half circles crossing once, an arc crossed by a line, a ring crossed twice by a line through it.
  const two = plan([arc([0, 0, 0], 10, 0, Math.PI), arc([10, 0, 0], 10, 0, Math.PI)], v1Opts).report;
  assert.deepStrictEqual([two.crossings, two.pipeFrames], [1, 2]);
  assert.match(two.warnings[0], /near \(5\.000, 8\.6\d\d, 0\.000\)/);
  assert.strictEqual(plan([arc([0, 0, 0], 10, 0, Math.PI), line([0, -5, 0], [0, 20, 0])], v1Opts).report.crossings, 1);
  assert.strictEqual(plan([arc([0, 0, 0], 10, 0, 2 * Math.PI), line([-20, 1, 0], [20, 1, 0])], v1Opts).report.crossings, 2);
  // Meeting at a node, or a tip landing on the curve, is not a crossing.
  assert.strictEqual(plan([arc([0, 0, 0], 10, 0, Math.PI), line([10, 0, 0], [10, -10, 0]), line([0, 10, 0], [0, 20, 0])], v1Opts).report.crossings, 0);
});
