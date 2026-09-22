// Planner tests. Run from the repo root: node --test
const test = require('node:test');
const assert = require('node:assert');
const { plan } = require('../../../scripts/multipipe/MultiPipePlanner.js');
const scenes = require('../scenes/scenes.json');

const TOL = 0.001;
const opts = { radius: 0.5, nodeSize: 1.2, tolerance: TOL };
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
// Scene curves as planner input. An interp curve's end tangents are approximated by its end chords.
const lines = (spec) => spec.map((c) => {
  if (c.type === 'polyline') return { kind: 'polyline', points: c.pts };
  const n = c.pts.length;
  if (c.type === 'interp') {
    return { kind: 'smooth', start: c.pts[0], end: c.pts[n - 1], startTangent: sub(c.pts[1], c.pts[0]),
      endTangent: sub(c.pts[n - 1], c.pts[n - 2]), samples: c.pts };
  }
  return { kind: 'line', start: c.pts[0], end: c.pts[n - 1] };
});
// Quarter arc of radius 10 around the origin in XY, from angle a0 to a0 + 90 degrees.
const arc = (a0) => {
  const r = a0 * Math.PI / 180, q = r + Math.PI / 2, p = (t) => [10 * Math.cos(t), 10 * Math.sin(t), 0];
  return { kind: 'smooth', start: p(r), end: p(q), startTangent: [-Math.sin(r), Math.cos(r), 0],
    endTangent: [-Math.sin(q), Math.cos(q), 0], samples: [p(r), p((r + q) / 2), p(q)] };
};
const line = (start, end) => ({ kind: 'line', start, end });
// Two struts meeting at [10,0,0] with the given bend in degrees.
const bent = (deg) => {
  const r = deg * Math.PI / 180;
  return plan([line([0, 0, 0], [10, 0, 0]), line([10, 0, 0], [10 + 10 * Math.cos(r), 10 * Math.sin(r), 0])], opts);
};
const run = (name) => plan(lines(scenes[name]), opts);

test('scenes with one joint node: degree and free ends', () => {
  for (const [name, degree] of [['y', 3], ['t', 3], ['node4', 4], ['node5', 5], ['node6', 6], ['node6tight', 6], ['acute', 2]]) {
    const p = run(name);
    assert.deepStrictEqual(p.errors, [], name);
    assert.strictEqual(p.rails.length, scenes[name].length, name);
    assert.deepStrictEqual(p.joints.map((j) => j.degree), [degree], name);
    assert.strictEqual(p.freeEnds.length, degree, name);
  }
});

test('freeends scene: one corner joint, two free ends', () => {
  const p = run('freeends');
  assert.deepStrictEqual(p.joints, [{ point: [10, 0, 0], degree: 2 }]);
  assert.deepStrictEqual(p.freeEnds.map((f) => [f.railIndex, f.atStart]), [[0, true], [1, false]]);
});

test('endpoints within tolerance cluster into one node, outside do not', () => {
  const near = plan([
    { kind: 'line', start: [0, 0, 0], end: [10, 0, 0] },
    { kind: 'line', start: [10, TOL / 2, 0], end: [10, 10, 0] }
  ], opts);
  assert.strictEqual(near.joints.length, 1);
  assert.strictEqual(near.freeEnds.length, 2);
  const far = plan([
    { kind: 'line', start: [0, 0, 0], end: [10, 0, 0] },
    { kind: 'line', start: [10, TOL * 2, 0], end: [10, 10, 0] }
  ], opts);
  assert.strictEqual(far.joints.length, 0);
  assert.strictEqual(far.freeEnds.length, 4);
});

test('node size is clamped to at least 1.02', () => {
  assert.strictEqual(plan(lines(scenes.y), { ...opts, nodeSize: 0.5 }).nodeSize, 1.02);
  assert.strictEqual(plan(lines(scenes.y), { ...opts, nodeSize: 1.5 }).nodeSize, 1.5);
});

test('errors: no curves, zero-length line, bad radius', () => {
  assert.strictEqual(plan([], opts).errors.length, 1);
  const zero = plan([{ kind: 'line', start: [1, 1, 1], end: [1, 1, 1 + TOL / 2] }], opts);
  assert.match(zero.errors[0], /zero length/);
  assert.strictEqual(plan(lines(scenes.y), { ...opts, radius: 0 }).errors.length, 1);
});

test('bend at a degree-2 node: 0 and 2 degrees are straight, just above 5 is a joint', () => {
  for (const deg of [0, 2]) {
    const p = bent(deg);
    assert.strictEqual(p.straightNodes, 1, deg);
    assert.strictEqual(p.joints.length, 0, deg);
    assert.strictEqual(p.rails.length, 1, deg);
    assert.strictEqual(p.rails[0].points.length, 3, deg);
    assert.strictEqual(p.freeEnds.length, 2, deg);
  }
  const p = bent(5.1);
  assert.strictEqual(p.straightNodes, 0);
  assert.deepStrictEqual(p.joints.map((j) => j.degree), [2]);
  assert.strictEqual(p.rails.length, 2);
});

test('chain through several straight nodes is one rail, in either drawing direction', () => {
  const p = plan([line([0, 0, 0], [10, 0, 0]), line([20, 0, 0], [10, 0, 0]), line([20, 0, 0], [30, 0.1, 0]),
    line([30, 0.1, 0], [40, 0.1, 0])], opts);
  assert.strictEqual(p.straightNodes, 3);
  assert.strictEqual(p.joints.length, 0);
  assert.deepStrictEqual(p.rails, [{ points: [[0, 0, 0], [10, 0, 0], [20, 0, 0], [30, 0.1, 0], [40, 0.1, 0]],
    curves: [null, null, null, null], closed: false }]);
  assert.deepStrictEqual(p.freeEnds.map((f) => [f.railIndex, f.atStart]), [[0, true], [0, false]]);
});

test('straight scene: lines end to end run through, the corner gets a joint', () => {
  const p = run('straight');
  assert.strictEqual(p.straightNodes, 1);
  assert.deepStrictEqual(p.joints, [{ point: [20, 0, 0], degree: 2 }]);
  assert.deepStrictEqual(p.rails.map((r) => r.points.length), [3, 2]);
});

test('polyline scene: mixed corners, 2 degree corner merged, others jointed', () => {
  const p = run('polyline');
  assert.deepStrictEqual(p.errors, []);
  assert.strictEqual(p.straightNodes, 1);
  assert.deepStrictEqual(p.joints.map((j) => j.point), [[10, 0, 0], [10, 10, 0], [23.8911, 24.3849, 0]]);
  assert.deepStrictEqual(p.rails.map((r) => r.points.length), [2, 2, 3, 2]);
  assert.strictEqual(p.freeEnds.length, 2);
});

test('closed loop of straight nodes becomes one closed rail', () => {
  const pts = [];
  for (let i = 0; i <= 100; i++) pts.push([Math.cos(i * Math.PI / 50) * 10, Math.sin(i * Math.PI / 50) * 10, 0]);
  const p = plan([{ kind: 'polyline', points: pts }], opts);
  assert.strictEqual(p.rails.length, 1);
  assert.strictEqual(p.rails[0].closed, true);
  assert.strictEqual(p.freeEnds.length, 0);
  assert.strictEqual(p.joints.length, 0);
});

test('smooth curve: end tangents decide straight nodes, not the chord', () => {
  // The arc ends at [0,10,0] heading -x; a line on along -x runs straight through, although the chord bends 45 degrees.
  const p = plan([arc(0), line([0, 10, 0], [-10, 10, 0])], opts);
  assert.deepStrictEqual(p.errors, []);
  assert.strictEqual(p.straightNodes, 1);
  assert.strictEqual(p.joints.length, 0);
  assert.strictEqual(p.rails.length, 1);
  assert.deepStrictEqual(p.rails[0].curves, [0, null]);
  assert.deepStrictEqual(p.rails[0].points[2], [-10, 10, 0]);
  // A line on along +y leaves the arc's end tangent at 90 degrees: joint.
  const q = plan([arc(0), line([0, 10, 0], [0, 20, 0])], opts);
  assert.strictEqual(q.straightNodes, 0);
  assert.deepStrictEqual(q.joints.map((j) => j.degree), [2]);
});

test('mixed chain: line, arc, line through straight nodes is one rail', () => {
  const p = plan([line([10, -10, 0], [10, 0, 0]), arc(0), line([0, 10, 0], [-10, 10, 0])], opts);
  assert.strictEqual(p.straightNodes, 2);
  assert.strictEqual(p.joints.length, 0);
  assert.strictEqual(p.rails.length, 1);
  assert.deepStrictEqual(p.rails[0].curves, [null, 1, null]);
  assert.strictEqual(p.freeEnds.length, 2);
});

test('smooth scene: curved strut joins both lines at joints', () => {
  const p = run('smooth');
  assert.deepStrictEqual(p.errors, []);
  assert.deepStrictEqual(p.joints.map((j) => j.point), [[0, 0, 0], [15, 5, 6]]);
  assert.strictEqual(p.rails.length, 3);
  assert.strictEqual(p.freeEnds.length, 2);
});

test('closed smooth curve is one closed rail with no nodes or free ends, even when a line touches it', () => {
  const circle = { kind: 'smooth', start: [10, 0, 0], end: [10, 0, 0], startTangent: [0, 1, 0], endTangent: [0, 1, 0],
    samples: [[10, 0, 0], [-10, 0, 0], [10, 0, 0]] };
  const p = plan([circle], opts);
  assert.deepStrictEqual(p.errors, []);
  assert.deepStrictEqual(p.rails, [{ points: [[10, 0, 0], [10, 0, 0]], curves: [0], closed: true }]);
  assert.strictEqual(p.joints.length + p.freeEnds.length + p.straightNodes, 0);
  const q = plan([circle, line([10, 0, 0], [20, 0, 0])], opts);
  assert.strictEqual(q.rails.length, 2);
  assert.strictEqual(q.joints.length, 0);
  assert.strictEqual(q.freeEnds.length, 2);
});

test('duplicates: exact and reversed lines dropped and counted', () => {
  const p = run('duplicates');
  assert.deepStrictEqual(p.errors, []);
  assert.strictEqual(p.duplicatesDropped, 2);
  assert.strictEqual(p.rails.length, 2);
  assert.deepStrictEqual(p.joints, [{ point: [0, 0, 0], degree: 2 }]);
  // A doubled polyline, whole or split into lines as describe() delivers it, drops every segment.
  const poly = { kind: 'polyline', points: [[0, 0, 0], [10, 0, 0], [10, 10, 0]] };
  assert.strictEqual(plan([poly, poly], opts).duplicatesDropped, 2);
  assert.strictEqual(plan([poly, line([10, 10, 0], [10, 0, 0]), line([10, 0, 0], [0, 0, 0])], opts).duplicatesDropped, 2);
});

test('duplicates: a curved duplicate is dropped, a curve bowing the other way is kept', () => {
  const p = plan([arc(0), arc(0)], opts);
  assert.strictEqual(p.duplicatesDropped, 1);
  assert.strictEqual(p.rails.length, 1);
  // Same ends, but a line instead of the arc: the midpoint differs, so both stay.
  const q = plan([arc(0), line(arc(0).start, arc(0).end)], opts);
  assert.strictEqual(q.duplicatesDropped, 0);
  assert.strictEqual(q.rails.length, 2);
});

test('duplicates: an S-curve is not a duplicate of the line between its ends, nor of its mirror', () => {
  const sCurve = (sign) => ({ kind: 'smooth', start: [0, 0, 0], end: [20, 0, 0], startTangent: [1, sign, 0],
    endTangent: [1, sign, 0], samples: [[0, 0, 0], [5, 5 * sign, 0], [10, 0, 0], [15, -5 * sign, 0], [20, 0, 0]] });
  const p = plan([sCurve(1), line([0, 0, 0], [20, 0, 0])], opts);
  assert.strictEqual(p.duplicatesDropped, 0);
  assert.strictEqual(p.rails.length, 2);
  const q = plan([sCurve(1), sCurve(-1)], opts);
  assert.strictEqual(q.duplicatesDropped, 0);
  assert.strictEqual(q.rails.length, 2);
  assert.strictEqual(plan([sCurve(1), sCurve(1)], opts).duplicatesDropped, 1);
});

test('duplicates: a near-duplicate outside tolerance is kept', () => {
  const p = plan([line([0, 0, 0], [10, 0, 0]), line([0, 0, 0], [10, TOL * 2, 0])], opts);
  assert.strictEqual(p.duplicatesDropped, 0);
  assert.strictEqual(p.rails.length, 2);
});

test('crossing scene: one mid-span crossing reported with its point, not split', () => {
  const p = run('crossing');
  assert.deepStrictEqual(p.errors, []);
  assert.strictEqual(p.rails.length, 3);
  assert.strictEqual(p.crossings.length, 1);
  // The crossing rails are the horizontal and the vertical one, whatever order the walk gave them.
  const ends = [p.crossings[0].railA, p.crossings[0].railB].flatMap((i) => p.rails[i].points);
  for (const want of [[-10, 0, 0], [0, -10, 0]]) assert.ok(ends.some((q) => Math.hypot(q[0] - want[0], q[1] - want[1], q[2]) < TOL), String(want));
  assert.ok(Math.hypot(...p.crossings[0].point) < TOL);
  assert.strictEqual(p.warnings.length, 1);
  assert.deepStrictEqual(p.joints, [{ point: [10, 0, 0], degree: 2 }]);
});

test('struts that touch only at a shared node are not crossings', () => {
  for (const name of ['y', 'node6tight', 'acute', 'smooth', 'polyline', 'freeends']) {
    assert.deepStrictEqual(run(name).crossings, [], name);
  }
  assert.deepStrictEqual(plan([arc(0), line([10, 0, 0], [20, 0, 0])], opts).crossings, []);
  // A line ending on a closed ring touches it at that end: a touch, not a crossing.
  const ring = { kind: 'smooth', start: [10, 0, 0], end: [10, 0, 0], startTangent: [0, 1, 0], endTangent: [0, 1, 0],
    samples: [[10, 0, 0], [0, 10, 0], [-10, 0, 0], [0, -10, 0], [10, 0, 0]] };
  const p = plan([ring, line([0, 10, 0], [0, 20, 0])], opts);
  assert.deepStrictEqual(p.crossings, []);
  assert.deepStrictEqual(p.warnings, []);
  // A line cutting straight through the ring, touching no end of either, is a crossing.
  assert.strictEqual(plan([ring, line([0, 20, 0], [0, -20, 0])], opts).crossings.length, 2);
  // Crossing detection must not depend on scale: the same X, 1000x smaller, still crosses.
  const tiny = { ...opts, tolerance: 1e-6 };
  assert.strictEqual(plan([line([0, 0, 0], [0.01, 0, 0]), line([0.005, -0.005, 0], [0.005, 0.005, 0])], tiny).crossings.length, 1);
});

test('cap: on by default, off only when asked for', () => {
  assert.strictEqual(run('y').cap, true);
  assert.strictEqual(plan(lines(scenes.y), { ...opts, cap: true }).cap, true);
  assert.strictEqual(plan(lines(scenes.y), { ...opts, cap: false }).cap, false);
});

test('joint style and fillet factor: defaults, pass-through and validation', () => {
  const p = run('y');
  assert.strictEqual(p.jointStyle, 'ball');
  const f = plan(lines(scenes.y), { ...opts, jointStyle: 'filleted', filletFactor: 0.2 });
  assert.strictEqual(f.jointStyle, 'filleted');
  assert.strictEqual(f.filletFactor, 0.2);
  assert.deepStrictEqual(f.errors, []);
  // A fillet factor of zero or less is an error, but only for filleted joints.
  for (const factor of [0, -1, undefined]) {
    assert.deepStrictEqual(plan(lines(scenes.y), { ...opts, jointStyle: 'filleted', filletFactor: factor }).errors,
      ['Fillet factor must be greater than zero.'], String(factor));
    assert.deepStrictEqual(plan(lines(scenes.y), { ...opts, filletFactor: factor }).errors, [], String(factor));
  }
});
