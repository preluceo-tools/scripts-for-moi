// MultiPipe builder: plan in, geometry out. ES5, MoI geometry API only, never the command UI.
// Every intermediate object comes from factory calculate(); nothing is added to the document here.
//
// describe(curves)
//   curves:  MoI object list of curves
//   returns: { input, objects }: planner input, and objects[i] the one-segment MoI curve behind input[i].
//            Every segment becomes its own entry: a line when straight, a smooth curve otherwise.
//
// build(plan, curveObjects, options)
//   plan:         result of plan() without errors
//   curveObjects: describe().objects, indexed like the planner input
//   options:      { union } optional: function (objects array) -> objects array, replaces booleanunion (tests)
//   returns:      { objects: [MoI objects], report: { struts, joints, freeEnds, straightNodes, duplicatesDropped,
//                 crossings, union, fillet, errors, warnings, ms } }; report.warnings starts with the plan's warnings
//   report.union: which path gave the result: 'single' (one part, no union), 'union' (one-shot union verified),
//                 'retry' (incremental per-joint union verified), 'separate' (separate parts, with a warning)
//   report.fillet: only with plan.jointStyle 'filleted' and at least one joint: 'filleted' (the creases were
//                 rounded) or 'fallback' (the whole frame kept its ball joints, with a warning)
//   report.cap:   only with plan.cap false and at least one free end: 'open' when the cap faces were removed,
//                 absent when they could not be and the frame stayed capped, with a warning

var FILLET_FALLBACK = 'The filleted joints could not be made, so the pipe frame kept its ball joints; try a smaller fillet factor.';
var CAP_FALLBACK = 'The free ends could not be opened, so the pipe frame was left capped.';

function describe(curves) {
  var input = [], objects = [], i, j, k;
  function A(p) { return [p.x, p.y, p.z]; }
  for (i = 0; i < curves.length; i++) {
    // Segments from getSubObjects() cannot be swept, so a multi-segment curve is separated into curves.
    var c = curves.item(i), segs = [c];
    if (c.getSubObjects().length > 1) {
      var f = moi.command.createFactory('separate'), l = moi.geometryDatabase.createObjectList();
      l.addObject(c); f.setInput(0, l);
      var r = f.calculate(); f.cancel();
      if (r.length) segs = [];
      for (j = 0; j < r.length; j++) segs.push(r.item(j));
    }
    for (j = 0; j < segs.length; j++) {
      var s = segs[j], t0 = s.domainMin, t1 = s.domainMax;
      if (s.isLine) {
        // A zero-length piece inside a longer curve is dropped; a zero-length curve on its own reaches the planner's error.
        if (segs.length > 1 && s.getLength() <= moi.geometryDatabase.tolerance) continue;
        input.push({ kind: 'line', start: A(s.evaluatePoint(t0)), end: A(s.evaluatePoint(t1)) });
      } else {
        var samples = [];
        for (k = 0; k <= 8; k++) samples.push(A(s.evaluatePoint(t0 + (t1 - t0) * k / 8)));
        input.push({ kind: 'smooth', start: samples[0], end: samples[8], samples: samples,
                     startTangent: A(s.evaluateTangent(t0)), endTangent: A(s.evaluateTangent(t1)) });
      }
      objects.push(s);
    }
  }
  return { input: input, objects: objects };
}

function build(plan, curveObjects, options) {
  var VM = moi.vectorMath, t0 = new Date().getTime(), i;
  var report = { struts: plan.rails.length, joints: plan.joints.length, freeEnds: plan.freeEnds.length,
                 straightNodes: plan.straightNodes, duplicatesDropped: plan.duplicatesDropped,
                 crossings: plan.crossings.length,
                 errors: [], warnings: plan.warnings.slice(), ms: 0 };

  function P(a) { return VM.createPoint(a[0], a[1], a[2]); }
  function A(p) { return [p.x, p.y, p.z]; }
  function list(items) { var l = moi.geometryDatabase.createObjectList(); for (var j = 0; j < items.length; j++) l.addObject(items[j]); return l; }
  function calc(name, setup) { var f = moi.command.createFactory(name); setup(f); var r = f.calculate(); f.cancel(); return r; }
  function norm(a) { var l = Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); return [a[0] / l, a[1] / l, a[2] / l]; }
  function dist(a, b) { return Math.sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2])); }
  function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
  function done(objects) { report.ms = new Date().getTime() - t0; return { objects: uncap(objects), report: report }; }

  // Cap off: the frame is built and unioned capped, because open pipes do not union with the solid balls;
  // the planar cap faces at the free ends are deleted afterwards. Only one verified solid can be opened,
  // so separate pieces and the error paths keep their caps. No free ends means nothing to do.
  function uncap(objs) {
    if (plan.cap || !plan.freeEnds.length || objs.length !== 1 || !objs[0].isSolidBRep) return objs;
    var faces = objs[0].getFaces(), pick = [], j, k;
    for (j = 0; j < faces.length; j++) {
      var face = faces.item(j);
      if (!face.isPlanar) continue;
      var bb = face.getBoundingBox(), c = [(bb.min.x + bb.max.x) / 2, (bb.min.y + bb.max.y) / 2, (bb.min.z + bb.max.z) / 2];
      // A cap is a disc across the pipe, so its bounding box is centred on the free end itself.
      for (k = 0; k < plan.freeEnds.length; k++) if (dist(c, plan.freeEnds[k].point) <= plan.radius) { pick.push(face); break; }
    }
    var r = pick.length === plan.freeEnds.length ? calc('delete', function (f) { f.setInput(0, list(pick)); }) : [];
    if (r.length !== 1 || r.item(0).isSolidBRep) { report.warnings.push(CAP_FALLBACK); return objs; }
    report.cap = 'open';
    return [r.item(0)];
  }

  // One curve per rail, so a run through straight nodes sweeps as one pipe. A single segment is a
  // line: node6tight unions with a strut missing when its struts are swept along 2-point polylines.
  // A rail with smooth pieces joins its input curves and lines for the straight pieces into one curve.
  function railCurve(rail) {
    var points = rail.points, pieces = [], j, smooth = false;
    for (j = 0; j < rail.curves.length; j++) if (rail.curves[j] !== null) smooth = true;
    if (smooth) {
      for (j = 0; j < rail.curves.length; j++) {
        if (rail.curves[j] !== null) { pieces.push(curveObjects[rail.curves[j]]); continue; }
        var ln = calc('line', function (f) { f.setInput(0, P(points[j])); f.setInput(1, P(points[j + 1])); });
        if (!ln.length) return null;
        pieces.push(ln.item(0));
      }
      if (pieces.length === 1) return pieces[0];
      var joined = calc('join', function (f) { f.setInput(0, list(pieces)); });
      return joined.length === 1 ? joined.item(0) : null;
    }
    var r = points.length === 2
      ? calc('line', function (f) { f.setInput(0, P(points[0])); f.setInput(1, P(points[1])); })
      : calc('polyline', function (f) {
        for (var j = 0; j < points.length; j++) { f.createInput('point'); f.setInput(f.numInputs - 1, P(points[j])); }
      });
    return r.length ? r.item(0) : null;
  }

  var parts = [];
  for (i = 0; i < plan.rails.length; i++) {
    var curve = railCurve(plan.rails[i]);
    if (!curve) { report.errors.push('Strut ' + (i + 1) + ' could not be built.'); return done([]); }
    var o = A(curve.evaluatePoint(curve.domainMin)), t = norm(A(curve.evaluateTangent(curve.domainMin)));
    var up = Math.abs(t[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    var x = norm(cross(up, t)), y = cross(t, x);
    var circle = calc('circle', function (f) {
      f.setInput(0, true); f.setInput(1, VM.createFrame(P(o), P(x), P(y))); f.setInput(3, plan.radius);
    });
    var pipe = circle.length ? calc('sweep', function (f) {
      f.setInput(0, list([circle.item(0)])); f.setInput(1, list([curve]));
      f.setInput(4, 'none'); f.setInput(5, 'freeform'); f.setInput(7, true); f.setInput(10, 'Auto');
    }) : circle;
    if (!pipe.length || !pipe.item(0).isSolidBRep) {
      report.errors.push('Strut ' + (i + 1) + ' could not be built; try a smaller strut radius.');
      return done([]);
    }
    parts.push(pipe.item(0));
  }

  for (i = 0; i < plan.joints.length; i++) {
    var c = plan.joints[i].point;
    var ball = calc('sphere', function (f) {
      f.setInput(0, true); f.setInput(1, VM.createFrame(P(c), P([1, 0, 0]), P([0, 1, 0]))); f.setInput(3, plan.radius * plan.nodeSize);
    });
    if (!ball.length) { report.errors.push('Joint ' + (i + 1) + ' could not be built.'); return done([]); }
    parts.push(ball.item(0));
  }

  // Filleted joints: round the creases of the verified body. The edges to fillet are those whose bounding-box
  // centre lies within 1.5 x ball radius of a joint node; filtering to closed edges does not work, as boolean
  // edges are split. Success is one solid with more faces than before; any failure keeps the ball-joint body.
  function finish(objs, path) {
    report.union = path;
    if (plan.jointStyle !== 'filleted' || !plan.joints.length) return done(objs);
    var body = objs[0], edges = body.getEdges(), faces0 = body.getFaces().length, pick = [], j, k;
    var reach = plan.radius * plan.nodeSize * 1.5;
    for (j = 0; j < edges.length; j++) {
      var bb = edges.item(j).getBoundingBox();
      var c = [(bb.min.x + bb.max.x) / 2, (bb.min.y + bb.max.y) / 2, (bb.min.z + bb.max.z) / 2];
      for (k = 0; k < plan.joints.length; k++) {
        if (dist(c, plan.joints[k].point) < reach) { pick.push(edges.item(j)); break; }
      }
    }
    var fr = pick.length ? calc('fillet', function (f) {
      f.setInput(0, list(pick)); f.setInput(1, false); f.setInput(3, plan.radius * plan.filletFactor);
      f.setInput(4, 'circular'); f.setInput(5, 0.5);
    }) : [];
    if (fr.length === 1 && fr.item(0).isSolidBRep && fr.item(0).getFaces().length > faces0) {
      report.fillet = 'filleted';
      return done([fr.item(0)]);
    }
    report.fillet = 'fallback';
    report.warnings.push(FILLET_FALLBACK);
    return done(objs);
  }

  if (parts.length === 1) return finish(parts, 'single');

  // Verification: one solid whose bounding box matches the inputs' combined box. A dropped strut shrinks
  // the box. A pipe's box follows its control points and runs up to about 0.3 x radius past the trimmed
  // union's box (node4), so the match allows one strut radius.
  var slack = Math.max(moi.geometryDatabase.tolerance, plan.radius);
  function box(objs) {
    var lo = [1e30, 1e30, 1e30], hi = [-1e30, -1e30, -1e30];
    for (var j = 0; j < objs.length; j++) {
      var b = objs[j].getBoundingBox(), bl = A(b.min), bh = A(b.max);
      for (var k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], bl[k]); hi[k] = Math.max(hi[k], bh[k]); }
    }
    return lo.concat(hi);
  }
  var want = box(parts);
  function verified(r) {
    if (r.length !== 1 || !r[0].isSolidBRep) return false;
    var got = box(r);
    for (var k = 0; k < 6; k++) if (Math.abs(got[k] - want[k]) > slack) return false;
    return true;
  }
  var union = options.union || function (objs) {
    var r = calc('booleanunion', function (f) { f.setInput(0, list(objs)); }), a = [];
    for (var j = 0; j < r.length; j++) a.push(r.item(j));
    return a;
  };
  // Folds objs into one body, one union at a time; null when a step does not give exactly one object.
  function fold(objs) {
    var body = objs[0];
    for (var j = 1; j < objs.length && body; j++) { var r = union([body, objs[j]]); body = r.length === 1 ? r[0] : null; }
    return body;
  }

  var u = union(parts);
  if (verified(u)) return finish(u, 'union');

  // Retry: per joint, the ball then each strut ending at it, one union at a time; then the per-node bodies.
  // Each strut goes to the first joint it touches; a strut touching no joint is its own body.
  var nRails = plan.rails.length, taken = [], bodies = [], ok = true;
  for (i = 0; i < plan.joints.length && ok; i++) {
    var group = [parts[nRails + i]], jp = plan.joints[i].point;
    for (var s = 0; s < nRails; s++) {
      var pts = plan.rails[s].points;
      if (taken[s] || plan.rails[s].closed) continue;
      if (dist(pts[0], jp) <= moi.geometryDatabase.tolerance || dist(pts[pts.length - 1], jp) <= moi.geometryDatabase.tolerance) { taken[s] = true; group.push(parts[s]); }
    }
    var body = fold(group);
    if (body) bodies.push(body); else ok = false;
  }
  for (i = 0; i < nRails; i++) if (!taken[i]) bodies.push(parts[i]);
  if (ok) {
    u = bodies.length === 1 ? bodies : union(bodies);
    if (verified(u)) return finish(u, 'retry');
  }

  report.union = 'separate';
  report.warnings.push('The union did not give one solid, even after a retry; struts and joints were kept as separate objects.');
  // No single body to fillet, so filleted joints fall back here too, and say so.
  if (plan.jointStyle === 'filleted' && plan.joints.length) {
    report.fillet = 'fallback';
    report.warnings.push(FILLET_FALLBACK);
  }
  return done(parts);
}
