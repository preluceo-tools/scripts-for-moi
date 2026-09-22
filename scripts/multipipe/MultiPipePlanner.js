// MultiPipe planner: curve descriptions in, plan out. Pure ES5, no MoI API,
// so it runs inside MoI (via #include) and under Node (via require).
//
// plan(curves, options)
//   curves:  [ { kind: 'line', start: [x,y,z], end: [x,y,z] }
//            | { kind: 'polyline', points: [[x,y,z], ...] }
//            | { kind: 'smooth', start, end, startTangent, endTangent, samples: [[x,y,z], ...] } ]
//            A smooth curve whose ends meet is a closed ring: its own rail, no nodes, no free ends.
//            samples run along the curve; they decide duplicates (midpoint) and crossings.
//   options: { radius, nodeSize, tolerance, jointStyle: 'ball' | 'filleted', filletFactor, cap }
//            cap defaults to true: free ends are capped flat. False leaves them open.
//   returns: { rails, joints, freeEnds, straightNodes, duplicatesDropped, crossings,
//              errors, warnings, radius, nodeSize, jointStyle, filletFactor, cap }
//   rails:   [ { points: [[x,y,z], ...], curves: [index | null, ...], closed } ]  one pipe each, chained
//            through straight nodes; segment k runs points[k] to points[k+1] and is the smooth input curve
//            curves[k], or a straight piece when null. A closed smooth ring has points [start, start].
//   duplicatesDropped: segments dropped because an earlier segment has the same ends (either direction) and midpoint.
//            Counted per segment: a doubled 3-segment polyline counts 3, as describe() splits it that way anyway.
//   crossings: [ { railA, railB, point } ]  rails that come within tolerance away from a shared end; each also
//            gets a warning. Never split.

var STRAIGHT_COS = Math.cos(5 * Math.PI / 180); // straight node: bend under 5 degrees, fixed

function plan(curves, options) {
  var tol = options.tolerance, i, j, k;
  var out = {
    rails: [], joints: [], freeEnds: [], straightNodes: 0, duplicatesDropped: 0, crossings: [],
    errors: [], warnings: [],
    radius: options.radius,
    nodeSize: options.nodeSize > 1.02 ? options.nodeSize : 1.02,
    jointStyle: options.jointStyle === 'filleted' ? 'filleted' : 'ball',
    filletFactor: options.filletFactor,
    cap: options.cap !== false
  };
  if (!(options.radius > 0)) out.errors.push('Strut radius must be greater than zero.');
  if (out.jointStyle === 'filleted' && !(out.filletFactor > 0)) out.errors.push('Fillet factor must be greater than zero.');
  if (!curves || !curves.length) { out.errors.push('Select at least one curve.'); return out; }

  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function dist(a, b) { var d = sub(a, b); return Math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]); }
  function unit(d) { var l = Math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]); return [d[0] / l, d[1] / l, d[2] / l]; }
  function dir(from, to) { return unit(sub(to, from)); }

  // Explode every curve into segments, dropping zero-length ones inside polylines and exact duplicates.
  // ponytail: O(n^2) duplicate scan, same ceiling as the clustering below.
  var segs = [], kept = [];
  // Quarter, middle and three-quarter points along the samples. The middle alone would call an S-curve and a
  // straight line between the same ends the same curve, since its middle sample sits on the chord centre.
  function at(ss, fraction) {
    var x = fraction * (ss.length - 1), lo = Math.floor(x), u = x - lo, p = ss[lo], q = ss[Math.min(lo + 1, ss.length - 1)];
    return [p[0] + (q[0] - p[0]) * u, p[1] + (q[1] - p[1]) * u, p[2] + (q[2] - p[2]) * u];
  }
  function shape(ss) { return [at(ss, 0.25), at(ss, 0.5), at(ss, 0.75)]; }
  function isDuplicate(a, b, samples) {
    var m = shape(samples);
    for (var d = 0; d < kept.length; d++) {
      var q = kept[d], forward = dist(q.a, a) <= tol && dist(q.b, b) <= tol, back = dist(q.a, b) <= tol && dist(q.b, a) <= tol;
      if (!forward && !back) continue;
      if (dist(q.m[1], m[1]) > tol) continue;
      if (forward && dist(q.m[0], m[0]) <= tol && dist(q.m[2], m[2]) <= tol ||
          back && dist(q.m[0], m[2]) <= tol && dist(q.m[2], m[0]) <= tol) {
        out.duplicatesDropped++;
        return true;
      }
    }
    kept.push({ a: a, b: b, m: m });
    return false;
  }
  for (i = 0; i < curves.length; i++) {
    var c = curves[i], pts = c.kind === 'polyline' ? c.points : [c.start, c.end], nonZero = false;
    if (c.kind === 'smooth') {
      if (isDuplicate(c.start, c.end, c.samples)) continue;
      if (dist(c.start, c.end) <= tol) out.rails.push({ points: [c.start, c.start], curves: [i], closed: true });
      else segs.push({ a: c.start, b: c.end, ta: unit(c.startTangent), tb: unit(c.endTangent), curve: i, used: false });
      continue;
    }
    for (j = 0; j + 1 < pts.length; j++) {
      if (dist(pts[j], pts[j + 1]) <= tol) continue;
      nonZero = true;
      if (!isDuplicate(pts[j], pts[j + 1], [pts[j], pts[j + 1]])) segs.push({ a: pts[j], b: pts[j + 1], curve: null, used: false });
    }
    if (!nonZero) out.errors.push('Curve ' + (i + 1) + ' has zero length.');
  }

  // ponytail: O(n^2) endpoint clustering, add a spatial hash when large frames need it (ticket 08).
  var nodes = [];
  function addEnd(s, atStart) {
    var p = atStart ? s.a : s.b, end = { seg: s, atStart: atStart };
    if (s.ta) end.dir = atStart ? s.ta : [-s.tb[0], -s.tb[1], -s.tb[2]];
    else end.dir = atStart ? dir(s.a, s.b) : dir(s.b, s.a);
    for (k = 0; k < nodes.length; k++) {
      if (dist(nodes[k].point, p) <= tol) { nodes[k].ends.push(end); break; }
    }
    if (k === nodes.length) nodes.push({ point: p, ends: [end] });
    if (atStart) s.na = nodes[k]; else s.nb = nodes[k];
  }
  for (i = 0; i < segs.length; i++) { addEnd(segs[i], true); addEnd(segs[i], false); }

  for (k = 0; k < nodes.length; k++) {
    var n = nodes[k], e = n.ends;
    // Away-from-node directions of a straight run point opposite ways, so their dot is near -1.
    n.straight = e.length === 2 && e[0].seg !== e[1].seg &&
      -(e[0].dir[0] * e[1].dir[0] + e[0].dir[1] * e[1].dir[1] + e[0].dir[2] * e[1].dir[2]) > STRAIGHT_COS;
    if (n.straight) out.straightNodes++;
    else if (e.length > 1) out.joints.push({ point: n.point, degree: e.length });
  }

  // Walk from a segment end at node, through straight nodes, into one rail.
  function walk(seg, atStart) {
    var points = [atStart ? seg.a : seg.b], used = [], node;
    while (true) {
      seg.used = true;
      points.push(atStart ? seg.b : seg.a);
      used.push(seg.curve);
      node = atStart ? seg.nb : seg.na;
      if (!node.straight) return { points: points, curves: used, closed: false, endNode: node };
      var next = node.ends[0].seg === seg ? node.ends[1] : node.ends[0];
      if (next.seg.used) return { points: points, curves: used, closed: true, endNode: node };
      seg = next.seg; atStart = next.atStart;
    }
  }
  function addRail(r, startNode) {
    out.rails.push({ points: r.points, curves: r.curves, closed: r.closed });
    var idx = out.rails.length - 1;
    if (r.closed) return;
    if (startNode.ends.length === 1) out.freeEnds.push({ point: startNode.point, railIndex: idx, atStart: true });
    if (r.endNode.ends.length === 1) out.freeEnds.push({ point: r.endNode.point, railIndex: idx, atStart: false });
  }
  for (k = 0; k < nodes.length; k++) {
    if (nodes[k].straight) continue;
    for (j = 0; j < nodes[k].ends.length; j++) {
      var end = nodes[k].ends[j];
      if (!end.seg.used) addRail(walk(end.seg, end.atStart), nodes[k]);
    }
  }
  // Whatever is left is a loop made only of straight nodes.
  for (i = 0; i < segs.length; i++) if (!segs[i].used) addRail(walk(segs[i], true), segs[i].na);

  // Crossings: each rail as polyline pieces (straight pieces, or a smooth curve's samples); pieces of two rails
  // closer than tolerance, away from an end the rails share, are a crossing.
  // ponytail: O(n^2) rail pairs and chord samples, so a smooth curve whose chords miss by more than tolerance goes unseen.
  function pieces(r) {
    var p = [];
    for (var q = 0; q < r.curves.length; q++) {
      var ss = r.curves[q] === null ? [r.points[q], r.points[q + 1]] : curves[r.curves[q]].samples;
      for (var t = 0; t + 1 < ss.length; t++) p.push([ss[t], ss[t + 1]]);
    }
    return p;
  }
  function add(a, b, s) { return [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function clamp(x) { return x < 0 ? 0 : x > 1 ? 1 : x; }
  // Closest points of segments p0-p1 and q0-q1 (Ericson, Real-Time Collision Detection 5.1.9).
  function closest(p0, p1, q0, q1) {
    var d1 = sub(p1, p0), d2 = sub(q1, q0), r = sub(p0, q0), a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    var cc = dot(d1, r), b = dot(d1, d2), den = a * e - b * b, sp = den > 1e-12 * a * e ? clamp((b * f - cc * e) / den) : 0;
    var tq = e > 0 ? (b * sp + f) / e : 0;
    if (e <= 0) { tq = 0; sp = a > 0 ? clamp(-cc / a) : 0; }
    else if (a > 0 && tq < 0) { tq = 0; sp = clamp(-cc / a); }
    else if (a > 0 && tq > 1) { tq = 1; sp = clamp((b - cc) / a); }
    return [add(p0, d1, sp), add(q0, d2, clamp(tq))];
  }
  var railPieces = [];
  for (i = 0; i < out.rails.length; i++) railPieces.push(pieces(out.rails[i]));
  for (i = 0; i < out.rails.length; i++) {
    for (j = i + 1; j < out.rails.length; j++) {
      // A rail's own ends are nodes: struts meeting at one, and a strut whose tip lands on another rail, are
      // touches, not crossings. Only rails that pass through each other away from every end are reported.
      var ri = out.rails[i].points, rj = out.rails[j].points, found = [];
      var ends = [ri[0], ri[ri.length - 1], rj[0], rj[rj.length - 1]];
      for (var pa = 0; pa < railPieces[i].length; pa++) {
        for (var pb = 0; pb < railPieces[j].length; pb++) {
          var A = railPieces[i][pa], B = railPieces[j][pb], cp = closest(A[0], A[1], B[0], B[1]);
          if (dist(cp[0], cp[1]) > tol) continue;
          var pt = add(cp[0], sub(cp[1], cp[0]), 0.5), skip = false;
          for (k = 0; k < ends.length; k++) if (dist(ends[k], pt) <= 2 * tol) skip = true;
          for (k = 0; k < found.length; k++) if (dist(found[k], pt) <= 2 * tol) skip = true;
          if (skip) continue;
          found.push(pt);
          out.crossings.push({ railA: i, railB: j, point: pt });
          out.warnings.push('Curves cross without sharing an end near (' + pt[0].toFixed(3) + ', ' + pt[1].toFixed(3) +
            ', ' + pt[2].toFixed(3) + '); left unjoined.');
        }
      }
    }
  }
  return out;
}

if (typeof module !== 'undefined') module.exports = { plan: plan };
