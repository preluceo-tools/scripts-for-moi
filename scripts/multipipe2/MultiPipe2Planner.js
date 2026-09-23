// MultiPipe2 planner: curve descriptions in, cage out. Pure ES5, no MoI API,
// so it runs inside MoI (via #include) and under Node (via require).
//
// plan(curves, options)
//   curves:  [ { kind: 'line', start: [x,y,z], end: [x,y,z] } | { kind: 'polyline', points: [[x,y,z], ...] }
//            | { kind: 'smooth', samples: [[x,y,z], ...], startTangent, endTangent } ]
//            any other kind is an error. A polyline gives one strut per segment, a node at every corner;
//            zero-length segments inside it are skipped. A smooth curve is one strut through its samples (dense, by
//            arc length; the tangents are optional and default to the end chords). A smooth curve whose ends meet
//            is a closed ring: with nothing else at its seam it is a closed tube with no node.
//   options: { radius, nodeSize, divisions, cap, tolerance, roundJoints, allNodes }  nodeSize >= 1.0; cap defaults
//            to true; divisions 'auto' (the default: straight struts get no extra rings, curved struts the fewest
//            evenly spaced rings that keep the curve's turn between consecutive rings within TURN) or a whole
//            number N >= 0, the extra rings on every strut, spaced evenly between its end rings (after any
//            free-end ring); a closed tube gets N + 1 rings, at least 3. roundJoints (default false): every node
//            that grew (report.grownNodes) gets one extra plain ring per strut, at least w further inboard than
//            the joint ring (more, scaling with how far the node grew past nodeSize x radius, so a heavily grown
//            node gets a gentler taper), to hold the SubD limit surface round at a pinched joint without
//            overshooting past it; skipped at a strut too short to fit it. allNodes (default false, no effect
//            unless roundJoints is true): every multi-strut node gets the collar, not just grown ones.
//   returns: { vertices: [[x,y,z], ...], faces: [[i, j, k, l], ...], box: [minX, minY, minZ, maxX, maxY, maxZ],
//              report: { pipeFrames, struts, nodes, freeEnds, duplicatesDropped, crossings, grownNodes, largestReach,
//                        shortStruts, errors, warnings } }
//   Endpoints within tolerance are one node.
//   duplicatesDropped: segments dropped because an earlier one has the same ends (either direction) and shape.
//   crossings: strut pairs that pass within tolerance of each other away from their ends; left unjoined, one
//              warning each.
//   largestReach: the largest ring offset at a grown node, as a factor of radius (0 when none grew).
//   shortStruts: struts shorter than the ring offsets at their two ends (still built).
//   The cage: a square ring (half-width radius / 0.93) at the node's ring offset (along the curve) from every node
//   on each strut, framed by a rotation-minimising frame carried along the curve from the start's reference rule,
//   the convex hull of a node's rings (ring facets removed) as its joint, quad tubes between rings, and at a
//   free end an end ring, an extra ring 1 x radius in, and a cap face when cap is on. Faces wind outward.
//
// objLines(cage)  -> OBJ lines, Y-up as the SubD import expects: (x, z, -y). No groups, no materials.
// checkImport(cageBox, boxes, tolerance) -> '' when the imported boxes lie inside the cage box and cover at
//   least 80% of it on every axis, else the message for the user. boxes: [[minX, ..., maxZ], ...].

var WIDTH = 1 / 0.93; // a 4-ring's limit radius is 0.917 w to 0.943 w, so w = radius / 0.93 centres it on radius
var TURN = 15 * Math.PI / 180; // Auto: the most a curve turns between consecutive rings

function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function mul(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function len(a) { return Math.sqrt(dot(a, a)); }
function unit(a) { return mul(a, 1 / len(a)); }
function clamp(x) { return x < 0 ? 0 : x > 1 ? 1 : x; }
function angle(a, b) { return Math.atan2(len(cross(a, b)), dot(a, b)); }

// One double-reflection step of a rotation-minimising frame (Wang et al. 2008): the frame vector r at point x with
// tangent t, carried to point x1 with tangent t1.
function carry(x, t, r, x1, t1) {
  var v = sub(x1, x), c = dot(v, v);
  if (c > 1e-24) { r = sub(r, mul(v, 2 * dot(v, r) / c)); t = sub(t, mul(v, 2 * dot(v, t) / c)); }
  v = sub(t1, t); c = dot(v, v);
  if (c > 1e-24) r = sub(r, mul(v, 2 * dot(v, r) / c));
  return unit(sub(r, mul(t1, dot(t1, r))));
}

// A strut's centreline from its points: unit tangents (central differences, or the given end tangents),
// cumulative length s, cumulative turn tv, and the rotation-minimising frame vector r at every point, starting from
// the reference rule (world Z, or X when the start is near-vertical).
function track(pts, t0, t1) {
  var p = [pts[0]], T = [], s = [0], tv = [0], r, i;
  for (i = 1; i < pts.length; i++) if (len(sub(pts[i], p[p.length - 1])) > 1e-12) { s.push(s[p.length - 1] + len(sub(pts[i], p[p.length - 1]))); p.push(pts[i]); }
  var n = p.length;
  for (i = 0; i < n; i++) T.push(n < 2 ? [1, 0, 0] : unit(sub(p[Math.min(i + 1, n - 1)], p[Math.max(i - 1, 0)])));
  if (t0 && len(t0) > 0) T[0] = unit(t0);
  if (t1 && len(t1) > 0) T[n - 1] = unit(t1);
  r = [unit(cross(Math.abs(T[0][2]) > 0.9 ? [1, 0, 0] : [0, 0, 1], T[0]))];
  for (i = 1; i < n; i++) { tv.push(tv[i - 1] + angle(T[i - 1], T[i])); r.push(carry(p[i - 1], T[i - 1], r[i - 1], p[i], T[i])); }
  return { p: p, t: T, s: s, tv: tv, r: r, L: s[n - 1] };
}

// The segment of a track holding arc length q, and how far along it.
function locate(tr, q) {
  var lo = 0, hi = tr.s.length - 1;
  while (hi - lo > 1) { var m = (lo + hi) >> 1; if (tr.s[m] <= q) lo = m; else hi = m; }
  return { i: lo, u: hi > lo ? clamp((q - tr.s[lo]) / (tr.s[hi] - tr.s[lo])) : 0, j: hi };
}
function turnAt(tr, q) { var k = locate(tr, q); return tr.tv[k.i] + (tr.tv[k.j] - tr.tv[k.i]) * k.u; }
// Centre, tangent and frame vector at arc length q.
function frameAt(tr, q) {
  var k = locate(tr, q), c = add(tr.p[k.i], mul(sub(tr.p[k.j], tr.p[k.i]), k.u));
  var t = len(sub(tr.t[k.j], tr.t[k.i])) < 1e-12 ? tr.t[k.i] : unit(add(tr.t[k.i], mul(sub(tr.t[k.j], tr.t[k.i]), k.u)));
  return { c: c, t: t, r: carry(tr.p[k.i], tr.t[k.i], tr.r[k.i], c, t) };
}
// Fewest even spans (at least min) of lo..hi whose turn each stays within TURN.
function spans(tr, lo, hi, min) {
  for (var n = min; n < 1000; n++) {
    var ok = true;
    for (var j = 0; j < n && ok; j++) ok = turnAt(tr, lo + (hi - lo) * (j + 1) / n) - turnAt(tr, lo + (hi - lo) * j / n) <= TURN + 1e-6;
    if (ok) return n;
  }
  return n;
}

// Closest points of segments p0-p1 and q0-q1 (Ericson, Real-Time Collision Detection 5.1.9).
function closest(p0, p1, q0, q1) {
  var d1 = sub(p1, p0), d2 = sub(q1, q0), r = sub(p0, q0), a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  var cc = dot(d1, r), b = dot(d1, d2), den = a * e - b * b, sp = den > 1e-12 * a * e ? clamp((b * f - cc * e) / den) : 0;
  var tq = e > 0 ? (b * sp + f) / e : 0;
  if (e <= 0) { tq = 0; sp = a > 0 ? clamp(-cc / a) : 0; }
  else if (a > 0 && tq < 0) { tq = 0; sp = clamp(-cc / a); }
  else if (a > 0 && tq > 1) { tq = 1; sp = clamp((b - cc) / a); }
  return [add(p0, mul(d1, sp)), add(q0, mul(d2, clamp(tq)))];
}

// Convex hull facets of a small point set, each ordered around its normal.
// ponytail: brute force, O(n^4) in a node's ring vertices (4 per strut, so n = 40 at 10 struts); fine to about
// 10 struts a node. Past that, swap in a real hull algorithm (e.g. incremental or quickhull, O(n log n)).
function hullFacets(P) {
  var seen = {}, out = [], n = P.length, scale = 1, i, j, k, m;
  for (i = 0; i < n; i++) scale = Math.max(scale, len(P[i]));
  var eps = 1e-7 * scale;
  for (i = 0; i < n; i++) for (j = i + 1; j < n; j++) for (k = j + 1; k < n; k++) {
    var nrm = cross(sub(P[j], P[i]), sub(P[k], P[i]));
    if (len(nrm) < eps) continue;
    nrm = unit(nrm);
    var pos = 0, neg = 0, on = [];
    for (m = 0; m < n; m++) {
      var d = dot(nrm, sub(P[m], P[i]));
      if (d > eps) pos++; else if (d < -eps) neg++; else on.push(m);
    }
    if (pos && neg) continue;
    var key = on.join(',');
    if (seen[key]) continue;
    seen[key] = 1;
    var c = [0, 0, 0];
    for (m = 0; m < on.length; m++) c = add(c, P[on[m]]);
    c = mul(c, 1 / on.length);
    var u = unit(sub(P[on[0]], c)), v = cross(nrm, u);
    var ang = function (q) { var dq = sub(P[q], c); return Math.atan2(dot(dq, v), dot(dq, u)); };
    on.sort(function (a, b) { return ang(a) - ang(b); });
    out.push(on);
  }
  return out;
}

// Consistent winding by walking shared edges, then each connected group flipped outward by signed volume.
// Returns the number of groups.
function orient(V, F) {
  var edgeFaces = {}, done = [], groups = 0, fi, i;
  function key(a, b) { return Math.min(a, b) + '_' + Math.max(a, b); }
  for (fi = 0; fi < F.length; fi++) for (i = 0; i < F[fi].length; i++) {
    var k = key(F[fi][i], F[fi][(i + 1) % F[fi].length]);
    (edgeFaces[k] = edgeFaces[k] || []).push(fi);
  }
  for (var start = 0; start < F.length; start++) {
    if (done[start]) continue;
    groups++;
    var comp = [start], stack = [start], vol = 0;
    done[start] = true;
    while (stack.length) {
      var f = F[stack.pop()];
      for (i = 0; i < f.length; i++) {
        var a = f[i], b = f[(i + 1) % f.length], nb = edgeFaces[key(a, b)];
        for (var n = 0; n < nb.length; n++) {
          var gj = nb[n], g = F[gj];
          if (done[gj]) continue;
          for (var j = 0; j < g.length; j++) if (g[j] === a && g[(j + 1) % g.length] === b) { g.reverse(); break; }
          done[gj] = true; comp.push(gj); stack.push(gj);
        }
      }
    }
    for (i = 0; i < comp.length; i++) {
      f = F[comp[i]];
      for (var t = 1; t + 1 < f.length; t++) vol += dot(V[f[0]], cross(V[f[t]], V[f[t + 1]]));
    }
    if (vol < 0) for (i = 0; i < comp.length; i++) F[comp[i]].reverse();
  }
  return groups;
}

function plan(curves, options) {
  var R = options.radius, tol = options.tolerance, cap = options.cap !== false, i, j, k;
  var V = [], F = [];
  var report = { pipeFrames: 0, struts: 0, nodes: 0, freeEnds: 0, duplicatesDropped: 0, crossings: 0, grownNodes: 0, largestReach: 0, shortStruts: 0,
    errors: [], warnings: [] };
  var out = { vertices: V, faces: F, box: null, report: report };
  if (!(R > 0)) report.errors.push('Radius must be greater than zero.');
  if (!(options.nodeSize >= 1)) report.errors.push('Node size must be at least 1.0.');
  var divs = options.divisions === undefined || options.divisions === 'auto' ? 0 : options.divisions;
  if (!(typeof divs === 'number' && isFinite(divs) && divs >= 0 && Math.floor(divs) === divs)) report.errors.push('Divisions must be a whole number of 0 or more.');
  if (!curves || !curves.length) report.errors.push('Select at least one curve.');
  if (report.errors.length) return out;
  var w = R * WIDTH, d0 = options.nodeSize * R;

  // ponytail: O(n^2) endpoint clustering, a spatial hash when large frames need it.
  var points = [], inc = [], struts = [], tracks = [];
  function node(p) {
    for (var n = 0; n < points.length; n++) if (len(sub(points[n], p)) <= tol) return n;
    points.push(p); inc.push([]);
    return points.length - 1;
  }
  // Duplicates: same ends in either direction, and the same quarter, middle and three-quarter points along the
  // samples (a line's samples are its ends), so a curve bowing the other way is kept.
  // ponytail: O(n^2) duplicate scan, same ceiling as the clustering.
  var kept = [];
  function at(ss, fraction) {
    var x = fraction * (ss.length - 1), lo = Math.floor(x), u = x - lo, p = ss[lo], q = ss[Math.min(lo + 1, ss.length - 1)];
    return add(p, mul(sub(q, p), u));
  }
  function near(p, q) { return len(sub(p, q)) <= tol; }
  function isDuplicate(a, b, samples) {
    var m = [at(samples, 0.25), at(samples, 0.5), at(samples, 0.75)];
    for (var d = 0; d < kept.length; d++) {
      var q = kept[d], forward = near(q.a, a) && near(q.b, b), back = near(q.a, b) && near(q.b, a);
      if (!near(q.m[1], m[1])) continue;
      if (forward && near(q.m[0], m[0]) && near(q.m[2], m[2]) || back && near(q.m[0], m[2]) && near(q.m[2], m[0])) {
        report.duplicatesDropped++;
        return true;
      }
    }
    kept.push({ a: a, b: b, m: m });
    return false;
  }
  function addStrut(tr) {
    var s = [node(tr.p[0]), node(tr.p[tr.p.length - 1])];
    inc[s[0]].push({ si: struts.length, end: 0 });
    inc[s[1]].push({ si: struts.length, end: 1 });
    struts.push(s); tracks.push(tr);
  }
  for (i = 0; i < curves.length; i++) {
    var c = curves[i], pts = c.kind === 'line' ? [c.start, c.end] : c.kind === 'polyline' ? c.points || [] : c.kind === 'smooth' ? c.samples || [] : null, nonZero = false;
    if (!pts) { report.errors.push('Curve ' + (i + 1) + ' is not a line, polyline or smooth curve; MultiPipe2 takes those.'); continue; }
    if (c.kind === 'smooth') {
      var tr = pts.length > 1 ? track(pts, c.startTangent, c.endTangent) : null;
      if (!tr || tr.L <= tol) { report.errors.push('Curve ' + (i + 1) + ' has zero length.'); continue; }
      if (!isDuplicate(tr.p[0], tr.p[tr.p.length - 1], tr.p)) addStrut(tr);
      continue;
    }
    for (j = 0; j + 1 < pts.length; j++) {
      if (near(pts[j], pts[j + 1])) continue;
      nonZero = true;
      if (!isDuplicate(pts[j], pts[j + 1], [pts[j], pts[j + 1]])) addStrut(track([pts[j], pts[j + 1]]));
    }
    if (!nonZero) report.errors.push('Curve ' + (i + 1) + ' has zero length.');
  }
  if (report.errors.length) return out;

  // Crossings: pieces (a smooth curve's samples, as chords) of two struts closer than tolerance away from all four of
  // their ends. Never joined, only warned about; a strut whose tip lands on another is a touch, not a crossing.
  // ponytail: O(n^2) strut pairs with a box reject, O(pieces^2) inside; a spatial grid when large frames need it.
  function where(p) { return '(' + p.map(function (x) { return x.toFixed(3); }).join(', ') + ')'; }
  var boxes = tracks.map(function (t) {
    var b = [1e30, 1e30, 1e30, -1e30, -1e30, -1e30];
    t.p.forEach(function (q) { for (var a = 0; a < 3; a++) { b[a] = Math.min(b[a], q[a] - tol); b[a + 3] = Math.max(b[a + 3], q[a] + tol); } });
    return b;
  });
  for (i = 0; i < struts.length; i++) for (j = i + 1; j < struts.length; j++) {
    var bi = boxes[i], bj = boxes[j], ends = [points[struts[i][0]], points[struts[i][1]], points[struts[j][0]], points[struts[j][1]]], seen = [];
    if (bi[0] > bj[3] || bj[0] > bi[3] || bi[1] > bj[4] || bj[1] > bi[4] || bi[2] > bj[5] || bj[2] > bi[5]) continue;
    for (var pa = 0; pa + 1 < tracks[i].p.length; pa++) for (var pb = 0; pb + 1 < tracks[j].p.length; pb++) {
      var cp = closest(tracks[i].p[pa], tracks[i].p[pa + 1], tracks[j].p[pb], tracks[j].p[pb + 1]);
      if (!near(cp[0], cp[1])) continue;
      var pt = mul(add(cp[0], cp[1]), 0.5), touch = false;
      for (k = 0; k < 4; k++) if (len(sub(ends[k], pt)) <= 2 * tol) touch = true;
      for (k = 0; k < seen.length; k++) if (len(sub(seen[k], pt)) <= 2 * tol) touch = true;
      if (touch) continue;
      seen.push(pt);
      report.crossings++;
      report.warnings.push('Curves cross without sharing an end near ' + where(pt) + '; left unjoined.');
    }
  }

  // One ring offset per node, shared by all its struts: nodeSize x R, grown so no neighbouring ring's corner
  // reaches past a ring's plane. It must be one value per node; the bound assumes both rings sit at the same offset.
  // A closed ring: a strut from a node back to itself with nothing else there. It becomes a closed tube, no node.
  function isLoop(n) { return inc[n].length === 2 && inc[n][0].si === inc[n][1].si; }
  function away(e) { var t = tracks[e.si].t; return e.end ? mul(t[t.length - 1], -1) : t[0]; }
  var reach = [], grew = [];
  for (var ni = 0; ni < points.length; ni++) {
    var here = inc[ni], need = d0;
    if (isLoop(ni)) { reach.push(0); grew.push(false); continue; }
    for (j = 0; j < here.length; j++) for (k = j + 1; k < here.length; k++) {
      var th = Math.acos(Math.max(-1, Math.min(1, dot(away(here[j]), away(here[k])))));
      if (th < 1e-6) { report.errors.push('Two curves at ' + where(points[ni]) + ' run in the same direction.'); continue; }
      need = Math.max(need, 1.05 * w * Math.SQRT2 / Math.tan(th / 2));
    }
    var didGrow = here.length > 1 && need > d0 * 1.0001;
    if (didGrow) { report.grownNodes++; report.largestReach = Math.max(report.largestReach, need / R); }
    reach.push(need); grew.push(didGrow);
  }
  if (report.errors.length) return out;

  // Ring frames carried along each strut by its rotation-minimising frame, so the tube does not twist; a straight
  // strut keeps one frame. Reference axis world Z, or X when the start is near-vertical, so axis-aligned frames give
  // cube-like joints.
  var auto = options.divisions === undefined || options.divisions === 'auto', rings = [];
  for (i = 0; i < struts.length; i++) {
    tr = tracks[i];
    var L = tr.L, free0 = inc[struts[i][0]].length === 1, free1 = inc[struts[i][1]].length === 1, loop = isLoop(struts[i][0]), rs = [], fr = [];
    if (loop) {
      // Closed tube: rings evenly round the loop. The frame carried round comes back turned by phi about the
      // tangent; that turn is taken out evenly along the loop so the last ring meets the first.
      var nl = auto ? spans(tr, 0, L, 3) : Math.max(3, divs + 1), last = tr.r.length - 1;
      var phi = Math.atan2(dot(cross(tr.r[0], tr.r[last]), tr.t[0]), dot(tr.r[0], tr.r[last]));
      for (j = 0; j < nl; j++) {
        var f0 = frameAt(tr, L * j / nl), ang = -phi * j / nl;
        f0.r = add(mul(f0.r, Math.cos(ang)), mul(cross(f0.t, f0.r), Math.sin(ang)));
        fr.push(f0);
      }
    }
    // Ring centres as distances from a: the node offset or the free end, plus the extra ring 1 x R in at a free end.
    // A free end counts its extra ring, 1 x R, as its offset for the short-strut check.
    else {
      var d1 = free0 ? R : reach[struts[i][0]], d2 = free1 ? R : reach[struts[i][1]];
      if (L < d1 + d2) report.shortStruts++;
      var at = [free0 ? 0 : d1];
      if (free0) at.push(R);
      if (free1) at.push(L - R);
      at.push(free1 ? L : L - d2);
      // Divisions: N extra rings spaced evenly between the innermost rings; Auto adds them only where the curve turns.
      var lo = at[free0 ? 1 : 0], hi = at[free1 ? at.length - 2 : at.length - 1], nd = auto ? spans(tr, lo, hi, 1) - 1 : divs;
      for (j = 1; j <= nd; j++) at.splice(at.length - (free1 ? 2 : 1), 0, lo + (hi - lo) * j / (nd + 1));
      // Round joints: one extra plain ring per qualifying node end, e further inboard than the joint ring
      // (independent of Divisions). e is at least w, but grows with how far the node's ring was pushed out
      // past its normal d0 offset (reach - d0), so a heavily grown node (ticket 02's acute-angle offset) gets
      // a gentler taper instead of a fixed w-wide step that Catmull-Clark overshoots past the corner.
      // Skipped silently if the strut has no room for it.
      if (options.roundJoints) {
        var qualifies = function (nid) { return inc[nid].length > 1 && (options.allNodes || grew[nid]); };
        if (!free0 && qualifies(struts[i][0])) {
          var e0 = Math.max(w, reach[struts[i][0]] - d0), c0 = at[0] + e0;
          if (c0 < at[1] - 1e-9) at.splice(1, 0, c0);
        }
        if (!free1 && qualifies(struts[i][1])) {
          var e1 = Math.max(w, reach[struts[i][1]] - d0), c1 = at[at.length - 1] - e1;
          if (c1 > at[at.length - 2] + 1e-9) at.splice(at.length - 1, 0, c1);
        }
      }
      for (j = 0; j < at.length; j++) fr.push(frameAt(tr, at[j]));
    }
    for (j = 0; j < fr.length; j++) {
      var e2 = cross(fr[j].t, fr[j].r), ring = [];
      for (k = 0; k < 4; k++) {
        var q = [[1, 1], [-1, 1], [-1, -1], [1, -1]][k];
        V.push(add(fr[j].c, add(mul(fr[j].r, q[0] * w), mul(e2, q[1] * w))));
        ring.push(V.length - 1);
      }
      rs.push(ring);
    }
    if (loop) rs.push(rs[0]);
    for (j = 0; j + 1 < rs.length; j++) for (k = 0; k < 4; k++) {
      F.push([rs[j][k], rs[j + 1][k], rs[j + 1][(k + 1) % 4], rs[j][(k + 1) % 4]]);
    }
    rings.push([rs[0], rs[rs.length - 1]]);
  }

  for (ni = 0; ni < points.length; ni++) {
    here = inc[ni];
    if (isLoop(ni)) continue;
    if (here.length === 1) {
      report.freeEnds++;
      if (cap) F.push(rings[here[0].si][here[0].end].slice());
      continue;
    }
    report.nodes++;
    // Joint: the hull of the node's ring vertices, minus the ring facets; those are where the tubes attach.
    var ids = [], P = [], ringKeys = {}, found = {};
    for (j = 0; j < here.length; j++) {
      var rg = rings[here[j].si][here[j].end];
      for (k = 0; k < 4; k++) { ids.push(rg[k]); P.push(V[rg[k]]); }
      ringKeys[[j * 4, j * 4 + 1, j * 4 + 2, j * 4 + 3].join(',')] = j;
    }
    var facets = hullFacets(P);
    for (j = 0; j < facets.length; j++) {
      var fk = facets[j].slice().sort(function (x, y) { return x - y; }).join(',');
      if (fk in ringKeys) { found[ringKeys[fk]] = true; continue; }
      F.push(facets[j].map(function (m) { return ids[m]; }));
    }
    for (j = 0; j < here.length; j++) {
      if (!found[j]) report.errors.push('The joint at ' + where(points[ni]) + ' could not be built; its struts meet at too tight an angle.');
    }
  }
  report.struts = struts.length;
  report.pipeFrames = orient(V, F);

  var box = [1e30, 1e30, 1e30, -1e30, -1e30, -1e30];
  for (i = 0; i < V.length; i++) for (k = 0; k < 3; k++) {
    box[k] = Math.min(box[k], V[i][k]); box[k + 3] = Math.max(box[k + 3], V[i][k]);
  }
  out.box = box;
  return out;
}

function objLines(cage) {
  var lines = [], i;
  for (i = 0; i < cage.vertices.length; i++) {
    var v = cage.vertices[i];
    lines.push('v ' + v[0].toFixed(6) + ' ' + v[2].toFixed(6) + ' ' + (-v[1]).toFixed(6));
  }
  for (i = 0; i < cage.faces.length; i++) lines.push('f ' + cage.faces[i].map(function (x) { return x + 1; }).join(' '));
  return lines;
}

function checkImport(cageBox, boxes, tolerance) {
  var lo = [1e30, 1e30, 1e30], hi = [-1e30, -1e30, -1e30], ok = true, i, k;
  for (i = 0; i < boxes.length; i++) for (k = 0; k < 3; k++) {
    if (boxes[i][k] < cageBox[k] - tolerance || boxes[i][k + 3] > cageBox[k + 3] + tolerance) ok = false;
    lo[k] = Math.min(lo[k], boxes[i][k]); hi[k] = Math.max(hi[k], boxes[i][k + 3]);
  }
  for (k = 0; k < 3; k++) if (hi[k] - lo[k] < 0.8 * (cageBox[k + 3] - cageBox[k])) ok = false;
  return ok ? '' : 'The pipe frame came in at the wrong size or place, so it was removed. ' +
    'Check that MoI\'s SubD import ScaleFactor setting is 1.';
}

if (typeof module !== 'undefined') module.exports = { plan: plan, objLines: objLines, checkImport: checkImport };
