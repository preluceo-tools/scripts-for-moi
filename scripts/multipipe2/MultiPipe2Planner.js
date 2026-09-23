// MultiPipe2 planner: curve descriptions in, cage out. Pure ES5, no MoI API,
// so it runs inside MoI (via #include) and under Node (via require).
//
// plan(curves, options)
//   curves:  [ { kind: 'line', start: [x,y,z], end: [x,y,z] } ]  any other kind is an error
//   options: { radius, nodeSize, cap, tolerance }  nodeSize >= 1.0; cap defaults to true
//   returns: { vertices: [[x,y,z], ...], faces: [[i, j, k, l], ...], box: [minX, minY, minZ, maxX, maxY, maxZ],
//              report: { pipeFrames, struts, nodes, freeEnds, grownNodes, largestReach, shortStruts, errors, warnings } }
//   largestReach: the largest ring offset at a grown node, as a factor of radius (0 when none grew).
//   shortStruts: struts shorter than the ring offsets at their two ends (still built).
//   The cage: a square ring (half-width radius / 0.93) at the node's ring offset from every node on each strut,
//   the convex hull of a node's rings (ring facets removed) as its joint, quad tubes between rings, and at a
//   free end an end ring, an extra ring 1 x radius in, and a cap face when cap is on. Faces wind outward.
//
// objLines(cage)  -> OBJ lines, Y-up as the SubD import expects: (x, z, -y). No groups, no materials.
// checkImport(cageBox, boxes, tolerance) -> '' when the imported boxes lie inside the cage box and cover at
//   least 80% of it on every axis, else the message for the user. boxes: [[minX, ..., maxZ], ...].

var WIDTH = 1 / 0.93; // a 4-ring's limit radius is 0.917 w to 0.943 w, so w = radius / 0.93 centres it on radius

function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function mul(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function len(a) { return Math.sqrt(dot(a, a)); }
function unit(a) { return mul(a, 1 / len(a)); }

// Convex hull facets of a small point set, each ordered around its normal.
// ponytail: brute force, O(n^4) in a node's ring vertices (4 per strut); fine to ~10 struts a node.
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
  var report = { pipeFrames: 0, struts: 0, nodes: 0, freeEnds: 0, grownNodes: 0, largestReach: 0, shortStruts: 0,
    errors: [], warnings: [] };
  var out = { vertices: V, faces: F, box: null, report: report };
  if (!(R > 0)) report.errors.push('Radius must be greater than zero.');
  if (!(options.nodeSize >= 1)) report.errors.push('Node size must be at least 1.0.');
  if (!curves || !curves.length) report.errors.push('Select at least one curve.');
  if (report.errors.length) return out;
  var w = R * WIDTH, d0 = options.nodeSize * R;

  // ponytail: O(n^2) endpoint clustering, a spatial hash when large frames need it.
  var points = [], inc = [], struts = [];
  function node(p) {
    for (var n = 0; n < points.length; n++) if (len(sub(points[n], p)) <= tol) return n;
    points.push(p); inc.push([]);
    return points.length - 1;
  }
  for (i = 0; i < curves.length; i++) {
    var c = curves[i];
    if (c.kind !== 'line') { report.errors.push('Curve ' + (i + 1) + ' is not a straight line; MultiPipe2 takes straight lines.'); continue; }
    if (len(sub(c.end, c.start)) <= tol) { report.errors.push('Curve ' + (i + 1) + ' has zero length.'); continue; }
    var s = [node(c.start), node(c.end)];
    inc[s[0]].push({ si: struts.length, end: 0 });
    inc[s[1]].push({ si: struts.length, end: 1 });
    struts.push(s);
  }
  if (report.errors.length) return out;

  // One ring offset per node, shared by all its struts: nodeSize x R, grown so no neighbouring ring's corner
  // reaches past a ring's plane. It must be one value per node; the bound assumes both rings sit at the same offset.
  function where(p) { return '(' + p.map(function (x) { return x.toFixed(3); }).join(', ') + ')'; }
  function away(e) { var s = struts[e.si]; return unit(sub(points[s[1 - e.end]], points[s[e.end]])); }
  var reach = [];
  for (var ni = 0; ni < points.length; ni++) {
    var here = inc[ni], need = d0;
    for (j = 0; j < here.length; j++) for (k = j + 1; k < here.length; k++) {
      var th = Math.acos(Math.max(-1, Math.min(1, dot(away(here[j]), away(here[k])))));
      if (th < 1e-6) { report.errors.push('Two curves at ' + where(points[ni]) + ' run in the same direction.'); continue; }
      need = Math.max(need, 1.05 * w * Math.SQRT2 / Math.tan(th / 2));
    }
    if (here.length > 1 && need > d0 * 1.0001) { report.grownNodes++; report.largestReach = Math.max(report.largestReach, need / R); }
    reach.push(need);
  }
  if (report.errors.length) return out;

  // One ring frame per strut, used at both ends so the tube does not twist. Reference axis world Z, or X when
  // the strut is near-vertical, so axis-aligned frames give cube-like joints.
  var rings = [];
  for (i = 0; i < struts.length; i++) {
    var a = points[struts[i][0]], b = points[struts[i][1]], L = len(sub(b, a)), dir = unit(sub(b, a));
    var e1 = unit(cross(Math.abs(dir[2]) > 0.9 ? [1, 0, 0] : [0, 0, 1], dir)), e2 = cross(dir, e1);
    var free0 = inc[struts[i][0]].length === 1, free1 = inc[struts[i][1]].length === 1;
    // Ring centres as distances from a: the node offset or the free end, plus the extra ring 1 x R in at a free end.
    // A free end counts its extra ring, 1 x R, as its offset for the short-strut check.
    var d1 = free0 ? R : reach[struts[i][0]], d2 = free1 ? R : reach[struts[i][1]];
    if (L < d1 + d2) report.shortStruts++;
    var at = [free0 ? 0 : d1];
    if (free0) at.push(R);
    if (free1) at.push(L - R);
    at.push(free1 ? L : L - d2);
    var rs = [];
    for (j = 0; j < at.length; j++) {
      var ctr = add(a, mul(dir, at[j])), ring = [];
      for (k = 0; k < 4; k++) {
        var q = [[1, 1], [-1, 1], [-1, -1], [1, -1]][k];
        V.push(add(ctr, add(mul(e1, q[0] * w), mul(e2, q[1] * w))));
        ring.push(V.length - 1);
      }
      rs.push(ring);
    }
    for (j = 0; j + 1 < rs.length; j++) for (k = 0; k < 4; k++) {
      F.push([rs[j][k], rs[j + 1][k], rs[j + 1][(k + 1) % 4], rs[j][(k + 1) % 4]]);
    }
    rings.push([rs[0], rs[rs.length - 1]]);
  }

  for (ni = 0; ni < points.length; ni++) {
    here = inc[ni];
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
