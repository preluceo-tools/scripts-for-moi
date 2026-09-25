// MultiPipe2 MoI suite, run inside MoI through the MoI MCP bridge (moi_eval). ES5 only.
// It loads the planner and the command's functions fresh from <repo>, makes each scene's curves as MoI curves
// (calculate() only, never added to the document), describes them with the command's own describe(), imports the
// cage through the command's own importCage(), checks the solids, then removes everything it made. Call it with:
//
//   var ROOT = '<repo folder>/';   // forward slashes, trailing slash
//   var s = moi.filesystem.openFileStream(ROOT + 'tests/multipipe2/moi/moi-suite.js', 'r'), t = '';
//   while (!s.AtEOF) t += s.readLine() + '\n';
//   s.close(); (0, eval)(t);
//   return runMoiSuite(ROOT);
//
// Every scene runs with Cap on; the 'cap off' runs repeat some with Cap off, where each free end must come in as one
// open boundary (its naked edges join into one closed loop), and a frame without free ends must still be a closed
// solid. arcframe, helix and ring are smooth curves (an interpolated half circle in a frame, a two-turn helix, a closed
// circle, which must come in as one closed tube with no node). The 'N=3' runs repeat some with Divisions 3 and must still come in as closed solids.
// The 'round joints, all nodes' runs repeat roofTruss and twobends with Round joints and All nodes on, so the
// apex-vertex fan joint (ticket 11) is checked live too, not just by the planner tests.
// Last comes 'lattice', a generated 4 x 4 x 4 grid (300 struts, 125 nodes, Radius 0.5): one closed solid, with its
// planner (describe plus plan), write and import times recorded separately (write is objLines plus writing the OBJ;
// import is importCage less that write).
// Then the cage-curves runs (ticket 12): the same planner data built as one closed curve per cage face with the
// command's own buildCageCurves(), checked for count, closedness and that no file is written, on scenes covering
// faces of 3, 4, 6, 8 and 10 vertices plus a 2-degree strut pair whose joint cannot be built.
// Returns { passed, failed: [ { scene, reason } ], results: [ { scene, ms, objects } ] }.
//
// The whole run can outlast the bridge's 30 s call timeout. The timed-out call keeps running inside MoI and leaves
// the result in the global MULTIPIPE2_SUITE; read it back with a follow-up moi_eval of:
//
//   return typeof MULTIPIPE2_SUITE == 'undefined' ? 'still running' : MULTIPIPE2_SUITE;

function runMoiSuite(root) {
  MULTIPIPE2_SUITE = undefined;   // so a follow-up read cannot return an earlier run's result
  function read(path) {
    var s = moi.filesystem.openFileStream(root + path, 'r'), t = '';
    while (!s.AtEOF) t += s.readLine() + '\n';
    s.close();
    return t;
  }
  (0, eval)(read('scripts/multipipe2/MultiPipe2Planner.js'));
  // The command's helpers without its #include line and without running it.
  (0, eval)(read('scripts/multipipe2/MultiPipe2.js').replace(/^#include.*$/m, '').replace(/^MultiPipe2\(\);\s*$/m, ''));
  var scenes = eval('(' + read('tests/multipipe2/scenes/scenes.json') + ')');

  var gd = moi.geometryDatabase, R = 2, tol = gd.tolerance;
  var tmp = moi.filesystem.getTempDir() + 'MultiPipe2-cage.obj';
  var names = ['line', 'bend90', 'cubeframe', 'twobends', 'hairpin30', 'k5skew', 'd8', 'roofTruss', 'polyframe', 'arcframe', 'helix', 'ring'], runs = [], n;
  for (n = 0; n < names.length; n++) runs.push({ name: names[n], cap: true });
  runs.push({ name: 'line', cap: false }, { name: 'bend90', cap: false }, { name: 'polyframe', cap: false }, { name: 'cubeframe', cap: false });
  runs.push({ name: 'cubeframe', cap: true, divisions: 3 }, { name: 'polyframe', cap: true, divisions: 3 }, { name: 'arcframe', cap: true, divisions: 3 });
  // Round joints, all nodes: the apex-vertex fan joint (ticket 11) must still import as a closed solid inside the
  // cage's own box, on the scene the bug was reported against (roofTruss) and a clean one (twobends).
  runs.push({ name: 'roofTruss', cap: true, roundJoints: true, allNodes: true }, { name: 'twobends', cap: true, roundJoints: true, allNodes: true });
  var VM = moi.vectorMath;
  function P(a) { return VM.createPoint(a[0], a[1], a[2]); }
  function curve(c) {
    var f = moi.command.createFactory(c.type === 'interp' ? 'interpcurve' : c.type === 'circle' ? 'circle' : 'polyline');
    if (c.type === 'circle') {
      f.setInput(0, true); f.setInput(1, VM.createFrame(P(c.center), P([1, 0, 0]), P([0, 1, 0]))); f.setInput(3, c.radius);
    } else for (var j = 0; j < c.pts.length; j++) {
      f.createInput('point'); f.setInput(f.numInputs - 1, P(c.pts[j]));
      if (c.type === 'interp') { f.createInput('bool'); f.setInput(f.numInputs - 1, false); }
    }
    var r = f.calculate(); f.cancel();
    return r.item(0);
  }
  // An object's open boundaries as closed loops: a naked edge that is closed on its own is one, the rest are joined
  // (join returns nothing when it has nothing to join). Returns -1 when some boundary does not close.
  function boundaries(obj) {
    var naked = obj.getNakedEdges(), rest = gd.createObjectList(), loops = 0, i;
    for (i = 0; i < naked.length; i++) if (naked.item(i).isClosed) loops++; else rest.addObject(naked.item(i));
    if (!rest.length) return loops;
    var f = moi.command.createFactory('join');
    f.setInput(0, rest);
    var r = f.calculate(); f.cancel();
    for (i = 0; i < r.length; i++) if (r.item(i).isClosed) loops++; else return -1;
    return r.length ? loops : -1;
  }
  var before = gd.getObjects().length, out = { passed: 0, failed: [], results: [] };
  for (n = 0; n < runs.length; n++) {
    var name = runs[n].name, label = name + (runs[n].cap ? '' : ' (cap off)') + (runs[n].divisions ? ' (N=' + runs[n].divisions + ')' : '') +
      (runs[n].roundJoints ? ' (round joints' + (runs[n].allNodes ? ', all nodes' : '') + ')' : ''), spec = scenes[name], curves = gd.createObjectList(), i, reason = '';
    for (i = 0; i < spec.length; i++) curves.addObject(curve(spec[i]));
    var input = describe(curves);
    var cage = plan(input, { radius: R, nodeSize: 1.6, divisions: runs[n].divisions, cap: runs[n].cap, tolerance: tol,
      roundJoints: runs[n].roundJoints, allNodes: runs[n].allNodes }), t0 = new Date().getTime();
    var fatal = cage.report.errors.concat(cage.report.jointErrors);
    var objs = fatal.length ? gd.createObjectList() : importCage(cage), boxes = [], nakedLoops = 0;
    out.results.push({ scene: label, ms: new Date().getTime() - t0, objects: objs.length });
    for (i = 0; i < objs.length; i++) {
      var b = objs.item(i).getBoundingBox();
      boxes.push([b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z]);
      var open = !runs[n].cap && cage.report.freeEnds, loops = boundaries(objs.item(i));
      if (!open && !objs.item(i).isSolidBRep) reason = 'object ' + i + ' is not a closed solid';
      if (open && objs.item(i).isSolidBRep) reason = 'object ' + i + ' is closed with Cap off';
      if (loops < 0) reason = 'object ' + i + ' has an open boundary that is not a closed loop';
      nakedLoops += loops;
      objs.item(i).name = '';
      if (objs.item(i).name) reason = 'name not cleared';
    }
    if (fatal.length) reason = fatal.join('; ');
    else if (name === 'polyframe' && (input[0].kind !== 'polyline' || objs.length !== 1)) reason = 'polyline not one pipe frame';
    else if (objs.length !== cage.report.pipeFrames) reason = 'expected ' + cage.report.pipeFrames + ' pipe frames, got ' + objs.length;
    else if (moi.filesystem.fileExists(tmp)) reason = 'temp file left behind';
    else if (!reason && nakedLoops !== (runs[n].cap ? 0 : cage.report.freeEnds)) reason = nakedLoops + ' open boundaries, expected ' + (runs[n].cap ? 0 : cage.report.freeEnds);
    else if (!reason) reason = checkImport(cage.box, boxes, tol);
    if (!reason && name === 'line' && runs[n].cap) {
      // A lone strut along X from 0 to 30: its radius within 2% of R, its rounded ends within 0.15 R of the line's ends.
      var bb = boxes[0], rad = (bb[4] - bb[1] + bb[5] - bb[2]) / 4;
      out.results[n].radius = rad;
      out.results[n].ends = [bb[0], 30 - bb[3]];
      if (Math.abs(rad - R) > 0.02 * R) reason = 'radius ' + rad + ', expected ' + R;
      else if (bb[0] > 0.15 * R || 30 - bb[3] > 0.15 * R) reason = 'ends stop ' + bb[0] + ' and ' + (30 - bb[3]) + ' short';
    }
    if (!reason && name === 'ring') {
      // A circle of radius 10 about the origin in XY: outer edge near 10 + R (the subdivided centreline runs about 1% inside the curve).
      bb = boxes[0];
      out.results[n].outer = (bb[3] - bb[0] + bb[4] - bb[1]) / 4;
      out.results[n].thickness = bb[5] - bb[2];
      if (cage.report.nodes || cage.report.freeEnds) reason = 'a ring has nodes or free ends';
      else if (Math.abs(out.results[n].outer - 10 - R) > 0.1 * R) reason = 'outer radius ' + out.results[n].outer + ', expected ' + (10 + R);
    }
    if (objs.length) gd.removeObjects(objs);
    if (reason) out.failed.push({ scene: label, reason: reason }); else out.passed++;
  }
  // Cage curves output (ticket 12): one closed curve per cage face, whatever the face size, nothing written to
  // disk, and a joint that could not be built is a warning rather than a stop. cubeframe has 3- and 4-vertex
  // faces, roofTruss 4, 6 and 10, x4planar 4 and 8; 'tightpair' is two struts 2 degrees apart, which is the
  // angle at which the joint actually fails (5 degrees still builds here, measured).
  var cageRuns = ['cubeframe', 'roofTruss', 'x4planar', 'tightpair'], A2 = 2 * Math.PI / 180;
  for (n = 0; n < cageRuns.length; n++) {
    var cname = cageRuns[n], cspec = cname === 'tightpair'
      ? [{ type: 'line', pts: [[0, 0, 0], [100, 0, 0]] },
         { type: 'line', pts: [[0, 0, 0], [100 * Math.cos(A2), 100 * Math.sin(A2), 0]] }]
      : scenes[cname];
    var why2 = '', sizes = {}, key;
    curves = gd.createObjectList();
    for (i = 0; i < cspec.length; i++) curves.addObject(curve(cspec[i]));
    cage = plan(describe(curves), { radius: R, nodeSize: 1.6, cap: true, tolerance: tol });
    for (i = 0; i < cage.faces.length; i++) { key = cage.faces[i].length; sizes[key] = (sizes[key] || 0) + 1; }
    var t1 = new Date().getTime();
    objs = cage.report.errors.length ? gd.createObjectList() : buildCageCurves(cage);
    var res = { scene: cname + ' (cage curves)', ms: new Date().getTime() - t1, objects: objs.length, faceSizes: sizes };
    out.results.push(res);
    for (i = 0; i < objs.length; i++) {
      if (!objs.item(i).isCurve || !objs.item(i).isClosed) why2 = 'object ' + i + ' is not a closed curve';
      objs.item(i).name = '';
    }
    if (cage.report.errors.length) why2 = cage.report.errors.join('; ');
    else if (objs.length !== cage.faces.length) why2 = objs.length + ' curves, expected ' + cage.faces.length;
    else if (moi.filesystem.fileExists(tmp)) why2 = 'temp file left behind';
    else if (cname === 'tightpair' && !cage.report.jointErrors.length) why2 = 'the 2 degree pair built its joint, so it does not test the warning';
    else if (cname !== 'tightpair' && cage.report.jointErrors.length) why2 = cage.report.jointErrors.join('; ');
    if (objs.length) gd.removeObjects(objs);
    if (why2) out.failed.push({ scene: cname + ' (cage curves)', reason: why2 }); else out.passed++;
  }

  // Large frame: time each stage; the import must give one closed solid.
  var L = [], g = 4, S = 10, a, c, d;
  for (a = 0; a <= g; a++) for (c = 0; c <= g; c++) for (d = 0; d <= g; d++) {
    if (a < g) L.push({ type: 'line', pts: [[a * S, c * S, d * S], [a * S + S, c * S, d * S]] });
    if (c < g) L.push({ type: 'line', pts: [[a * S, c * S, d * S], [a * S, c * S + S, d * S]] });
    if (d < g) L.push({ type: 'line', pts: [[a * S, c * S, d * S], [a * S, c * S, d * S + S]] });
  }
  curves = gd.createObjectList();
  for (i = 0; i < L.length; i++) curves.addObject(curve(L[i]));
  var t = new Date().getTime(), lat = { scene: 'lattice' }, why = '';
  cage = plan(describe(curves), { radius: 0.5, nodeSize: 1.6, cap: true, tolerance: tol });
  lat.planMs = new Date().getTime() - t;
  out.results.push(lat);
  if (cage.report.errors.concat(cage.report.jointErrors).length) why = cage.report.errors.concat(cage.report.jointErrors).join('; ');
  else if (cage.report.struts !== 300 || cage.report.nodes !== 125) why = cage.report.struts + ' struts and ' + cage.report.nodes + ' nodes, expected 300 and 125';
  else {
    var wpath = moi.filesystem.getTempDir() + 'MultiPipe2-write-timing.obj', lines, ws;
    t = new Date().getTime();
    lines = objLines(cage); ws = moi.filesystem.openFileStream(wpath, 'w');
    try { for (i = 0; i < lines.length; i++) ws.writeLine(lines[i]); } finally { ws.close(); moi.filesystem.deleteFile(wpath); }
    lat.writeMs = new Date().getTime() - t;
    t = new Date().getTime();
    objs = importCage(cage);
    lat.importMs = new Date().getTime() - t - lat.writeMs;
    lat.totalMs = lat.planMs + lat.writeMs + lat.importMs;
    lat.objects = objs.length;
    if (objs.length !== 1 || !objs.item(0).isSolidBRep) why = 'expected one closed solid, got ' + objs.length + ' objects';
    else if (boundaries(objs.item(0)) !== 0) why = 'open boundaries on the lattice';
    else if (moi.filesystem.fileExists(tmp)) why = 'temp file left behind';
    else {
      b = objs.item(0).getBoundingBox();
      why = checkImport(cage.box, [[b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z]], tol);
    }
    if (objs.length) gd.removeObjects(objs);
  }
  if (why) out.failed.push({ scene: 'lattice', reason: why }); else out.passed++;

  if (gd.getObjects().length !== before) out.failed.push({ scene: '*', reason: 'document changed' });
  // The run can outlast the bridge's call timeout, so the result is left in a global for a follow-up call to read.
  MULTIPIPE2_SUITE = out;
  return out;
}
