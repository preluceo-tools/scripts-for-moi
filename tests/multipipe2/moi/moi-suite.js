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
// faces of 3, 4, 6, 8 and 10 vertices plus a 2-degree strut pair whose joint cannot be built. The same scenes
// repeat for the cage surfaces output (ticket 15) — one surface per face, counted by diffing the document, with
// the construction curves cleaned up — and for the cage solid output (ticket 16), where those surfaces are joined
// into one closed solid per pipe frame, with 'twoframes' (cubeframe plus a distant line) checking that two groups
// come out as two solids. The lattice cage gets a timed surfaces run and a timed solid run, which fail if the
// batched planarsrf is ever replaced by a call per face.
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
    var fatal = cage.report.errors.slice();
    if (cage.report.jointFailures) fatal.push(cage.report.jointFailures + ' joint(s) could not be built');
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
  // The same runs repeat for the cage surfaces output (ticket 15): one planar surface per face, built with a
  // single batched planarsrf, with the construction curves gone from the document afterwards.
  var cageRuns = ['cubeframe', 'roofTruss', 'x4planar', 'tightpair', 'twoframes'], A2 = 2 * Math.PI / 180;
  var kinds = ['curves', 'surfaces', 'solid'], ki, kind;
  for (ki = 0; ki < kinds.length; ki++)
  for (n = 0; n < cageRuns.length; n++) {
    kind = kinds[ki];
    var cname = cageRuns[n], clabel = cname + ' (cage ' + kind + ')', cspec = cname === 'tightpair'
      ? [{ type: 'line', pts: [[0, 0, 0], [100, 0, 0]] },
         { type: 'line', pts: [[0, 0, 0], [100 * Math.cos(A2), 100 * Math.sin(A2), 0]] }]
      // Two connected groups 200 units apart: one solid per group, so join must not weld them into one object.
      : cname === 'twoframes'
      ? scenes.cubeframe.concat([{ type: 'line', pts: [[200, 0, 0], [300, 0, 0]] }])
      : scenes[cname];
    var why2 = '', sizes = {}, key;
    curves = gd.createObjectList();
    for (i = 0; i < cspec.length; i++) curves.addObject(curve(cspec[i]));
    cage = plan(describe(curves), { radius: R, nodeSize: 1.6, cap: true, tolerance: tol });
    for (i = 0; i < cage.faces.length; i++) { key = cage.faces[i].length; sizes[key] = (sizes[key] || 0) + 1; }
    var t1 = new Date().getTime();
    objs = cage.report.errors.length ? gd.createObjectList()
      : kind === 'curves' ? buildCageCurves(cage) : kind === 'surfaces' ? buildCageSurfaces(cage) : buildCageSolid(cage);
    // Cage solid joins the faces, so the count to expect is one object per pipe frame, not one per face — except on
    // tightpair, where the joint that could not be built leaves its cage in 3 unconnected pieces (measured).
    var want = kind !== 'solid' ? cage.faces.length : cname === 'tightpair' ? 3 : cage.report.pipeFrames;
    var res = { scene: clabel, ms: new Date().getTime() - t1, objects: objs.length, faceSizes: sizes };
    out.results.push(res);
    for (i = 0; i < objs.length; i++) {
      if (kind === 'curves' ? (!objs.item(i).isCurve || !objs.item(i).isClosed) : objs.item(i).isCurve) why2 = 'object ' + i + ' is not a ' + (kind === 'curves' ? 'closed curve' : 'surface');
      // A failed joint leaves the cage open there, so tightpair is the one scene whose solid is not closed.
      if (kind === 'solid' && cname !== 'tightpair' && !objs.item(i).isSolidBRep) why2 = 'object ' + i + ' is not a closed solid';
      objs.item(i).name = '';
    }
    if (cage.report.errors.length) why2 = cage.report.errors.join('; ');
    else if (objs.length !== want) why2 = objs.length + ' ' + kind + ', expected ' + want;
    else if (moi.filesystem.fileExists(tmp)) why2 = 'temp file left behind';
    // The cage surfaces output builds curves and must clean them up: nothing but its surfaces may be left.
    else if (gd.getObjects().length !== before + objs.length) why2 = (gd.getObjects().length - before - objs.length) + ' extra objects left in the document';
    else if (cname === 'tightpair' && !cage.report.jointFailures) why2 = 'the 2 degree pair built its joint, so it does not test the warning';
    else if (cname !== 'tightpair' && cage.report.jointFailures) why2 = cage.report.jointFailures + ' joint(s) could not be built';
    if (objs.length) gd.removeObjects(objs);
    if (why2) out.failed.push({ scene: clabel, reason: why2 }); else out.passed++;
  }

  // Partial build (ticket 13): cubeframe plus a 2 degree hairpin 200 units away. Written whole, that cage imports
  // as nothing at all, taking the good frame with it; dropping the failed frame must leave cubeframe's own solid.
  var pspec = scenes.cubeframe.concat([{ type: 'line', pts: [[200, 0, 0], [300, 0, 0]] },
    { type: 'line', pts: [[200, 0, 0], [200 + 100 * Math.cos(A2), 100 * Math.sin(A2), 0]] }]);
  curves = gd.createObjectList();
  for (i = 0; i < pspec.length; i++) curves.addObject(curve(pspec[i]));
  cage = plan(describe(curves), { radius: R, nodeSize: 1.6, cap: true, tolerance: tol });
  var pwhy = '', pr = cage.report;
  objs = pr.errors.length || !cage.partial ? gd.createObjectList() : importCage(cage.partial);
  out.results.push({ scene: 'cubeframe + tight hairpin (partial build)', objects: objs.length,
    jointFailures: pr.jointFailures, framesDropped: pr.framesDropped, failedCurves: pr.failedCurves.length });
  if (pr.errors.length) pwhy = pr.errors.join('; ');
  else if (pr.jointFailures !== 1 || pr.framesDropped !== 1) pwhy = pr.jointFailures + ' joint failures, ' + pr.framesDropped + ' frames dropped, expected 1 and 1';
  else if (pr.failedCurves.length !== 2) pwhy = pr.failedCurves.length + ' failed curves, expected 2';
  else if (objs.length !== 1 || !objs.item(0).isSolidBRep) pwhy = 'expected one closed solid, got ' + objs.length + ' objects';
  else if (moi.filesystem.fileExists(tmp)) pwhy = 'temp file left behind';
  else {
    b = objs.item(0).getBoundingBox();
    pwhy = checkImport(cage.partial.box, [[b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z]], tol);
    if (!pwhy && b.max.x > 100) pwhy = 'the hairpin frame came through: max x ' + b.max.x;
  }
  if (objs.length) gd.removeObjects(objs);
  if (pwhy) out.failed.push({ scene: 'cubeframe + tight hairpin (partial build)', reason: pwhy }); else out.passed++;

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
  if (cage.report.errors.length || cage.report.jointFailures) why = cage.report.errors.concat(cage.report.jointFailures ? [cage.report.jointFailures + ' joint(s) could not be built'] : []).join('; ');
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

  // Cage surfaces on the same 300-strut cage (ticket 15): one batched planarsrf over 2830 curves measured 2.2 s
  // here, one call per face 50.8 s, so a generous ceiling still catches an unbatched implementation.
  var swhy = why ? 'the lattice cage failed: ' + why : '', sres = { scene: 'lattice (cage surfaces)' };
  out.results.push(sres);
  if (!swhy) {
    t = new Date().getTime();
    objs = buildCageSurfaces(cage);
    sres.ms = new Date().getTime() - t;
    sres.objects = objs.length;
    sres.faces = cage.faces.length;
    if (objs.length !== cage.faces.length) swhy = objs.length + ' surfaces, expected ' + cage.faces.length;
    else if (gd.getObjects().length !== before + objs.length) swhy = 'construction curves left in the document';
    else if (sres.ms > 15000) swhy = 'took ' + sres.ms + ' ms; planarsrf is probably not batched';
    if (objs.length) gd.removeObjects(objs);
  }
  if (swhy) out.failed.push({ scene: 'lattice (cage surfaces)', reason: swhy }); else out.passed++;

  // Cage solid on the same cage (ticket 16): surfaces plus one join, measured at 9.1 s, of which the join is 6.5 s.
  // The ceiling catches a lost planarsrf batching, which took this output to about a minute.
  var dwhy = why ? 'the lattice cage failed: ' + why : '', dres = { scene: 'lattice (cage solid)' };
  out.results.push(dres);
  if (!dwhy) {
    t = new Date().getTime();
    objs = buildCageSolid(cage);
    dres.ms = new Date().getTime() - t;
    dres.objects = objs.length;
    if (objs.length !== 1) dwhy = objs.length + ' objects, expected 1';
    else if (!objs.item(0).isSolidBRep) dwhy = 'the joined cage is not a closed solid';
    else if (gd.getObjects().length !== before + objs.length) dwhy = 'construction geometry left in the document';
    else if (dres.ms > 25000) dwhy = 'took ' + dres.ms + ' ms; planarsrf is probably not batched';
    if (objs.length) gd.removeObjects(objs);
  }
  if (dwhy) out.failed.push({ scene: 'lattice (cage solid)', reason: dwhy }); else out.passed++;

  if (gd.getObjects().length !== before) out.failed.push({ scene: '*', reason: 'document changed' });
  // The run can outlast the bridge's call timeout, so the result is left in a global for a follow-up call to read.
  MULTIPIPE2_SUITE = out;
  return out;
}
