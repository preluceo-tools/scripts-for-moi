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
// solid. The 'N=3' runs repeat some with Divisions 3 and must still come in as closed solids.
// Returns { passed, failed: [ { scene, reason } ], results: [ { scene, ms, objects } ] }.

function runMoiSuite(root) {
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
  var names = ['line', 'bend90', 'cubeframe', 'twobends', 'hairpin30', 'k5skew', 'd8', 'roofTruss', 'polyframe'], runs = [], n;
  for (n = 0; n < names.length; n++) runs.push({ name: names[n], cap: true });
  runs.push({ name: 'line', cap: false }, { name: 'bend90', cap: false }, { name: 'polyframe', cap: false }, { name: 'cubeframe', cap: false });
  runs.push({ name: 'cubeframe', cap: true, divisions: 3 }, { name: 'polyframe', cap: true, divisions: 3 });
  var VM = moi.vectorMath;
  function curve(c) {
    var f = moi.command.createFactory('polyline');
    for (var j = 0; j < c.pts.length; j++) { f.createInput('point'); f.setInput(f.numInputs - 1, VM.createPoint(c.pts[j][0], c.pts[j][1], c.pts[j][2])); }
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
    var name = runs[n].name, label = name + (runs[n].cap ? '' : ' (cap off)') + (runs[n].divisions ? ' (N=' + runs[n].divisions + ')' : ''), spec = scenes[name], curves = gd.createObjectList(), i, reason = '';
    for (i = 0; i < spec.length; i++) curves.addObject(curve(spec[i]));
    var input = describe(curves);
    var cage = plan(input, { radius: R, nodeSize: 1.6, divisions: runs[n].divisions, cap: runs[n].cap, tolerance: tol }), t0 = new Date().getTime();
    var objs = cage.report.errors.length ? gd.createObjectList() : importCage(cage), boxes = [], nakedLoops = 0;
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
    if (cage.report.errors.length) reason = cage.report.errors.join('; ');
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
    if (objs.length) gd.removeObjects(objs);
    if (reason) out.failed.push({ scene: label, reason: reason }); else out.passed++;
  }
  if (gd.getObjects().length !== before) out.failed.push({ scene: '*', reason: 'document changed' });
  return out;
}
