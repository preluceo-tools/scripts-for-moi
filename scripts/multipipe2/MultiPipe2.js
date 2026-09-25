// config: norepeat
// MultiPipe2: turn selected curves into smooth pipe frames. Thin wrapper: the planner builds the cage,
// MoI's SubD import subdivides it into solids.
#include "MultiPipe2Planner.js"

function getCurves() {
  var curves = moi.geometryDatabase.getSelectedObjects().getStandaloneCurves();
  if (curves.length) return curves;
  var picker = moi.ui.createObjectPicker();
  picker.allowCurves();
  while (true) {
    if (!picker.waitForEvent()) return null;
    if (picker.event == 'finished') break;
    if (picker.event == 'done' && picker.done()) break;
  }
  return picker.objects.getStandaloneCurves();
}

// Waits for Done; false on Cancel.
function waitForDone() {
  while (true) {
    if (!moi.ui.commandDialog.waitForEvent()) return false;
    if (moi.ui.commandDialog.event == 'done') return true;
  }
}

function show(id, text) {
  var ui = moi.ui;
  ui.beginUIUpdate();
  ui.hideUI('SelectPrompt'); ui.hideUI('OptionsPrompt'); ui.hideUI('Options');
  ui.hideUI('BuildingPrompt'); ui.hideUI('SummaryPrompt');
  ui.showUI(id);
  if (id == 'Options') ui.showUI('OptionsPrompt');
  if (text !== undefined) { ui.commandUI.Summary.innerHTML = text; ui.showUI('Summary'); }
  ui.endUIUpdate();
}

function stop(message) { show('SummaryPrompt', message); waitForDone(); }

// Planner input: a line, a polyline when every segment is straight, and otherwise one entry per segment: a line when
// straight, a smooth curve when not, sampled at equal arc length (parameters found from a finer parameter sweep).
// ponytail: fixed 128 samples a segment; a curve winding round many turns wants a count from its length or turn.
// owners, when given, is filled in step with the entries: the curve object each entry came from, so a failed
// joint's planner indices map back to the objects to name and select.
function describe(curves, owners) {
  var input = [];
  function push(entry) { input.push(entry); if (owners) owners.push(owner); }
  function A(p) { return [p.x, p.y, p.z]; }
  function smooth(s) {
    var t0 = s.domainMin, t1 = s.domainMax, M = 1024, N = 128, p = [], acc = [0], out = [], j = 0, k;
    function at(u) { return A(s.evaluatePoint(t0 + (t1 - t0) * u / M)); }
    for (k = 0; k <= M; k++) {
      p.push(at(k));
      if (k) acc.push(acc[k - 1] + Math.sqrt(Math.pow(p[k][0] - p[k - 1][0], 2) + Math.pow(p[k][1] - p[k - 1][1], 2) + Math.pow(p[k][2] - p[k - 1][2], 2)));
    }
    for (k = 0; k <= N; k++) {
      var q = acc[M] * k / N;
      while (j < M - 1 && acc[j + 1] < q) j++;
      out.push(k === 0 ? p[0] : k === N ? p[M] : at(j + (acc[j + 1] > acc[j] ? (q - acc[j]) / (acc[j + 1] - acc[j]) : 0)));
    }
    return { kind: 'smooth', samples: out, startTangent: A(s.evaluateTangent(t0)), endTangent: A(s.evaluateTangent(t1)) };
  }
  for (var i = 0, owner; i < curves.length; i++) {
    var c = owner = curves.item(i), segs = c.getSubObjects(), pts = [A(c.evaluatePoint(c.domainMin))], j;
    if (c.isLine) { push({ kind: 'line', start: pts[0], end: A(c.evaluatePoint(c.domainMax)) }); continue; }
    for (j = 0; j < segs.length && segs.item(j).isLine; j++) pts.push(A(segs.item(j).evaluatePoint(segs.item(j).domainMax)));
    if (segs.length && j === segs.length) { push({ kind: 'polyline', points: pts }); continue; }
    if (!segs.length) { push(smooth(c)); continue; }
    for (j = 0; j < segs.length; j++) {
      var s = segs.item(j);
      if (s.isLine && s.getLength() <= moi.geometryDatabase.tolerance) continue;
      push(s.isLine ? { kind: 'line', start: A(s.evaluatePoint(s.domainMin)), end: A(s.evaluatePoint(s.domainMax)) } : smooth(s));
    }
  }
  return input;
}

// Runs fn, which adds objects to the document, and returns what it added. Both fileImportSubD and a factory's
// commit() add objects while returning nothing useful, so the new ones are found by id.
function addedBy(fn) {
  var gd = moi.geometryDatabase, before = {}, all = gd.getObjects(), added = gd.createObjectList(), i;
  for (i = 0; i < all.length; i++) before[all.item(i).id] = true;
  fn();
  all = gd.getObjects();
  for (i = 0; i < all.length; i++) if (!before[all.item(i).id]) added.addObject(all.item(i));
  return added;
}

// Writes the cage to a temp OBJ, imports it as SubD and deletes the file whatever happens.
function importCage(cage) {
  var fs = moi.filesystem, path = fs.getTempDir() + 'MultiPipe2-cage.obj';
  return addedBy(function () {
    try {
      var lines = objLines(cage), s = fs.openFileStream(path, 'w'), i;
      try { for (i = 0; i < lines.length; i++) s.writeLine(lines[i]); } finally { s.close(); }
      moi.geometryDatabase.fileImportSubD(path);
    } finally {
      if (fs.fileExists(path)) fs.deleteFile(path);
    }
  });
}

// One closed curve per cage face, of any vertex count, built straight from the planner's data: no file is written.
// The polyline factory closes the curve when its last point repeats its first.
function buildCageCurves(cage) {
  return addedBy(function () {
    var VM = moi.vectorMath, i, j;
    for (i = 0; i < cage.faces.length; i++) {
      var face = cage.faces[i], f = moi.command.createFactory('polyline');
      for (j = 0; j <= face.length; j++) {
        var v = cage.vertices[face[j % face.length]];
        f.createInput('point');
        f.setInput(f.numInputs - 1, VM.createPoint(v[0], v[1], v[2]));
      }
      f.commit();
    }
  });
}

function plural(n, word) { return n + ' ' + word + (n == 1 ? '' : 's'); }

// The input curves meeting a joint that could not be built: named and left selected so the user can see which
// ones defeated the command, and every other input curve deselected so only those stand out.
function markFailed(curves, owners, failed) {
  var bad = {}, i, obj;
  for (i = 0; i < failed.length; i++) bad[owners[failed[i]].id] = true;
  for (i = 0; i < curves.length; i++) {
    obj = curves.item(i);
    if (bad[obj.id]) { obj.name = 'MultiPipe2 failed'; obj.selected = true; }
    else obj.selected = false;
  }
}

function MultiPipe2() {
  var curves = getCurves();
  if (!curves) return;
  curves.lockSelection();

  show('Options');
  moi.ui.commandUI.divisions.disabled = moi.ui.commandUI.auto.value;
  moi.ui.commandUI.allNodes.disabled = !moi.ui.commandUI.roundJoints.value;
  if (!waitForDone()) return;

  // The count before planning is the input segments; duplicates the planner drops are still in it.
  var ui = moi.ui.commandUI, cap = ui.cap.value, output = ui.output.value, owners = [], input = describe(curves, owners), segments = 0, i;
  for (i = 0; i < input.length; i++) segments += input[i].kind == 'polyline' ? input[i].points.length - 1 : 1;
  show('BuildingPrompt', 'Building ' + plural(segments, 'strut') + '...');
  var cage = plan(input, { radius: ui.radius.value, nodeSize: ui.nodesize.value,
    divisions: ui.auto.value ? 'auto' : ui.divisions.value, cap: cap,
    roundJoints: ui.roundJoints.value, allNodes: ui.allNodes.value,
    tolerance: moi.geometryDatabase.tolerance });
  var r = cage.report, frame = output == 'frame', objs, notes = '';
  if (r.errors.length) { stop(r.errors.join('<br>')); return; }
  // A joint that could not be built is fatal to its own pipe frame only (the SubD import rejects the whole cage,
  // so the frame is dropped from the file); the rest are built, and a cage output draws everything.
  if (r.jointFailures) markFailed(curves, owners, r.failedCurves);
  var frames = frame ? r.pipeFrames - r.framesDropped : r.pipeFrames;
  var failures = r.jointFailures ? '<br>' + plural(r.jointFailures, 'joint') + ' could not be built; ' +
    (frame ? plural(r.framesDropped, 'pipe frame') + ' dropped. ' : '') +
    'The curves there are named MultiPipe2 failed and selected.' : '';

  if (frame) {
    var build = r.jointFailures ? cage.partial : cage;
    if (!build.faces.length) { stop('No pipe frame could be built: every one of them has a joint whose struts meet at too tight an angle.' + failures); return; }
    objs = importCage(build);
    var boxes = [];
    if (!objs.length) { stop('The SubD import produced nothing, so nothing was added.'); return; }
    for (i = 0; i < objs.length; i++) {
      var b = objs.item(i).getBoundingBox();
      boxes.push([b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z]);
    }
    var bad = checkImport(build.box, boxes, moi.geometryDatabase.tolerance);
    if (bad) { moi.geometryDatabase.removeObjects(objs); stop(bad); return; }
  } else {
    // Cage surfaces and Cage solid are not built yet; they give the curves so no value in the option does nothing.
    objs = buildCageCurves(cage);
    notes = plural(objs.length, 'cage curve') + ' for ' + plural(cage.faces.length, 'cage face') + '.<br>' +
      (output == 'curves' ? '' : 'Cage surfaces and Cage solid are not available yet, so the cage curves were added instead.<br>');
  }
  for (i = 0; i < objs.length; i++) objs.item(i).name = '';

  show('SummaryPrompt', notes + plural(frames, 'pipe frame') + ', ' + plural(r.struts, 'strut') + ', ' +
    plural(r.nodes, 'node') + ', ' + plural(r.freeEnds, 'free end') +
    (r.duplicatesDropped ? '<br>' + plural(r.duplicatesDropped, 'duplicate segment') + ' dropped.' : '') +
    (r.crossings ? '<br>' + plural(r.crossings, 'crossing') + ' left unjoined; split the curves there to make a node.<br>' +
      r.warnings.join('<br>') : '') +
    (r.grownNodes ? '<br>' + plural(r.grownNodes, 'node') + ' grew for tight angles, reaching up to ' +
      r.largestReach.toFixed(2) + ' x Radius.' : '') +
    (r.shortStruts ? '<br>' + plural(r.shortStruts, 'strut') + ' shorter than ' + (r.shortStruts == 1 ? 'its' : 'their') +
      ' joints; the frame may intersect itself there.' : '') +
    (r.roundedNodes ? '<br>' + plural(r.roundedNodes, 'node') + ' came to a miter point.' : '') +
    (r.roundedNodeFallbacks ? '<br>' + plural(r.roundedNodeFallbacks, 'node') + ' could not and kept the usual joint.' : '') +
    failures +
    (!cap && r.freeEnds ? '<br>Cap is off: ' + plural(r.freeEnds, 'free end') + ' left open, so the result is an open surface, not a solid.' : ''));
  if (!waitForDone()) moi.geometryDatabase.removeObjects(objs);
}

MultiPipe2();
