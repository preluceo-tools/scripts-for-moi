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

// Planner input: a line, or a polyline when every segment is straight. Anything else reaches the planner as an
// unsupported kind, and it says so.
function describe(curves) {
  var input = [];
  function A(p) { return [p.x, p.y, p.z]; }
  for (var i = 0; i < curves.length; i++) {
    var c = curves.item(i), segs = c.getSubObjects(), pts = [A(c.evaluatePoint(c.domainMin))], j;
    if (c.isLine) { input.push({ kind: 'line', start: pts[0], end: A(c.evaluatePoint(c.domainMax)) }); continue; }
    for (j = 0; j < segs.length && segs.item(j).isLine; j++) pts.push(A(segs.item(j).evaluatePoint(segs.item(j).domainMax)));
    input.push(segs.length && j === segs.length ? { kind: 'polyline', points: pts } : { kind: 'curve' });
  }
  return input;
}

// Writes the cage to a temp OBJ, imports it as SubD and deletes the file whatever happens.
// fileImportSubD adds to the document and returns nothing, so the new objects are found by id.
function importCage(cage) {
  var gd = moi.geometryDatabase, fs = moi.filesystem, path = fs.getTempDir() + 'MultiPipe2-cage.obj';
  var before = {}, all = gd.getObjects(), added = gd.createObjectList(), i;
  for (i = 0; i < all.length; i++) before[all.item(i).id] = true;
  try {
    var lines = objLines(cage), s = fs.openFileStream(path, 'w');
    try { for (i = 0; i < lines.length; i++) s.writeLine(lines[i]); } finally { s.close(); }
    gd.fileImportSubD(path);
  } finally {
    if (fs.fileExists(path)) fs.deleteFile(path);
  }
  all = gd.getObjects();
  for (i = 0; i < all.length; i++) if (!before[all.item(i).id]) added.addObject(all.item(i));
  return added;
}

function plural(n, word) { return n + ' ' + word + (n == 1 ? '' : 's'); }

function MultiPipe2() {
  var curves = getCurves();
  if (!curves) return;
  curves.lockSelection();

  show('Options');
  if (!waitForDone()) return;

  var ui = moi.ui.commandUI, cap = ui.cap.value;
  var cage = plan(describe(curves), { radius: ui.radius.value, nodeSize: ui.nodesize.value, cap: cap,
    tolerance: moi.geometryDatabase.tolerance });
  var r = cage.report;
  if (r.errors.length) { stop(r.errors.join('<br>')); return; }

  show('BuildingPrompt', 'Building ' + plural(r.struts, 'strut') + '...');
  var objs = importCage(cage), boxes = [], i;
  if (!objs.length) { stop('The SubD import produced nothing, so nothing was added.'); return; }
  for (i = 0; i < objs.length; i++) {
    var b = objs.item(i).getBoundingBox();
    boxes.push([b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z]);
  }
  var bad = checkImport(cage.box, boxes, moi.geometryDatabase.tolerance);
  if (bad) { moi.geometryDatabase.removeObjects(objs); stop(bad); return; }
  for (i = 0; i < objs.length; i++) objs.item(i).name = '';

  show('SummaryPrompt', plural(r.pipeFrames, 'pipe frame') + ', ' + plural(r.struts, 'strut') + ', ' +
    plural(r.nodes, 'node') + ', ' + plural(r.freeEnds, 'free end') +
    (r.duplicatesDropped ? '<br>' + plural(r.duplicatesDropped, 'duplicate segment') + ' dropped.' : '') +
    (r.crossings ? '<br>' + plural(r.crossings, 'crossing') + ' left unjoined; split the curves there to make a node.<br>' +
      r.warnings.join('<br>') : '') +
    (r.grownNodes ? '<br>' + plural(r.grownNodes, 'node') + ' grew for tight angles, reaching up to ' +
      r.largestReach.toFixed(2) + ' x Radius.' : '') +
    (r.shortStruts ? '<br>' + plural(r.shortStruts, 'strut') + ' shorter than ' + (r.shortStruts == 1 ? 'its' : 'their') +
      ' joints; the frame may intersect itself there.' : '') +
    (!cap && r.freeEnds ? '<br>Cap is off: ' + plural(r.freeEnds, 'free end') + ' left open, so the result is an open surface, not a solid.' : ''));
  if (!waitForDone()) moi.geometryDatabase.removeObjects(objs);
}

MultiPipe2();
