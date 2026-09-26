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

// Waits for Done; true on Done, 'back' on the Back button, false on Cancel.
function waitForDone() {
  while (true) {
    if (!moi.ui.commandDialog.waitForEvent()) return false;
    if (moi.ui.commandDialog.event == 'done') return true;
    if (moi.ui.commandDialog.event == 'back') return 'back';
  }
}

function show(id, text) {
  var ui = moi.ui;
  ui.beginUIUpdate();
  ui.hideUI('SelectPrompt'); ui.hideUI('OptionsPrompt'); ui.hideUI('Options');
  ui.hideUI('BuildingPrompt'); ui.hideUI('SummaryPrompt'); ui.hideUI('Back');
  ui.showUI(id);
  if (id == 'Options') ui.showUI('OptionsPrompt');
  if (id == 'SummaryPrompt') ui.showUI('Back');
  if (text !== undefined) { ui.commandUI.Summary.innerHTML = text; ui.showUI('Summary'); }
  ui.endUIUpdate();
}

// Shows a message and ends the pass on whatever the user presses there: Done, Back or Cancel.
function stop(message) { show('SummaryPrompt', message); return waitForDone(); }

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

// One surface per cage face, from the cage curves. planarsrf is called once over all of them: one call
// per face costs about 23x more, and the per-call cost grows as the document fills. Its commit() returns a
// falsy value while succeeding, so what it made is counted by diffing the document, as addedBy does. The curves
// are construction geometry and planarsrf does not consume them, so they are removed here.
function buildCageSurfaces(cage) {
  var curves = buildCageCurves(cage);
  var surfaces = addedBy(function () {
    var f = moi.command.createFactory('planarsrf');
    f.setInput(0, curves);
    f.commit();
  });
  moi.geometryDatabase.removeObjects(curves);
  return surfaces;
}

// The cage surfaces joined into one solid per pipe frame. join consumes the surfaces it merges and only merges the
// ones that share edges, so several pipe frames come out as several solids with no grouping needed. It is the slow
// half of this output and, unlike planarsrf, is already one call over the whole list, so there is nothing to batch.
// The result is a closed solid only when the cage is closed: with Cap off it is an open surface, as the summary says.
function buildCageSolid(cage) {
  var surfaces = buildCageSurfaces(cage);
  return addedBy(function () {
    var f = moi.command.createFactory('join');
    f.setInput(0, surfaces);
    f.commit();
  });
}

function plural(n, word) { return n + ' ' + word + (n == 1 ? '' : 's'); }

// Radius per curve, carried by the curve's MoI style. The distinct styles in the locked selection, in the order
// they are first met: row n of the options panel belongs to the n-th distinct style, so a style the user added at
// any index needs no reserved row and the cap is on how many styles one run mixes, not on the style index.
// MAX_STYLES must match the number of style rows declared in MultiPipe2.htm.
var MAX_STYLES = 8;
function distinctStyles(curves) {
  var seen = {}, out = [], i, s;
  for (i = 0; i < curves.length; i++) {
    s = curves.item(i).styleIndex;
    if (!(s in seen)) { seen[s] = true; out.push(s); }
  }
  return out;
}
// A style name is whatever the user typed in Edit styles, and it reaches the panel and the summary as HTML.
function escapeHTML(s) { return ('' + s).split('&').join('&amp;').split('<').join('&lt;').split('>').join('&gt;'); }
function styleNames(styles) {
  var all = moi.geometryDatabase.getObjectStyles(), out = [], i;
  for (i = 0; i < styles.length; i++) out.push(styles[i] >= 0 && styles[i] < all.length ? all.item(styles[i]).name : 'Style ' + styles[i]);
  return out;
}
// The per-style rows, revealed only when the selection spans two or more styles, so a selection on one style shows
// exactly the panel it always did. seed writes the single Radius value into each row, done once a run: on the way
// back from the summary the rows keep whatever the user typed.
function showStyleRows(styles, names, seed) {
  var ui = moi.ui, cui = ui.commandUI, i;
  if (styles.length < 2) return;
  ui.hideUI('radiustr');
  for (i = 0; i < styles.length; i++) {
    ui.showUI('style' + (i + 1) + 'tr');
    cui['style' + (i + 1) + 'name'].innerHTML = escapeHTML(names[i]) + ':';
    if (seed) cui['style' + (i + 1) + 'radius'].value = cui.radius.value;
  }
}
// One radius per input entry, from the row its curve's style owns; null when the selection is on one style, and the
// single Radius is used as before. Row values at or below zero are reported by style name, which the planner cannot
// do because it never sees a style.
function styleRadii(owners, styles, names, errors) {
  var cui = moi.ui.commandUI, byStyle = {}, radii = [], i;
  if (styles.length < 2) return null;
  for (i = 0; i < styles.length; i++) {
    var v = cui['style' + (i + 1) + 'radius'].value;
    if (!(v > 0)) errors.push('Radius for ' + escapeHTML(names[i]) + ' must be greater than zero.');
    byStyle[styles[i]] = v;
  }
  for (i = 0; i < owners.length; i++) radii.push(byStyle[owners[i].styleIndex]);
  return radii;
}

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

// One pass over the options, the build and the summary. Returns what the user ended it on: true for Done,
// 'back' for the Back button, false for Cancel. Anything the pass added is removed unless it ended on Done.
function pass(curves, styles, names, seed) {
  show('Options');
  showStyleRows(styles, names, seed);
  moi.ui.commandUI.divisions.disabled = moi.ui.commandUI.auto.value;
  moi.ui.commandUI.allNodes.disabled = !moi.ui.commandUI.roundJoints.value;
  if (waitForDone() !== true) return false;   // Back is hidden on the options step, so this is Cancel.

  // The count before planning is the input segments; duplicates the planner drops are still in it.
  var ui = moi.ui.commandUI, cap = ui.cap.value, output = ui.output.value, owners = [], input = describe(curves, owners), segments = 0, i;
  for (i = 0; i < input.length; i++) segments += input[i].kind == 'polyline' ? input[i].points.length - 1 : 1;
  var rowErrors = [], radii = styleRadii(owners, styles, names, rowErrors);
  if (rowErrors.length) return stop(rowErrors.join('<br>'));
  show('BuildingPrompt', 'Building ' + plural(segments, 'strut') + '...');
  // The single Radius is hidden while the per-style rows are up, so it must not be what the planner validates:
  // a zero left in it by an earlier run would be an error about a field the user cannot see. On a multi-style run
  // the first row stands in for it, and every strut has its own radius anyway.
  var fallback = radii && radii.length ? radii[0] : ui.radius.value;
  var cage = plan(input, { radius: fallback, radii: radii, nodeSize: ui.nodesize.value,
    divisions: ui.auto.value ? 'auto' : ui.divisions.value, cap: cap,
    roundJoints: ui.roundJoints.value, allNodes: ui.allNodes.value,
    tolerance: moi.geometryDatabase.tolerance });
  var r = cage.report, frame = output == 'frame', objs, notes = '';
  if (r.errors.length) return stop(r.errors.join('<br>'));
  // The radii actually built, for the summary; only worth saying when they differ.
  var used = [];
  for (i = 0; radii && i < radii.length; i++) { var seen = false, m; for (m = 0; m < used.length; m++) if (used[m] == radii[i]) seen = true; if (!seen) used.push(radii[i]); }
  used.sort(function (a, b) { return a - b; });
  // A joint that could not be built is fatal to its own pipe frame only (the SubD import rejects the whole cage,
  // so the frame is dropped from the file); the rest are built, and a cage output draws everything.
  if (r.jointFailures) markFailed(curves, owners, r.failedCurves);
  var frames = frame ? r.pipeFrames - r.framesDropped : r.pipeFrames;
  var failures = r.jointFailures ? '<br>' + plural(r.jointFailures, 'joint') + ' could not be built; ' +
    (frame ? plural(r.framesDropped, 'pipe frame') + ' dropped. ' : '') +
    'The curves there are named MultiPipe2 failed and selected.' : '';

  if (frame) {
    var build = r.jointFailures ? cage.partial : cage;
    if (!build.faces.length) return stop('No pipe frame could be built: every one of them has a joint whose struts meet at too tight an angle.' + failures);
    objs = importCage(build);
    var boxes = [];
    if (!objs.length) return stop('The SubD import produced nothing, so nothing was added.');
    for (i = 0; i < objs.length; i++) {
      var b = objs.item(i).getBoundingBox();
      boxes.push([b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z]);
    }
    var bad = checkImport(build.box, boxes, moi.geometryDatabase.tolerance);
    if (bad) { moi.geometryDatabase.removeObjects(objs); return stop(bad); }
  } else if (output == 'surfaces') {
    objs = buildCageSurfaces(cage);
    notes = plural(objs.length, 'cage surface') + ' for ' + plural(cage.faces.length, 'cage face') + '.<br>';
  } else if (output == 'solid') {
    objs = buildCageSolid(cage);
    notes = plural(objs.length, 'cage solid') + ' from ' + plural(cage.faces.length, 'cage face') + '.<br>';
  } else {
    objs = buildCageCurves(cage);
    notes = plural(objs.length, 'cage curve') + ' for ' + plural(cage.faces.length, 'cage face') + '.<br>';
  }
  for (i = 0; i < objs.length; i++) objs.item(i).name = '';

  show('SummaryPrompt', notes + plural(frames, 'pipe frame') + ', ' + plural(r.struts, 'strut') + ', ' +
    plural(r.nodes, 'node') + ', ' + plural(r.freeEnds, 'free end') +
    (used.length > 1 ? '<br>Radii built: ' + used.join(', ') + '.' : '') +
    (r.duplicatesDropped ? '<br>' + plural(r.duplicatesDropped, 'duplicate segment') + ' dropped.' +
      (r.duplicateRadii ? ' ' + (r.duplicateRadii == 1 ? 'One of them carried' : r.duplicateRadii + ' of them carried') +
        ' a different radius from the segment kept, so the thickness there follows the one selected first.' : '') : '') +
    (r.crossings ? '<br>' + plural(r.crossings, 'crossing') + ' left unjoined; split the curves there to make a node.<br>' +
      r.warnings.join('<br>') : '') +
    (r.grownNodes ? '<br>' + plural(r.grownNodes, 'node') + ' grew for tight angles, reaching up to ' +
      r.largestReach.toFixed(2) + ' x the largest radius at the node.' : '') +
    (r.shortStruts ? '<br>' + plural(r.shortStruts, 'strut') + ' shorter than ' + (r.shortStruts == 1 ? 'its' : 'their') +
      ' joints; the frame may intersect itself there.' : '') +
    (r.roundedNodes ? '<br>' + plural(r.roundedNodes, 'node') + ' came to a miter point.' : '') +
    (r.roundedNodeFallbacks ? '<br>' + plural(r.roundedNodeFallbacks, 'node') + ' could not and kept the usual joint.' : '') +
    failures +
    (!cap && r.freeEnds ? '<br>Cap is off: ' + plural(r.freeEnds, 'free end') + ' left open, so the result is an open surface, not a solid.' : ''));
  var end = waitForDone();
  if (end !== true) moi.geometryDatabase.removeObjects(objs);
  return end;
}

function MultiPipe2() {
  var curves = getCurves();
  if (!curves) return;
  curves.lockSelection();
  // What the input curves looked like before the run, so Back and Cancel can put back the names and the selection
  // markFailed changes. The selection itself is held, so Back never asks for the curves again.
  var was = [], i;
  for (i = 0; i < curves.length; i++) was.push([curves.item(i).name, curves.item(i).selected]);
  // The selection is locked for the run, so its styles are fixed: found once, not again when Back returns.
  var styles = distinctStyles(curves);
  if (styles.length > MAX_STYLES) {
    stop('The selection spans ' + styles.length + ' styles; MultiPipe2 takes at most ' + MAX_STYLES +
      ' in one run, one radius each. Run it on fewer styles at a time.');
    return;
  }
  var names = styleNames(styles), seed = true, end;
  do {
    end = pass(curves, styles, names, seed);
    seed = false;
    if (end === true) return;
    for (i = 0; i < curves.length; i++) { var c = curves.item(i); c.name = was[i][0]; c.selected = was[i][1]; }
  } while (end === 'back');
}

MultiPipe2();
