// config: norepeat
// MultiPipe: turn selected curves into one pipe frame. Thin wrapper around the planner and builder.
#include "MultiPipePlanner.js"
#include "MultiPipeBuilder.js"

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
  // The fillet factor row starts hidden, so a remembered Filleted choice has to show it again.
  if (id == 'Options' && ui.commandUI.filleted.value) ui.showUI('FilletRow');
  if (text !== undefined) { ui.commandUI.Summary.innerHTML = text; ui.showUI('Summary'); }
  ui.endUIUpdate();
}

function MultiPipe() {
  var curves = getCurves();
  if (!curves) return;
  curves.lockSelection();

  show('Options');
  if (!waitForDone()) return;

  var d = describe(curves), i;
  var p = plan(d.input, {
    radius: moi.ui.commandUI.radius.value,
    nodeSize: moi.ui.commandUI.nodesize.value,
    jointStyle: moi.ui.commandUI.filleted.value ? 'filleted' : 'ball',
    filletFactor: moi.ui.commandUI.filletfactor.value,
    cap: moi.ui.commandUI.cap.value,
    tolerance: moi.geometryDatabase.tolerance
  });
  if (p.errors.length) { show('SummaryPrompt', p.errors.join('<br>')); waitForDone(); return; }

  // A large frame spends a long time in the union with no other sign of life, so say what is happening first.
  show('BuildingPrompt', 'Building ' + p.rails.length + ' strut' + (p.rails.length > 1 ? 's' : '') + '...');

  var result = build(p, d.objects, {});
  var r = result.report;
  if (r.errors.length) { show('SummaryPrompt', r.errors.join('<br>')); waitForDone(); return; }

  show('SummaryPrompt', r.struts + ' struts, ' + r.joints + ' joints, ' + r.straightNodes + ' straight nodes, ' + r.freeEnds + ' free ends' +
    (r.duplicatesDropped ? '<br>' + r.duplicatesDropped + ' duplicate curve segments dropped.' : '') +
    (r.crossings ? '<br>' + r.crossings + ' crossing' + (r.crossings > 1 ? 's' : '') + ' left unjoined.' : '') +
    (r.union == 'retry' ? '<br>Joined by the slower step-by-step union.' : '') +
    (r.cap == 'open' ? '<br>Free ends left open.' : '') +
    (r.warnings.length ? '<br>' + r.warnings.join('<br>') : ''));
  if (!waitForDone()) return;
  for (i = 0; i < result.objects.length; i++) moi.geometryDatabase.addObject(result.objects[i]);
}

MultiPipe();
