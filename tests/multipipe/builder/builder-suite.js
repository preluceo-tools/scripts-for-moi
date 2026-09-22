// Builder suite, run inside MoI through the MoI MCP bridge (moi_eval). ES5 only.
// It loads the planner and builder fresh from <repo>, makes each scene's curves and builds it with calculate() only,
// and adds nothing to the document. Call it with a moi_eval script like:
//
//   var ROOT = '<repo folder>/';   // forward slashes, trailing slash
//   var s = moi.filesystem.openFileStream(ROOT + 'tests/multipipe/builder/builder-suite.js', 'r'), t = '';
//   while (!s.AtEOF) t += s.readLine() + '\n';
//   s.close(); (0, eval)(t);
//   return runBuilderSuite(ROOT);
//
// Returns { passed, failed: [ { scene, reason } ], results }.
//
// The 300-strut lattice takes about 40 s, so the whole run is well past the bridge's 30 s call timeout.
// The timed-out call keeps running inside MoI and leaves the result in the global MULTIPIPE_SUITE; read it
// back with a follow-up moi_eval of:
//
//   return typeof MULTIPIPE_SUITE == 'undefined' ? 'still running' : MULTIPIPE_SUITE;

function runBuilderSuite(root) {
  MULTIPIPE_SUITE = undefined;   // so a follow-up read cannot return an earlier run's result
  function read(path) {
    var s = moi.filesystem.openFileStream(root + path, 'r'), t = '';
    while (!s.AtEOF) t += s.readLine() + '\n';
    s.close();
    return t;
  }
  (0, eval)(read('scripts/multipipe/MultiPipePlanner.js'));
  (0, eval)(read('scripts/multipipe/MultiPipeBuilder.js'));
  var scenes = eval('(' + read('tests/multipipe/scenes/scenes.json') + ')');

  var names = ['y', 't', 'node4', 'node4x50', 'node5', 'node6', 'node6tight', 'acute', 'freeends', 'polyline', 'straight',
    'smooth', 'circle', 'ring', 'duplicates', 'crossing'];
  var VM = moi.vectorMath;
  function P(a) { return VM.createPoint(a[0], a[1], a[2]); }
  // Scene curves as MoI curves, made with calculate() only.
  function curve(c) {
    var f = moi.command.createFactory(c.type === 'interp' ? 'interpcurve' : c.type === 'circle' ? 'circle' : 'polyline');
    if (c.type === 'circle') {
      f.setInput(0, true); f.setInput(1, VM.createFrame(P(c.center), P([1, 0, 0]), P([0, 1, 0]))); f.setInput(3, c.radius);
    } else {
      for (var j = 0; j < c.pts.length; j++) {
        f.createInput('point'); f.setInput(f.numInputs - 1, P(c.pts[j]));
        if (c.type === 'interp') { f.createInput('bool'); f.setInput(f.numInputs - 1, false); }
      }
    }
    var r = f.calculate(); f.cancel();
    return r.item(0);
  }
  // A generated scene gives its grid instead of a curve list: n cells per side, spacing s, axis-aligned struts.
  function gridSpec(n, s) {
    var spec = [], x, y, z;
    for (x = 0; x <= n; x++) for (y = 0; y <= n; y++) for (z = 0; z < n; z++) {
      spec.push({ type: 'line', pts: [[x * s, y * s, z * s], [x * s, y * s, (z + 1) * s]] });
      spec.push({ type: 'line', pts: [[x * s, z * s, y * s], [x * s, (z + 1) * s, y * s]] });
      spec.push({ type: 'line', pts: [[z * s, x * s, y * s], [(z + 1) * s, x * s, y * s]] });
    }
    return spec;
  }
  function sceneCurves(name) {
    var spec = scenes[name], curves = moi.geometryDatabase.createObjectList();
    if (spec.grid) spec = gridSpec(spec.grid.n, spec.grid.s);
    for (var i = 0; i < spec.length; i++) curves.addObject(curve(spec[i]));
    return curves;
  }
  // Expected union path per scene; scenes not listed must not fall back to separate pieces.
  var paths = { node4x50: 'retry' };
  // Scenes whose plan must report dropped duplicates or crossings.
  var counts = { duplicates: { duplicatesDropped: 2, crossings: 0 }, crossing: { duplicatesDropped: 0, crossings: 1 } };
  var before = moi.geometryDatabase.getObjects().length;
  var out = { passed: 0, failed: [], results: [] };
  for (var n = 0; n < names.length; n++) {
    var curves = sceneCurves(names[n]), lo = [1e30, 1e30, 1e30], hi = [-1e30, -1e30, -1e30], i;
    var d = describe(curves);
    if (names[n] === 'node4') var forced = curves;
    // Curve bounding boxes follow the control points, so coverage is checked on points along the curves.
    for (i = 0; i < d.input.length; i++) {
      var pts = d.input[i].samples || [d.input[i].start, d.input[i].end];
      for (var j = 0; j < pts.length; j++) for (var k = 0; k < 3; k++) {
        lo[k] = Math.min(lo[k], pts[j][k]); hi[k] = Math.max(hi[k], pts[j][k]);
      }
    }
    var p = plan(d.input, { radius: 0.5, nodeSize: 1.2, tolerance: moi.geometryDatabase.tolerance });
    var reason = p.errors.join('; '), want = counts[names[n]];
    if (want && (p.duplicatesDropped !== want.duplicatesDropped || p.crossings.length !== want.crossings)) {
      reason = 'expected ' + want.duplicatesDropped + ' duplicates and ' + want.crossings + ' crossings, got ' +
        p.duplicatesDropped + ' and ' + p.crossings.length;
    }
    if (!reason) {
      var r = build(p, d.objects, {});
      out.results.push({ scene: names[n], report: r.report, count: r.objects.length });
      if (want && (r.report.duplicatesDropped !== want.duplicatesDropped || r.report.crossings !== want.crossings)) {
        reason = 'report says ' + r.report.duplicatesDropped + ' duplicates and ' + r.report.crossings + ' crossings';
      }
      else if (paths[names[n]] && r.report.union !== paths[names[n]]) reason = 'union path ' + r.report.union + ', expected ' + paths[names[n]];
      else if (r.report.errors.length) reason = r.report.errors.join('; ');
      else if (r.objects.length !== 1 || !r.objects[0].isSolidBRep) reason = 'expected one solid, got ' + r.objects.length + ' objects';
      else {
        // The solid's bounding box must cover every point along the input curves.
        var b = r.objects[0].getBoundingBox(), bl = [b.min.x, b.min.y, b.min.z], bh = [b.max.x, b.max.y, b.max.z];
        for (k = 0; k < 3; k++) if (bl[k] > lo[k] || bh[k] < hi[k]) reason = 'bounding box misses a strut';
      }
    }
    if (reason) out.failed.push({ scene: names[n], reason: reason }); else out.passed++;
  }
  // Filleted joints at the default factor 0.2: these scenes fillet, node4 falls back to ball joints with a warning.
  var fillets = [['y', 'filleted'], ['node6', 'filleted'], ['smooth', 'filleted'], ['node4', 'fallback']];
  for (n = 0; n < fillets.length; n++) {
    var name = fillets[n][0], wantFillet = fillets[n][1];
    d = describe(sceneCurves(name));
    p = plan(d.input, { radius: 0.5, nodeSize: 1.2, jointStyle: 'filleted', filletFactor: 0.2,
      tolerance: moi.geometryDatabase.tolerance });
    r = build(p, d.objects, {});
    out.results.push({ scene: name + '-filleted', report: r.report, count: r.objects.length });
    var warned = r.report.warnings.join(' ').indexOf('kept its ball joints') >= 0;
    reason = '';
    if (r.report.fillet !== wantFillet) reason = 'fillet ' + r.report.fillet + ', expected ' + wantFillet;
    else if (r.objects.length !== 1 || !r.objects[0].isSolidBRep) reason = 'expected one solid, got ' + r.objects.length + ' objects';
    else if (wantFillet === 'fallback' && !warned) reason = 'no fallback warning';
    else if (wantFillet === 'filleted' && warned) reason = 'filleted but warned about a fallback';
    if (reason) out.failed.push({ scene: name + '-filleted', reason: reason }); else out.passed++;
  }

  // Cap off: a scene with free ends gives one open body with no cap left at any free end; a closed ring has
  // no free ends, so the option changes nothing.
  var caps = [['freeends', 2], ['y', 3], ['circle', 0]];
  for (n = 0; n < caps.length; n++) {
    var cname = caps[n][0], wantEnds = caps[n][1];
    d = describe(sceneCurves(cname));
    p = plan(d.input, { radius: 0.5, nodeSize: 1.2, cap: false, tolerance: moi.geometryDatabase.tolerance });
    r = build(p, d.objects, {});
    out.results.push({ scene: cname + '-open', report: r.report, count: r.objects.length });
    reason = '';
    if (p.freeEnds.length !== wantEnds) reason = 'expected ' + wantEnds + ' free ends, got ' + p.freeEnds.length;
    else if (r.report.errors.length) reason = r.report.errors.join('; ');
    else if (r.objects.length !== 1) reason = 'expected one body, got ' + r.objects.length + ' objects';
    else if (wantEnds === 0) {
      if (r.report.cap !== undefined || !r.objects[0].isSolidBRep) reason = 'no free ends, so Cap off must change nothing';
    } else if (r.report.cap !== 'open' || r.objects[0].isSolidBRep) {
      reason = 'expected one open body, got cap ' + r.report.cap + ' solid ' + r.objects[0].isSolidBRep;
    } else {
      var fs = r.objects[0].getFaces();
      for (i = 0; i < fs.length; i++) {
        var fc = fs.item(i);
        if (!fc.isPlanar) continue;
        var fb = fc.getBoundingBox(), fcc = [(fb.min.x + fb.max.x) / 2, (fb.min.y + fb.max.y) / 2, (fb.min.z + fb.max.z) / 2];
        for (k = 0; k < p.freeEnds.length; k++) {
          var q = p.freeEnds[k].point;
          if (Math.sqrt((fcc[0] - q[0]) * (fcc[0] - q[0]) + (fcc[1] - q[1]) * (fcc[1] - q[1]) +
                        (fcc[2] - q[2]) * (fcc[2] - q[2])) <= 0.5) reason = 'a planar face is left at a free end';
        }
      }
    }
    if (reason) out.failed.push({ scene: cname + '-open', reason: reason }); else out.passed++;
  }

  // Forced failure: a union that returns nothing must give the separate struts and joints plus a warning.
  // With filleted joints it must also report the fillet fallback: there is no single body to fillet.
  for (n = 0; n < 2; n++) {
    var filleted = n === 1, label = 'forced-failure' + (filleted ? '-filleted' : '');
    d = describe(forced);
    p = plan(d.input, { radius: 0.5, nodeSize: 1.2, jointStyle: filleted ? 'filleted' : 'ball', filletFactor: 0.2,
      tolerance: moi.geometryDatabase.tolerance });
    r = build(p, d.objects, { union: function () { return []; } });
    out.results.push({ scene: label, report: r.report, count: r.objects.length });
    var text = r.report.warnings.join(' ');
    reason = '';
    if (r.report.union !== 'separate' || r.objects.length !== p.rails.length + p.joints.length ||
        text.indexOf('even after a retry') < 0) {
      reason = 'expected separate pieces with a warning, got ' + r.report.union;
    } else if (filleted && (r.report.fillet !== 'fallback' || text.indexOf('kept its ball joints') < 0)) {
      reason = 'expected the fillet fallback reported, got ' + r.report.fillet;
    } else if (!filleted && r.report.fillet !== undefined) {
      reason = 'ball joints must not report a fillet, got ' + r.report.fillet;
    }
    if (reason) out.failed.push({ scene: label, reason: reason }); else out.passed++;
  }
  // Large frame: the generated 4 x 4 x 4 lattice, 300 struts and 125 joints, with ball joints, under a minute.
  d = describe(sceneCurves('lattice'));
  p = plan(d.input, { radius: 0.5, nodeSize: 1.2, tolerance: moi.geometryDatabase.tolerance });
  r = build(p, d.objects, {});
  out.results.push({ scene: 'lattice', report: r.report, count: r.objects.length });
  reason = '';
  if (p.errors.length) reason = p.errors.join('; ');
  else if (p.rails.length !== 300 || p.joints.length !== 125) {
    reason = 'expected 300 struts and 125 joints, got ' + p.rails.length + ' and ' + p.joints.length;
  } else if (r.report.errors.length) reason = r.report.errors.join('; ');
  else if (r.objects.length !== 1 || !r.objects[0].isSolidBRep) reason = 'expected one solid, got ' + r.objects.length + ' objects';
  else if (r.report.ms > 60000) reason = 'took ' + r.report.ms + ' ms, over the 60000 ms limit';
  if (reason) out.failed.push({ scene: 'lattice', reason: reason }); else out.passed++;

  if (moi.geometryDatabase.getObjects().length !== before) out.failed.push({ scene: '*', reason: 'document changed' });
  // The run outlasts the bridge's call timeout, so the result is left in a global for a follow-up call to read.
  MULTIPIPE_SUITE = out;
  return out;
}
