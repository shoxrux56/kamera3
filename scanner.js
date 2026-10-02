/* DocScanner: hujjat/daftar chetini avtomatik topish, tekislash, filtr, PDF.
   index.html bilan ishlaydi: detect, cornerShift, orderCorners, warp, applyFilter, buildPdf */
(function (root) {
  "use strict";

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }

  function polyArea(p) {
    var s = 0;
    for (var i = 0; i < p.length; i++) { var a = p[i], b = p[(i + 1) % p.length]; s += a.x * b.y - b.x * a.y; }
    return Math.abs(s) / 2;
  }

  /* tartib: chap-yuqori, o'ng-yuqori, o'ng-past, chap-past */
  function orderCorners(pts) {
    var cx = 0, cy = 0, i;
    for (i = 0; i < pts.length; i++) { cx += pts[i].x; cy += pts[i].y; }
    cx /= pts.length; cy /= pts.length;
    var s = pts.map(function (p) { return { x: p.x, y: p.y, a: Math.atan2(p.y - cy, p.x - cx) }; })
      .sort(function (u, v) { return u.a - v.a; });
    var start = 0, best = Infinity;
    for (i = 0; i < s.length; i++) { var d = s[i].x + s[i].y; if (d < best) { best = d; start = i; } }
    var out = [];
    for (i = 0; i < s.length; i++) { var q = s[(start + i) % s.length]; out.push({ x: q.x, y: q.y }); }
    return out;
  }

  function cornerShift(a, b) {
    var m = 0;
    for (var i = 0; i < 4; i++) m = Math.max(m, dist(a[i], b[i]));
    return m;
  }

  /* to'rtburchak yaroqlimi: qavariq, burchaklari ~90, tomonlari yetarli */
  function goodQuad(p, W, H) {
    var i, sign = 0, minSide = 0.12 * Math.min(W, H);
    for (i = 0; i < 4; i++) {
      var a = p[i], b = p[(i + 1) % 4], c = p[(i + 2) % 4];
      var v1x = b.x - a.x, v1y = b.y - a.y, v2x = c.x - b.x, v2y = c.y - b.y;
      var cr = v1x * v2y - v1y * v2x;
      if (cr === 0) return false;
      var sg = cr > 0 ? 1 : -1;
      if (sign && sg !== sign) return false;
      sign = sg;
      var l1 = Math.hypot(v1x, v1y), l2 = Math.hypot(v2x, v2y);
      if (l1 < minSide || l2 < minSide) return false;
      var ang = Math.acos(Math.max(-1, Math.min(1, -(v1x * v2x + v1y * v2y) / (l1 * l2)))) * 180 / Math.PI;
      if (ang < 50 || ang > 130) return false;
    }
    var r1 = dist(p[0], p[1]) / dist(p[2], p[3]), r2 = dist(p[1], p[2]) / dist(p[3], p[0]);
    if (r1 < 0.45 || r1 > 2.2 || r2 < 0.45 || r2 > 2.2) return false;
    return true;
  }

  function clip(p, W, H) {
    return { x: Math.max(0, Math.min(W - 1, p.x)), y: Math.max(0, Math.min(H - 1, p.y)) };
  }

  /* ---------------- aniqlash ---------------- */
  function detect(cv, src) {
    var W = src.cols, H = src.rows, frame = W * H, trash = [];
    function M() { var m = new cv.Mat(); trash.push(m); return m; }
    var best = null, bestScore = 0.12;

    function evalContour(cnt) {
      var area = cv.contourArea(cnt);
      if (area < frame * 0.12 || area > frame * 0.985) return;
      var hull = new cv.Mat(), ap = new cv.Mat();
      try {
        cv.convexHull(cnt, hull, false, true);
        var hullArea = cv.contourArea(hull), peri = cv.arcLength(hull, true);
        var cands = [], eps = [0.015, 0.025, 0.035, 0.05, 0.07, 0.09], i, k;
        for (i = 0; i < eps.length; i++) {
          cv.approxPolyDP(hull, ap, eps[i] * peri, true);
          if (ap.rows === 4) {
            var d = ap.data32S, q = [];
            for (k = 0; k < 4; k++) q.push({ x: d[k * 2], y: d[k * 2 + 1] });
            cands.push({ pts: q, w: 1 });
            break;
          }
        }
        var rect = cv.minAreaRect(hull);
        cands.push({ pts: cv.RotatedRect.points(rect).map(function (p) { return { x: p.x, y: p.y }; }), w: 0.9 });
        cands.forEach(function (c) {
          var o = orderCorners(c.pts.map(function (p) { return clip(p, W, H); }));
          if (!goodQuad(o, W, H)) return;
          var qa = polyArea(o);
          var fit = Math.min(qa, hullArea) / Math.max(qa, hullArea);
          var score = (qa / frame) * fit * fit * c.w;
          if (score > bestScore) { bestScore = score; best = o; }
        });
      } finally { hull.delete(); ap.delete(); }
    }

    function scan(mask, mode) {
      var cs = new cv.MatVector(), hi = new cv.Mat();
      try {
        cv.findContours(mask, cs, hi, mode, cv.CHAIN_APPROX_SIMPLE);
        for (var i = 0; i < cs.size(); i++) { var c = cs.get(i); try { evalContour(c); } finally { c.delete(); } }
      } finally { cs.delete(); hi.delete(); }
    }

    try {
      var gray = M(); cv.cvtColor(src, gray, cv.COLOR_RGBA2GRAY);
      var blur = M(); cv.GaussianBlur(gray, blur, new cv.Size(5, 5), 0);
      var k = (Math.max(3, Math.round(Math.min(W, H) * 0.02)) | 1);
      var ker = cv.getStructuringElement(cv.MORPH_RECT, new cv.Size(k, k)); trash.push(ker);

      function clean(m) { cv.morphologyEx(m, m, cv.MORPH_OPEN, ker); cv.morphologyEx(m, m, cv.MORPH_CLOSE, ker); }

      // 1) yorqin varaq (stol to'q bo'lsa)
      var bright = M(); cv.threshold(blur, bright, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU);
      var dark = M(); cv.bitwise_not(bright, dark);
      clean(bright); clean(dark);
      scan(bright, cv.RETR_EXTERNAL);
      // 2) to'q daftar (stol och bo'lsa)
      scan(dark, cv.RETR_EXTERNAL);

      // 3) rang to'yinganligi: varaq kam to'yingan, stol ko'proq
      var rgb = M(); cv.cvtColor(src, rgb, cv.COLOR_RGBA2RGB);
      var hsv = M(); cv.cvtColor(rgb, hsv, cv.COLOR_RGB2HSV);
      var ch = new cv.MatVector(); cv.split(hsv, ch);
      var sat = ch.get(1), satB = M();
      try { cv.GaussianBlur(sat, satB, new cv.Size(5, 5), 0); } finally { sat.delete(); ch.delete(); }
      var lowSat = M(); cv.threshold(satB, lowSat, 0, 255, cv.THRESH_BINARY_INV | cv.THRESH_OTSU);
      clean(lowSat); scan(lowSat, cv.RETR_EXTERNAL);

      // 4) chetlar (Canny): ichki kontur ham olinadi
      var edges = M(); cv.Canny(blur, edges, 40, 120);
      var ek = cv.getStructuringElement(cv.MORPH_RECT, new cv.Size(3, 3)); trash.push(ek);
      cv.dilate(edges, edges, ek, new cv.Point(-1, -1), 2);
      scan(edges, cv.RETR_LIST);
    } finally {
      trash.forEach(function (m) { try { m.delete(); } catch (_) {} });
    }
    return best;
  }

  /* ---------------- tekislash ---------------- */
  function warp(cv, src, corners, maxSide) {
    var c = orderCorners(corners);
    var w = Math.max(dist(c[0], c[1]), dist(c[3], c[2])), h = Math.max(dist(c[0], c[3]), dist(c[1], c[2]));
    var k = Math.min(1, (maxSide || 2000) / Math.max(w, h));
    var ow = Math.max(16, Math.round(w * k)), oh = Math.max(16, Math.round(h * k));
    var sm = cv.matFromArray(4, 1, cv.CV_32FC2, [c[0].x, c[0].y, c[1].x, c[1].y, c[2].x, c[2].y, c[3].x, c[3].y]);
    var dm = cv.matFromArray(4, 1, cv.CV_32FC2, [0, 0, ow - 1, 0, ow - 1, oh - 1, 0, oh - 1]);
    var T = cv.getPerspectiveTransform(sm, dm), out = new cv.Mat();
    try {
      cv.warpPerspective(src, out, T, new cv.Size(ow, oh), cv.INTER_LINEAR, cv.BORDER_REPLICATE, new cv.Scalar());
    } finally { sm.delete(); dm.delete(); T.delete(); }
    return out;
  }

  /* ---------------- filtrlar ---------------- */
  // yoritilish notekisligi va soyani yo'qotadi (varaq fonini 255 ga keltiradi)
  function flatten(cv, rgb) {
    var w = rgb.cols, h = rgb.rows, s = Math.max(1, Math.round(Math.max(w, h) / 200));
    var small = new cv.Mat(), bg = new cv.Mat(), out = new cv.Mat();
    var ker = cv.getStructuringElement(cv.MORPH_RECT, new cv.Size(5, 5));
    try {
      cv.resize(rgb, small, new cv.Size(Math.max(1, Math.round(w / s)), Math.max(1, Math.round(h / s))), 0, 0, cv.INTER_AREA);
      cv.dilate(small, small, ker);
      cv.GaussianBlur(small, small, new cv.Size(9, 9), 0);
      cv.resize(small, bg, new cv.Size(w, h), 0, 0, cv.INTER_LINEAR);
      cv.divide(rgb, bg, out, 255, -1);
    } finally { small.delete(); bg.delete(); ker.delete(); }
    return out;
  }

  function applyFilter(cv, mat, mode) {
    var rgb = new cv.Mat(), flat = null, out = new cv.Mat();
    try {
      cv.cvtColor(mat, rgb, cv.COLOR_RGBA2RGB);
      flat = flatten(cv, rgb);
      if (mode === "gray") {
        cv.cvtColor(flat, out, cv.COLOR_RGB2GRAY);
      } else if (mode === "bw") {
        var g = new cv.Mat();
        try {
          cv.cvtColor(flat, g, cv.COLOR_RGB2GRAY);
          cv.GaussianBlur(g, g, new cv.Size(3, 3), 0);
          cv.threshold(g, out, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU);
        } finally { g.delete(); }
      } else {
        flat.copyTo(out);
      }
    } finally { rgb.delete(); if (flat) flat.delete(); }
    return out;
  }

  /* ---------------- PDF ---------------- */
  // items: [{bytes: Uint8Array (JPEG), w, h}]
  function buildPdf(jsPDF, items) {
    var doc = null, A = [595.28, 841.89];
    items.forEach(function (it) {
      var land = it.w > it.h;
      var bw = land ? A[1] : A[0], bh = land ? A[0] : A[1];
      var k = Math.min(bw / it.w, bh / it.h), pw = it.w * k, ph = it.h * k;
      var orient = land ? "l" : "p";
      if (!doc) doc = new jsPDF({ unit: "pt", format: [pw, ph], orientation: orient, compress: true });
      else doc.addPage([pw, ph], orient);
      doc.addImage(it.bytes, "JPEG", 0, 0, pw, ph, undefined, "FAST");
    });
    return doc.output("arraybuffer");
  }

  root.DocScanner = {
    detect: detect, warp: warp, applyFilter: applyFilter, buildPdf: buildPdf,
    orderCorners: orderCorners, cornerShift: cornerShift
  };
})(typeof window !== "undefined" ? window : this);
