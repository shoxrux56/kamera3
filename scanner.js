/*
 * scanner.js — hujjatni topish, perspektivani tekislash va skan filtrlari.
 * OpenCV.js ustida ishlaydi. Brauzerda `DocScanner`, node'da module.exports sifatida chiqadi.
 * Barcha funksiyalar `cv` ni birinchi argument sifatida oladi.
 */
(function (root, factory) {
  if (typeof module === "object" && module.exports) module.exports = factory();
  else root.DocScanner = factory();
})(typeof self !== "undefined" ? self : this, function () {
  "use strict";

  // Nomzodni qabul qilish chegaralari (tomonlarning eng zaifi va o'rtachasi)
  const MIN_WEAKEST = 0.32;
  const MIN_MEAN = 0.7;

  function dist(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
  }

  /** 4 ta nuqtani [chap-yuqori, o'ng-yuqori, o'ng-past, chap-past] tartibiga keltiradi. */
  function orderCorners(pts) {
    const bySum = pts.slice().sort((p, q) => p.x + p.y - (q.x + q.y));
    const tl = bySum[0];
    const br = bySum[3];
    const rest = [bySum[1], bySum[2]];
    // y - x: eng kichigi o'ng-yuqori, eng kattasi chap-past
    rest.sort((p, q) => p.y - p.x - (q.y - q.x));
    return [tl, rest[0], br, rest[1]];
  }

  function polyArea(p) {
    let s = 0;
    for (let i = 0; i < p.length; i++) {
      const a = p[i];
      const b = p[(i + 1) % p.length];
      s += a.x * b.y - b.x * a.y;
    }
    return Math.abs(s) / 2;
  }

  /** To'rtburchak "hujjatga o'xshaydimi": burchaklari 50°–130° oralig'ida, tomonlari juda qisqa emas. */
  function looksLikeDocument(q) {
    const sides = [dist(q[0], q[1]), dist(q[1], q[2]), dist(q[2], q[3]), dist(q[3], q[0])];
    const longest = Math.max.apply(null, sides);
    if (Math.min.apply(null, sides) < longest * 0.12) return false;
    for (let i = 0; i < 4; i++) {
      const p = q[(i + 3) % 4];
      const c = q[i];
      const n = q[(i + 1) % 4];
      const v1 = { x: p.x - c.x, y: p.y - c.y };
      const v2 = { x: n.x - c.x, y: n.y - c.y };
      const cos = (v1.x * v2.x + v1.y * v2.y) / (Math.hypot(v1.x, v1.y) * Math.hypot(v2.x, v2.y) || 1);
      if (Math.abs(cos) > 0.64) return false; // ~50° dan kichik yoki 130° dan katta
    }
    return true;
  }

  /** Burchaklarni markazga qarab d piksel siljitadi (qalinlashtirilgan qirra chizig'i sabab paydo bo'ladigan tashqi og'ishni qaytaradi). */
  function shrink(q, d) {
    const cx = (q[0].x + q[1].x + q[2].x + q[3].x) / 4;
    const cy = (q[0].y + q[1].y + q[2].y + q[3].y) / 4;
    return q.map(function (p) {
      const vx = cx - p.x;
      const vy = cy - p.y;
      const l = Math.hypot(vx, vy) || 1;
      return { x: p.x + (vx / l) * d, y: p.y + (vy / l) * d };
    });
  }

  /** Ikki burchaklar to'plami orasidagi eng katta siljish (piksel). */
  function cornerShift(a, b) {
    let m = 0;
    for (let i = 0; i < 4; i++) m = Math.max(m, dist(a[i], b[i]));
    return m;
  }

  /** Ikkilik tasvirdagi barcha "hujjatsimon" to'rtburchak nomzodlarni `out` ga yig'adi. */
  function collectQuads(cv, bin, mode, minArea, maxArea, tag, out) {
    const contours = new cv.MatVector();
    const hierarchy = new cv.Mat();
    try {
      cv.findContours(bin, contours, hierarchy, mode, cv.CHAIN_APPROX_SIMPLE);
      for (let i = 0; i < contours.size(); i++) {
        const cnt = contours.get(i);
        try {
          // Qirra chizig'i uzilgan bo'lsa, kontur yuzi kichik chiqadi — shuning uchun o'rab turuvchi to'rtburchakka qaraymiz
          const br = cv.boundingRect(cnt);
          if (br.width * br.height < minArea) continue;
          const hull = new cv.Mat();
          try {
            cv.convexHull(cnt, hull, false, true);
            const peri = cv.arcLength(hull, true);
            const eps = [0.02, 0.03, 0.045, 0.06];
            for (let k = 0; k < eps.length; k++) {
              const approx = new cv.Mat();
              try {
                cv.approxPolyDP(hull, approx, eps[k] * peri, true);
                if (approx.rows !== 4) continue;
                const d = approx.data32S;
                const pts = [];
                for (let j = 0; j < 4; j++) pts.push({ x: d[j * 2], y: d[j * 2 + 1] });
                const q = orderCorners(pts);
                const area = polyArea(q);
                if (area < minArea || area > maxArea) break;
                if (!looksLikeDocument(q)) break;
                out.push({ corners: q, area: area, tag: tag });
                break;
              } finally {
                approx.delete();
              }
            }
          } finally {
            hull.delete();
          }
        } finally {
          cnt.delete();
        }
      }
    } finally {
      contours.delete();
      hierarchy.delete();
    }
  }

  /**
   * Nomzod to'rtburchakning har bir tomoni haqiqiy chegara (qirra) ustida yotishini o'lchaydi.
   * `edgeMap` — qalinlashtirilgan Canny xaritasi. Tasvir chetiga yopishgan tomonlar "qo'llab-quvvatlangan" hisoblanadi.
   * Qaytaradi: {sides: [4 ta ulush 0..1], border: chetga yopishgan nuqtalar ulushi}.
   */
  function edgeSupport(q, edgeMap) {
    const W = edgeMap.cols;
    const H = edgeMap.rows;
    const data = edgeMap.data;
    const margin = Math.max(3, Math.round(Math.min(W, H) * 0.02));
    const sides = [];
    const sideBorder = [];
    let onBorderCount = 0;
    let total = 0;
    for (let i = 0; i < 4; i++) {
      const a = q[i];
      const b = q[(i + 1) % 4];
      const len = dist(a, b);
      const n = Math.max(8, Math.round(len / 3));
      let hit = 0;
      let onSide = 0;
      for (let k = 0; k < n; k++) {
        const t = (k + 0.5) / n;
        const x = Math.round(a.x + (b.x - a.x) * t);
        const y = Math.round(a.y + (b.y - a.y) * t);
        const onBorder = x <= margin || y <= margin || x >= W - 1 - margin || y >= H - 1 - margin;
        if (onBorder) { onBorderCount++; onSide++; }
        if (onBorder || (x >= 0 && y >= 0 && x < W && y < H && data[y * W + x] > 0)) hit++;
      }
      total += n;
      sides.push(hit / n);
      sideBorder.push(onSide / n);
    }
    return { sides: sides, border: onBorderCount / total, sideBorder: sideBorder };
  }

  /**
   * Rang kontrasti bo'yicha qo'llab-quvvatlash: tomonning ichkarisi va tashqarisidagi ranglar
   * butun tomon bo'ylab bir xil yo'nalishda farq qiladimi. Qirra xaritasi uzilgan, lekin kontrast bor joylarni ham ushlaydi.
   * `img` — biroz xiralashtirilgan RGBA Mat.
   */
  function contrastSupport(q, img) {
    const W = img.cols;
    const H = img.rows;
    const px = img.data;
    const off = Math.max(3, Math.round(Math.min(W, H) * 0.011));
    const T = 7;
    const cx = (q[0].x + q[1].x + q[2].x + q[3].x) / 4;
    const cy = (q[0].y + q[1].y + q[2].y + q[3].y) / 4;
    const sides = [];
    for (let i = 0; i < 4; i++) {
      const a = q[i];
      const b = q[(i + 1) % 4];
      const len = dist(a, b) || 1;
      const n = Math.max(8, Math.round(len / 3));
      let nx = -(b.y - a.y) / len;
      let ny = (b.x - a.x) / len;
      if (nx * (cx - (a.x + b.x) / 2) + ny * (cy - (a.y + b.y) / 2) < 0) {
        nx = -nx;
        ny = -ny;
      }
      const diffs = [];
      let skipped = 0;
      for (let k = 0; k < n; k++) {
        const t = (k + 0.5) / n;
        const x = a.x + (b.x - a.x) * t;
        const y = a.y + (b.y - a.y) * t;
        const xi = Math.round(x + nx * off);
        const yi = Math.round(y + ny * off);
        const xo = Math.round(x - nx * off);
        const yo = Math.round(y - ny * off);
        if (xo < 0 || yo < 0 || xo >= W || yo >= H || xi < 0 || yi < 0 || xi >= W || yi >= H) {
          skipped++; // tasvir chetidan chiqib ketgan — hujjat kadrdan o'tgan tomon
          continue;
        }
        const pi = (yi * W + xi) * 4;
        const po = (yo * W + xo) * 4;
        diffs.push([px[pi] - px[po], px[pi + 1] - px[po + 1], px[pi + 2] - px[po + 2]]);
      }
      let hit = skipped;
      if (diffs.length) {
        let mx = 0;
        let my = 0;
        let mz = 0;
        diffs.forEach(function (d) {
          mx += d[0];
          my += d[1];
          mz += d[2];
        });
        mx /= diffs.length;
        my /= diffs.length;
        mz /= diffs.length;
        const mag = Math.hypot(mx, my, mz);
        if (mag >= T) {
          diffs.forEach(function (d) {
            if ((d[0] * mx + d[1] * my + d[2] * mz) / mag > T) hit++;
          });
        }
      }
      sides.push(hit / n);
    }
    return sides;
  }

  /**
   * RGBA Mat ichidan hujjat burchaklarini topadi.
   * Qaytaradi: [{x,y} x4] (TL, TR, BR, BL) yoki null.
   * opts.debug = true bo'lsa, {best, candidates} qaytaradi (sinov uchun).
   */
  function detect(cv, src, opts) {
    const W = src.cols;
    const H = src.rows;
    const img = src;
    const imgArea = W * H;
    const minArea = imgArea * 0.12;
    const maxArea = imgArea * 0.985;
    const gray = new cv.Mat();
    const blur = new cv.Mat();
    const edges = new cv.Mat();
    const edgesLow = new cv.Mat();
    const support = new cv.Mat();
    const bin = new cv.Mat();
    const tmp = new cv.Mat();
    const k3 = cv.getStructuringElement(cv.MORPH_RECT, new cv.Size(3, 3));
    const k9 = cv.getStructuringElement(cv.MORPH_RECT, new cv.Size(9, 9));
    const cands = [];
    try {
      cv.cvtColor(src, gray, cv.COLOR_RGBA2GRAY);
      const planes = new cv.MatVector();
      cv.split(src, planes);
      const rgb = new cv.Mat();
      const lab = new cv.Mat();
      const labPlanes = new cv.MatVector();
      cv.cvtColor(src, rgb, cv.COLOR_RGBA2RGB);
      cv.cvtColor(rgb, lab, cv.COLOR_RGB2Lab);
      cv.split(lab, labPlanes);
      rgb.delete();
      lab.delete();
      // Yopish (close): matn, ingichka chiziq va sim kabi qorong'i mayda narsalarni "o'chiradi",
      // shunda hujjat bir tekis yorqin dog' bo'lib, chegarasi aniq ko'rinadi.
      const ks = oddAtLeast(Math.min(W, H) * 0.022, 5);
      const kClose = cv.getStructuringElement(cv.MORPH_RECT, new cv.Size(ks, ks));
      cv.morphologyEx(gray, gray, cv.MORPH_CLOSE, kClose);
      cv.GaussianBlur(gray, blur, new cv.Size(5, 5), 0);

      let otsu = cv.threshold(blur, tmp, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU);
      otsu = Math.min(200, Math.max(40, otsu));

      // Qirra xaritalari: kuchli va zaif (past kontrastli hujjat uchun). Yorqinlik + R, G, B kanallari birlashtiriladi.
      cv.Canny(blur, edges, otsu * 0.4, otsu);
      cv.Canny(blur, edgesLow, 12, 36);
      const ch = new cv.Mat();
      const chE = new cv.Mat();
      try {
        for (let c = 0; c < 5; c++) {
          const plane = c < 3 ? planes.get(c) : labPlanes.get(c - 2); // a*, b* — sof rang farqi kanallari
          try {
            cv.morphologyEx(plane, ch, cv.MORPH_CLOSE, kClose);
            cv.GaussianBlur(ch, ch, new cv.Size(5, 5), 0);
            let t = cv.threshold(ch, tmp, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU);
            t = Math.min(200, Math.max(40, t));
            cv.Canny(ch, chE, t * 0.4, t);
            cv.bitwise_or(edges, chE, edges);
            cv.Canny(ch, chE, 12, 36);
            cv.bitwise_or(edgesLow, chE, edgesLow);
          } finally {
            plane.delete();
          }
        }
      } finally {
        ch.delete();
        chE.delete();
        planes.delete();
        labPlanes.delete();
        kClose.delete();
      }
      cv.dilate(edges, support, k3, new cv.Point(-1, -1), 3);

      // 1) Kuchli Canny
      cv.dilate(edges, edges, k3, new cv.Point(-1, -1), 2);
      collectQuads(cv, edges, cv.RETR_LIST, minArea, maxArea, "canny", cands);

      // 2) Zaif Canny (oq qog'oz — och stol)
      cv.dilate(edgesLow, edgesLow, k3, new cv.Point(-1, -1), 2);
      collectQuads(cv, edgesLow, cv.RETR_LIST, minArea, maxArea, "cannyLow", cands);

      // 3) Yorqin qog'oz — Otsu
      cv.threshold(blur, bin, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU);
      cv.morphologyEx(bin, bin, cv.MORPH_CLOSE, k9);
      collectQuads(cv, bin, cv.RETR_EXTERNAL, minArea, maxArea, "otsu", cands);

      // 3.5) Notekis yoritilgan sahna: mahalliy o'rtachaga nisbatan chegara (global Otsu emas)
      const blockSize = oddAtLeast(Math.min(W, H) * 0.06, 15);
      cv.adaptiveThreshold(blur, bin, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, blockSize, -4);
      cv.morphologyEx(bin, bin, cv.MORPH_CLOSE, k9);
      cv.morphologyEx(bin, bin, cv.MORPH_OPEN, k3);
      collectQuads(cv, bin, cv.RETR_EXTERNAL, minArea, maxArea, "adaptive", cands);
      cv.adaptiveThreshold(blur, bin, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, blockSize, -4);
      cv.morphologyEx(bin, bin, cv.MORPH_CLOSE, k9);
      cv.morphologyEx(bin, bin, cv.MORPH_OPEN, k3);
      collectQuads(cv, bin, cv.RETR_EXTERNAL, minArea, maxArea, "adaptiveInv", cands);

      // 4) Qorong'i hujjat yorug' fonda — teskari Otsu
      cv.threshold(blur, bin, 0, 255, cv.THRESH_BINARY_INV | cv.THRESH_OTSU);
      cv.morphologyEx(bin, bin, cv.MORPH_CLOSE, k9);
      collectQuads(cv, bin, cv.RETR_EXTERNAL, minArea, maxArea, "otsuInv", cands);

      // Baholash: har tomon haqiqiy chegara ustida bo'lsin (qirra yoki rang kontrasti), kadr chetiga yopishganlar jarimalansin.
      const lowSupport = new cv.Mat();
      const soft = new cv.Mat();
      cv.dilate(edgesLow, lowSupport, k3, new cv.Point(-1, -1), 1);
      cv.GaussianBlur(src, soft, new cv.Size(5, 5), 0);
      cands.forEach(function (c) {
        const strict = edgeSupport(c.corners, support);
        const low = edgeSupport(c.corners, lowSupport);
        // Hujjat bir vaqtning o'zida kadrning ham chap, ham o'ng (yoki ham yuqori, ham past)
        // chetiga "yopishib" chiqishi kamdan-kam uchraydi — bu ko'proq gorizontal/vertikal
        // tekstura chizig'i (yog'och tomirlari, sim) ni hujjat deb qabul qilib qo'yish belgisi.
        const oppositeBorder =
          Math.max(
            Math.min(strict.sideBorder[1], strict.sideBorder[3]),
            Math.min(strict.sideBorder[0], strict.sideBorder[2])
          ) > 0.5;
        const areaFrac = c.area / (img.cols * img.rows);
        const con = contrastSupport(c.corners, soft);
        const sides = [];
        for (let i = 0; i < 4; i++) sides.push(Math.max(strict.sides[i], low.sides[i] * 0.9, con[i]));
        c.support = Math.min.apply(null, strict.sides);
        c.supportLow = Math.min.apply(null, low.sides);
        c.contrast = Math.min.apply(null, con);
        c.border = strict.border;
        const weakest = Math.min.apply(null, sides);
        const mean = (sides[0] + sides[1] + sides[2] + sides[3]) / 4;
        c.weakest = weakest;
        c.mean = mean;
        const penalty = oppositeBorder && areaFrac < 0.55 ? 0 : 1;
        c.score = weakest >= MIN_WEAKEST && mean >= MIN_MEAN ? c.area * mean * (1 - 0.8 * strict.border) * penalty : 0;
      });
      lowSupport.delete();
      soft.delete();
      // Ishonchlilik tartibi: qirra bo'yicha topilganlar mintaqa bo'yicha topilganlardan ustun
      let best = null;
      const tiers = ["canny", "cannyLow", "adaptive", "adaptiveInv", "otsu", "otsuInv"];
      for (let t = 0; t < tiers.length && !best; t++) {
        cands.forEach(function (c) {
          if (c.tag === tiers[t] && c.score > 0 && (!best || c.score > best.score)) best = c;
        });
      }
      if (best && (best.tag === "canny" || best.tag === "cannyLow")) best.corners = shrink(best.corners, 2);
      if (opts && opts.debug) return { best: best, candidates: cands };
      return best ? best.corners : null;
    } finally {
      gray.delete();
      blur.delete();
      edges.delete();
      edgesLow.delete();
      support.delete();
      bin.delete();
      tmp.delete();
      k3.delete();
      k9.delete();
    }
  }

  /** Perspektivani tekislaydi: burchaklar ichidagi hujjatni to'g'ri to'rtburchakka aylantiradi. Yangi RGBA Mat qaytaradi. */
  function warp(cv, src, corners, maxSide) {
    const c = orderCorners(corners);
    let w = Math.max(dist(c[0], c[1]), dist(c[3], c[2]));
    let h = Math.max(dist(c[0], c[3]), dist(c[1], c[2]));
    const limit = maxSide || 2000;
    const scale = Math.min(1, limit / Math.max(w, h));
    w = Math.max(2, Math.round(w * scale));
    h = Math.max(2, Math.round(h * scale));
    const from = cv.matFromArray(4, 1, cv.CV_32FC2, [c[0].x, c[0].y, c[1].x, c[1].y, c[2].x, c[2].y, c[3].x, c[3].y]);
    const to = cv.matFromArray(4, 1, cv.CV_32FC2, [0, 0, w - 1, 0, w - 1, h - 1, 0, h - 1]);
    const M = cv.getPerspectiveTransform(from, to);
    const out = new cv.Mat();
    try {
      cv.warpPerspective(src, out, M, new cv.Size(w, h), cv.INTER_LINEAR, cv.BORDER_REPLICATE, new cv.Scalar());
      return out;
    } finally {
      from.delete();
      to.delete();
      M.delete();
    }
  }

  function oddAtLeast(n, min) {
    n = Math.max(min, Math.round(n));
    return n % 2 === 0 ? n + 1 : n;
  }

  /** Yorug'lik notekisligini yo'qotadi: fon oq bo'ladi, matn qorong'i qoladi. 8-bit bir kanalli Mat qaytaradi. */
  function flattenGray(cv, rgba) {
    const gray = new cv.Mat();
    const small = new cv.Mat();
    const bg = new cv.Mat();
    const out = new cv.Mat();
    const k = cv.getStructuringElement(cv.MORPH_RECT, new cv.Size(5, 5));
    try {
      cv.cvtColor(rgba, gray, cv.COLOR_RGBA2GRAY);
      const sw = Math.max(8, Math.round(gray.cols / 6));
      const sh = Math.max(8, Math.round(gray.rows / 6));
      cv.resize(gray, small, new cv.Size(sw, sh), 0, 0, cv.INTER_AREA);
      cv.dilate(small, small, k); // qorong'i matnni fon bahosidan olib tashlaydi
      const g = oddAtLeast(Math.min(sw, sh) / 3, 5);
      cv.GaussianBlur(small, small, new cv.Size(g, g), 0);
      cv.resize(small, bg, new cv.Size(gray.cols, gray.rows), 0, 0, cv.INTER_LINEAR);
      cv.divide(gray, bg, out, 255);
      return out;
    } catch (e) {
      out.delete();
      throw e;
    } finally {
      gray.delete();
      small.delete();
      bg.delete();
      k.delete();
    }
  }

  /** mode: "color" | "gray" | "bw". Yangi RGBA Mat qaytaradi (src ga tegmaydi). */
  function applyFilter(cv, src, mode) {
    const out = new cv.Mat();
    if (mode === "gray" || mode === "bw") {
      const flat = flattenGray(cv, src);
      try {
        if (mode === "bw") {
          const bw = new cv.Mat();
          try {
            cv.threshold(flat, bw, 0, 255, cv.THRESH_BINARY | cv.THRESH_OTSU);
            cv.cvtColor(bw, out, cv.COLOR_GRAY2RGBA);
          } finally {
            bw.delete();
          }
        } else {
          cv.cvtColor(flat, out, cv.COLOR_GRAY2RGBA);
        }
      } finally {
        flat.delete();
      }
      return out;
    }
    src.copyTo(out);
    return out;
  }

  /**
   * JPEG sahifalardan PDF yig'adi. pages: [{bytes: Uint8Array, w, h}]
   * Har bir sahifa o'z nisbatida bo'ladi (uzun tomoni 297 mm). ArrayBuffer qaytaradi.
   */
  function buildPdf(jsPDF, pages) {
    const LONG = 297;
    let doc = null;
    pages.forEach(function (p) {
      const land = p.w >= p.h;
      const pw = land ? LONG : (LONG * p.w) / p.h;
      const ph = land ? (LONG * p.h) / p.w : LONG;
      const o = land ? "l" : "p";
      if (!doc) doc = new jsPDF({ unit: "mm", format: [pw, ph], orientation: o, compress: true });
      else doc.addPage([pw, ph], o);
      doc.addImage(p.bytes, "JPEG", 0, 0, pw, ph, undefined, "FAST");
    });
    return doc.output("arraybuffer");
  }

  return {
    buildPdf: buildPdf,
    orderCorners: orderCorners,
    looksLikeDocument: looksLikeDocument,
    cornerShift: cornerShift,
    detect: detect,
    warp: warp,
    applyFilter: applyFilter,
  };
});
