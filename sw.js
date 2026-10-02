/* Service worker: kutubxonalar (OpenCV, jsPDF) bir marta yuklanib keshlanadi, o'z fayllaringiz esa
   har doim avval tarmoqdan olinadi (GitHub'ga yangi kod yuklasangiz, darrov ko'rinadi), oflaynda keshdan ishlaydi. */
var V = "docscan-v1";
var LIBS = /^https:\/\/(cdn\.jsdelivr\.net|unpkg\.com|cdnjs\.cloudflare\.com|docs\.opencv\.org)\//;

self.addEventListener("install", function () { self.skipWaiting(); });

self.addEventListener("activate", function (e) {
  e.waitUntil(
    caches.keys()
      .then(function (ks) { return Promise.all(ks.filter(function (k) { return k !== V; }).map(function (k) { return caches.delete(k); })); })
      .then(function () { return self.clients.claim(); })
  );
});

self.addEventListener("fetch", function (e) {
  var req = e.request;
  if (req.method !== "GET") return;

  if (LIBS.test(req.url)) {
    e.respondWith(
      caches.open(V).then(function (c) {
        return c.match(req.url).then(function (hit) {
          if (hit) return hit;
          return fetch(req.url, { mode: "cors" }).then(function (r) { if (r.ok) c.put(req.url, r.clone()); return r; });
        });
      }).catch(function () { return fetch(req); })
    );
    return;
  }

  if (new URL(req.url).origin === self.location.origin) {
    e.respondWith(
      fetch(req).then(function (r) {
        if (r.ok) { var cp = r.clone(); caches.open(V).then(function (c) { c.put(req, cp); }); }
        return r;
      }).catch(function () { return caches.match(req); })
    );
  }
});
