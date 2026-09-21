# Hujjat skaneri (serversiz)

Faqat statik fayllar: `index.html` + `scanner.js`. Backend yo'q.

## Joylash (istalgan statik hosting)

Cloudflare Pages, GitHub Pages, Netlify yoki o'z serveringiz — ikkala faylni bir papkaga qo'yasiz.
Telefonda kamera faqat HTTPS da ishlaydi (hostinglar buni o'zi beradi).

Kompyuterda sinash: papkada `python -m http.server 8000`, keyin http://localhost:8000
(localhost kameraga ruxsat beradi).

## Telegram

Saytda: Ko'rib chiqish → "Telegram sozlamalari" → bot tokeni + qabul qiluvchi chat ID.
Qabul qiluvchi avval botga /start yozgan bo'lishi kerak; "Chat ID ni avtomatik topish" tugmasi shundan keyin ishlaydi.
Token faqat shu brauzerning localStorage ida saqlanadi. Uni kodga yozmang.

"Ulashish" tugmasi (telefonda ko'rinadi) tokensiz ishlaydi: PDF ni Telegram ochiladigan ulashish oynasi orqali
istalgan odamga yuborasiz.

## Eslatma

- OpenCV.js (~13 MB) CDN dan yuklanadi va keshlanadi. CDN ishlamasa, `opencv.js` ni shu papkaga qo'ying.
- Telegram bot orqali fayl chegarasi 50 MB.
