# Kanıt durumu

Düzen `SPECS/DOCS/TEST_PLAN.md §0.5`'e göredir. Son güncelleme: 2026-10-12.

| Faz | Durum | Açıklama |
|---|---|---|
| 0 — Donanım hazırlığı | **Kayıt yok** | Jumper kesimi ve süreklilik ölçümleri elle yapılır; `phase0/jumper_check.md` henüz yazılmadı |
| 1 — Bus ve kimlik (`tests/hw/bus_id`) | **Geçti** | `phase1/SUMMARY.md`. T-BUS-10 (lojik analizör) yapılmadı. Faz 0 kaydı olmadan koşturuldu |
| 2 — Ham veri (`tests/hw/raw_data`, elle testler) | **Kapanmadı** | `phase2/SUMMARY.md`: T-RAW-00, 03, 04, 06, 08 geçti; T-RAW-01, 05, 07 kaldı. Koşuda kart düz değildi. T-RAW-09…12 elle hareket ister. `mag_cal.h` `MAG_CAL_VALID 0`, `accel_cal.h` birim değerlerle duruyor |
| 3 — Ztest matematik testleri | **Geçti** | `phase3/SUMMARY.md` |
| 4 — Dinamik entegrasyon | **Koşmadı** | Faz 1 ve 2 kapanmadan başlamaz. Ana uygulama `nucleo_g491re` için uyarısız derleniyor; hedefte hiç çalıştırılmadı |

Donanımda doğrulananlar (2026-10-12): SPI trafiği ve init sırası, MMC5983MA sürücüsünün init'i,
boru hattı okuması ve dönüşüm süresi, DRDY zinciri ve 10 dakikalık hatasız akış.

Hâlâ doğrulanmamış olanlar: ana uygulama (füzyon, telemetri, shell komutları) hedefte hiç
çalıştırılmadı; thread yığın kullanımları ölçülmedi; `tools/capture.py` gerçek seri portla denenmedi.
