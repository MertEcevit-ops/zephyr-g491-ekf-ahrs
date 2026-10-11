# Kanıt durumu

Düzen `SPECS/DOCS/TEST_PLAN.md §0.5`'e göredir. Son güncelleme: 2026-10-12.

| Faz | Durum | Açıklama |
|---|---|---|
| 0 — Donanım hazırlığı | **Kayıt yok** | Jumper kesimi ve süreklilik ölçümleri elle yapılır; `phase0/jumper_check.md` henüz yazılmadı |
| 1 — Bus ve kimlik (`tests/hw/bus_id`) | **Koşmadı** | İki senaryo `nucleo_g491re` için uyarısız derlendi. Koşu sırasında ST-LINK USB'de görünüyordu ama sanal seri port yoktu (`/dev/ttyACM0` yok; `cdc_acm` arayüzden ayrılmış, yeniden bağlamak root ister) |
| 2 — Ham veri (`tests/hw/raw_data`, elle testler) | **Koşmadı** | Üç senaryo derlendi. T-RAW-09…12 elle hareket ister. `mag_cal.h` `MAG_CAL_VALID 0`, `accel_cal.h` birim değerlerle duruyor |
| 3 — Ztest matematik testleri | **Geçti** | `phase3/SUMMARY.md` |
| 4 — Dinamik entegrasyon | **Koşmadı** | Faz 1 ve 2 kapanmadan başlamaz. Ana uygulama `nucleo_g491re` için uyarısız derleniyor; hedefte hiç çalıştırılmadı |

Donanımda doğrulanmamış olanlar: SPI trafiği ve init sırası, MMC5983MA sürücüsünün gerçek çiple
davranışı, DRDY zincirinin zamanlaması, thread yığın kullanımları, telemetri ve shell komutları,
`tools/capture.py`'nin gerçek seri portla çalışması.
