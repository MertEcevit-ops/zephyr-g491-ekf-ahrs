# Faz 2 — Ham veri, ölçekleme, zamanlama: SUMMARY

Koşu: 2026-10-12, git commit `e7f35e9`, NUCLEO-G491RE, seri port `/dev/ttyACM1`.

```sh
west twister -T app/tests/hw/raw_data -p nucleo_g491re --device-testing --device-serial /dev/ttyACM1
```

Twister özeti: 3 senaryodan 1'i geçti, 10 test durumundan 6'sı geçti, 4'ü kaldı. **Faz 2 kapanmadı.**

**Koşu koşulları test ön koşullarını sağlamıyordu:** kart düz değildi (ivme ortalaması ≈ 27° eğim
gösteriyor) ve ölçüm masasının manyetik temizliği doğrulanmadı (PC yanında). Aşağıdaki FAIL'lerin
ikisi bundan etkilenir; biri (T-RAW-07 ortalama Δt) koşullardan bağımsız, çipe özgü bir bulgudur.

| Test ID | Sonuç | Tarih | git commit | Not |
|---|---|---|---|---|
| T-RAW-00 | PASS | 2026-10-12 | e7f35e9 | Sürücü hazır; init ofseti (30822, 139221, 50354) µG |
| T-RAW-01 | **FAIL** | 2026-10-12 | e7f35e9 | $\bar f_A$ = (1.154, 4.515, 8.808) m/s². Norm 9.965 ve std (5, 4, 5) mm/s² sınırların içinde; x/y ortalamaları kart eğik durduğu için sınır dışı. Düz yüzeyde tekrarlanmalı |
| T-RAW-02 | Değerlendirilmedi | — | — | T-RAW-01 ile aynı test fonksiyonunda; ilk assert'te durdu. Ölçülen $\bar f_{b,z}$ = −8.808 (eşik < −9.0, eğim nedeniyle) |
| T-RAW-03 | PASS | 2026-10-12 | e7f35e9 | Jiroskop ortalaması (1.6, 2.2, 1.2) mrad/s, std (1.0, 1.1, 1.4) mrad/s |
| T-RAW-04 | PASS | 2026-10-12 | e7f35e9 | $\lVert\bar m_M\rVert$ = 0.729 G; std (0.48, 1.35, 0.87) mG; `-EBUSY` 0, `-ERANGE` 0 |
| T-RAW-05 (1000 µs) | **FAIL** | 2026-10-12 | e7f35e9 | Y ekseni std 1190 µG (sınır 1000), aralık 4211 µG (sınır 4000). X: std 319, aralık 1098. Z raporlanmadı (test Y'de durdu) |
| T-RAW-05 (100 µs, bilgi) | FAIL | 2026-10-12 | e7f35e9 | Y: std 1617 µG, aralık 6226 µG. X: std 415, aralık 1709 |
| T-RAW-06 | PASS | 2026-10-12 | e7f35e9 | BW=01 dönüşüm süresi 3864–3865 µs (200 ölçüm, sınır 6000); STATUS yazımından sonra INT düşük |
| T-RAW-07 | **FAIL** | 2026-10-12 | e7f35e9 | Ortalama Δt 8995 µs (beklenen 9615 ± %2). std < 1 µs, `dt_fault` 0, drop 0, tik p99 ≤ 280 µs, en yavaş tik 1416 µs, 10 SET tiki: bunlar sınırların içinde |
| T-RAW-08 | PASS | 2026-10-12 | e7f35e9 | 62 400 örnek (≈ 561 s); `dt_fault`, drop, SPI hatası, `-EBUSY`, DRDY zaman aşımı = 0 |
| T-RAW-09…12 | Koşmadı | — | — | Elle hareket ister |

Kanıt: `app.hw.raw_data.hub.log`, `app.hw.raw_data.int_diag.log`, `app.hw.raw_data.settle100.log`, `twister.json`.

## Bulgular

**1. Örnekleme periyodu 9.615 ms değil 8.995 ms (T-RAW-07).** ISM330DHCX veri hızını kendi iç
osilatöründen üretir. Bu çipin `INTERNAL_FREQ_FINE` (0x63) register'ı +45 okuyor (adım %0.15):
nominalden %+6.75 hızlı, öngörülen periyot 8992 µs, ölçülen 8995 µs (`phase1/SUMMARY.md`). Yani
bu kart 104 Hz değil ≈ 111.2 Hz örnekliyor. Etkileri:

- EKF her adımda ölçülen Δt'yi kullandığı için entegrasyon doğru kalır.
- `MATH_SPEC.md §14`'teki örnek sayısıyla tanımlı süreler (ör. 312 örnek = "3 s") bu kartta %6.3 kısa sürer.
- T-RAW-07'nin "9.615 ms ± %2" eşiği çip toleransını hesaba katmıyor. Eşik değiştirilmedi; karar bekliyor.
- `tools/analyze.py gyro_turn` kayıt aralığını `seq / 104 Hz` ile hesaplıyor; bu kartta açıyı ≈ %6.3 fazla bulur. Düzeltilmedi.

**2. Y ekseninde manyetometre gürültüsü yüksek (T-RAW-05).** Akıştaki tek örnek std'si Y'de
1.35 mG (datasheet BW=01 için 0.6 mG RMS). Ofset iki ölçümün ortalaması olduğundan yalnızca bu
gürültüden ≈ 1349/√2 ≈ 950 µG ofset std'si beklenir; ölçülen 1190 µG. Yani T-RAW-05'teki sapmanın
büyük kısmı SET/RESET tekrarlanabilirliğinden değil, Y eksenindeki alan gürültüsünden geliyor
olabilir. Kaynağı (ortam alanı mı, çip mi) bu koşudan ayırt edilemez; kart metalden ve PC'den
uzakta tekrar ölçülmeli. 100 µs bekleme 1000 µs'tan belirgin kötü; `SR_SETTLE_US` kısaltılmamalı.

**3. Zamanlama bütçesi tutuyor ama payı dar.** Sensör thread'i tik süresi p99 ≤ 280 µs (sınır 300).

## İkinci koşu — sensör kartı düz (2026-10-12, commit `7573af6`)

Yalnızca `app.hw.raw_data.hub`, sensör kartı düz ve bileşen yüzü yukarı. 7 test durumundan 6'sı
geçti. Kanıt: `app.hw.raw_data.hub.flat.log`.

| Test ID | Sonuç | Not |
|---|---|---|
| T-RAW-00 | PASS | Init ofseti (−14282, 179748, 67138) µG |
| T-RAW-01, 02 | PASS | $\bar f_A$ = (0.193, −0.163, 10.160) m/s², std (4, 5, 4) mm/s²; $\bar f_{b,z}$ = −10.160 |
| T-RAW-03 | PASS | Ortalama (1.6, 2.3, 1.2) mrad/s, std (0.7, 0.6, 0.5) mrad/s |
| T-RAW-04 | PASS | $\bar m_M$ = (−78, 390, 443) mG, norm 0.595 G; std (0.66, 0.65, 1.19) mG |
| T-RAW-05 | PASS | std (580, 665, 744) µG, aralık (2015, 2380, 2319) µG |
| T-RAW-07 | **FAIL** | Ortalama Δt 8995 µs; diğer ölçütler sınırların içinde (tik p99 ≤ 280 µs, en yavaş 1415 µs, 10 SET tiki) |
| T-RAW-08 | PASS | 62 400 örnek, sayaçların hepsi 0 |

İlk koşudaki T-RAW-01 ve T-RAW-05 başarısızlıkları bu koşuda tekrarlanmadı. T-RAW-07 aynı değerle
kaldı; nedeni Bulgu 1'deki çip osilatörü sapmasıdır.

**Manyetik ortam kararsız.** Üç ölçümde alan ve köprü ofseti koşudan koşuya büyük değişti:
init ofsetinin X bileşeni +30.8 mG → −14.3 mG, Y bileşeni 139 mG → 180 mG; ana uygulamayla alınan
ara okumada alan normu 1.03 G idi, bu koşuda 0.595 G. Kartın duruşu da değiştiği için bunun ne
kadarının ortamdan geldiği ayrılamıyor, ama köprü ofseti duruştan bağımsız olmalıdır. T-RAW-04 ve
T-RAW-05'in bu koşuda geçmesi ortamın temiz olduğunu göstermez; manyetometre kalibrasyonundan
(T-RAW-11) önce ölçüm yeri doğrulanmalı.

Faz 2 hâlâ kapanmadı: T-RAW-07 ortalama Δt eşiği için karar bekleniyor, T-RAW-09…12 koşmadı.
