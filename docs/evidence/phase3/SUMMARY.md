# Faz 3 — Ztest matematik testleri: SUMMARY

Koşu: 2026-10-12, git commit `cd3a070`, Zephyr `8076e9d493a` (4.4.99), Zephyr SDK 1.0.1.

```sh
west twister -T app/tests -p native_sim/native -p native_sim/native/64 \
	-p mps2/an385 -p mps3/corstone300/an547 -p nucleo_g491re
```

Twister özeti: 8 konfigürasyon koştu ve geçti, 160/160 test durumu geçti, 0 hata, 0 uyarı.
`nucleo_g491re` konfigürasyonları yalnızca derlendi.

| Test ID | Sonuç | Tarih | git commit | Not |
|---|---|---|---|---|
| T-MATH-01…10 (`ekf_math`, 10 test) | PASS | 2026-10-12 | cd3a070 | native_sim/native, native_sim/native/64, mps2/an385, mps3/corstone300/an547 |
| T-EKF-01…13 (`ekf_core`, 19 test) | PASS | 2026-10-12 | cd3a070 | Aynı dört platform. 13 plan testine ek olarak 6 kapı/argüman testi |
| T-HUB-01…06 (`axis_map`, `mag_cal`, `sensor_cal`, `mmc_decode`, 11 test) | PASS | 2026-10-12 | cd3a070 | Aynı dört platform |
| `app.ekf.build.nucleo`, `app.sensor_hub.build.nucleo` | Derlendi | 2026-10-12 | cd3a070 | `nucleo_g491re`, `-Werror` ile; hedefte koşturulmadı |

## Çıkış kriterleri

| Kriter | Durum | Kanıt |
|---|---|---|
| Tüm suite'ler dört platformda PASS, nucleo derleniyor | Sağlandı | `twister_report/twister.json`, `native_sim_ekf.log`, `native_sim_sensor_hub.log` |
| Satır kapsaması: `src/ekf/*.c` ≥ %90, `src/sensor_hub` saf modülleri ≥ %85 | Sağlandı: `ekf_core.c` %95, `ekf_math.c` %100, `sensor_cal.c` %100, `mmc5983ma_decode.h` %100 | `coverage_app.txt` (native_sim/native, gcovr) |
| `tools/ekf_reference.py` ile V1–V9 yeniden üretildi, `MATH_SPEC.md §15` ile fark yok | Sağlandı: en büyük sapma 5·10⁻⁸ (belgedeki 7 basamaklı yazım hassasiyeti) | `ekf_reference.txt` |
| Twister raporu `docs/evidence/phase3/` altında | Sağlandı | `twister_report/twister.json` |

`ekf_core.c` içinde kapsanmayan 15 satırın 14'ü `return -EDOM;` satırıdır (CMSIS-DSP çağrısının
başarısız olması, tekil $S_a$, $S_\psi$ alt sınırı, normalizasyon hatası), biri de
`state_is_sane()` içindeki sonlu olmayan durum dalıdır. `-EDOM` yolunun kendisi T-EKF-13 ile
(kovaryansa NaN enjeksiyonu) sınanır.

## Float64 referansla karşılaştırma

`ekf_reference.txt`, T-EKF-06…12 senaryolarını aynı gürültü üreteciyle float64'te koşar; hepsi
aynı eşiklerle PASS. Float32 ztest sonuçları da aynı eşikleri geçer. Senaryo başına sayısal
float32 değerleri ayrıca kaydedilmedi; ztest yalnızca eşik aşımında değer basar.
