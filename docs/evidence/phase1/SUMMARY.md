# Faz 1 — Bus ve WHO_AM_I / PRODUCT_ID: SUMMARY

Koşu: 2026-10-12, git commit `e7f35e9`, NUCLEO-G491RE (ST-LINK V3J9M3, SN 0042004C3532510531333430),
ST-LINK doğrudan PC portunda, seri port `/dev/ttyACM1`.

```sh
west twister -T app/tests/hw/bus_id -p nucleo_g491re --device-testing --device-serial /dev/ttyACM1
```

Twister özeti: 2/2 konfigürasyon geçti, 16/16 test durumu geçti. `drivers` senaryosu, veri hızı ayarını basan bilgi testi eklendikten sonra yeniden koşturuldu (10/10); `app.hw.bus_id.drivers.log` o koşunun kaydıdır. Ham testler iki senaryoda da koştu.

| Test ID | Sonuç | Tarih | git commit | Not |
|---|---|---|---|---|
| T-BUS-01 | PASS | 2026-10-12 | e7f35e9 | İki `spi_dt_spec` hazır |
| T-BUS-02 | PASS | 2026-10-12 | e7f35e9 | ISM330DHCX WHO_AM_I 1000/1000 = `0x6B` |
| T-BUS-03 | PASS | 2026-10-12 | e7f35e9 | MMC5983MA PRODUCT_ID 1000/1000 = `0x30` |
| T-BUS-04 | PASS | 2026-10-12 | e7f35e9 | SW reset sonrası `OTP_Rd_Done` = 1, kimlik `0x30` |
| T-BUS-05 | PASS | 2026-10-12 | e7f35e9 | Sayımlar (11736, −740, 3580) ve (11748, −653, 3636); hepsi < 16384, doyma yok. Ham değerdir, köprü ofseti çıkarılmamıştır |
| T-BUS-06 | PASS | 2026-10-12 | e7f35e9 | `CTRL4_C` bit 2 = 1, bit 7/4/3/0 = 0 (`drivers` senaryosu) |
| T-BUS-07 | PASS | 2026-10-12 | e7f35e9 | 100 000 döngü, 0 SPI hatası, 0 kimlik uyuşmazlığı; 14.85 s ve 15.08 s, ≈ 33 400 işlem/s |
| T-BUS-08 | PASS | 2026-10-12 | e7f35e9 | `ism330dhcx` sürücüsü hazır |
| T-BUS-09 | PASS | 2026-10-12 | e7f35e9 | Mode 3: 100/100. Mode 0: 100/100 (yalnızca bilgi) |
| (bilgi) ISM330DHCX `INTERNAL_FREQ_FINE` | +45 | 2026-10-12 | çalışma ağacı | Çipin kendi osilatörü nominalden %+6.75 hızlı: 104 Hz ayarında gerçek periyot ≈ 8992 µs (≈ 111.2 Hz). Faz 2'de ölçülen 8995 µs ile tutarlı |
| T-BUS-10 | Yapılmadı | — | — | Lojik analizör ölçümü isteğe bağlı; SCK frekansı ölçülmedi |

Kanıt: `app.hw.bus_id.raw.log`, `app.hw.bus_id.drivers.log`, `twister.json`.

T-BUS-09 notu: MMC5983MA bu kartta iki modda da doğru yanıt veriyor. Mode 3 kararı
(`HARDWARE_MAP.md §3`, `MMC5983MA_SPEC.md §10 #8`) geçerliliğini koruyor.

Faz kapısı notu: `docs/evidence/phase0/jumper_check.md` yazılmadan bu faz koşturuldu. Jumper
kesimleri ve süreklilik ölçümleri bu kayıtta doğrulanmış sayılmaz; 100 000 döngülük hatasız soak
kablolamanın çalıştığını gösterir, `I2C_PU` ve `6DOF_ADDR` jumper'larının durumunu göstermez.
