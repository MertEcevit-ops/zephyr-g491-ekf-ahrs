# TEST_PLAN.md — Aşamalı Devreye Alma ve Ztest Senaryoları

| Alan | Değer |
|---|---|
| Amaç | Projeyi doğrulanabilir fazlara bölmek. Her fazın kapsamı, testleri, sayısal kabul eşikleri ve kanıt biçimi burada tanımlıdır |
| Bağlı belgeler | `HARDWARE_MAP.md`, `MATH_SPEC.md`, `MMC5983MA_SPEC.md`, `PROJECT_SPEC.md` |

---

## 0. Kurallar

### 0.1 Faz kapısı

1. Bir faz, tüm testleri **kanıtla birlikte** geçmeden "bitti" sayılmaz. Kanıt bir log, CSV veya ölçüm kaydıdır. "Derleniyor" veya "çalışıyor gibi" kanıt değildir.
2. Faz N+1'in kodu, Faz N kapanmadan yazılmaz ve birleştirilmez. Tek istisna: Faz 3 donanım gerektirmediği için Faz 1–2 ile paralel geliştirilebilir. Faz 4 için 1, 2 ve 3'ün üçü de kapalı olmalıdır.
3. PROJECT_SPEC'teki "yarım kod yasak" kuralı fazlar için de geçerlidir. Her faz, **kendi kapsamındaki** modülleri eksiksiz yazar. Kapsam, modül ekleyerek büyür; sonraki faza bırakılan `TODO`, boş fonksiyon gövdesi veya `-ENOTSUP` dönen yer tutucu olmaz.
4. Bir test başarısız olursa önce bu belgedeki ilgili sorun giderme tablosuna bakılır. Bir eşik ancak gerekçesiyle bu belgede güncellenerek değiştirilir; testi geçirmek için koddan gevşetilmez.

### 0.2 Faz kapsamları

| Faz | Bu fazda yazılan kod (tam ve üretime hazır) | Henüz var olmayan |
|---|---|---|
| 0 | — (donanım hazırlığı) | Her şey |
| 1 | `boards/nucleo_g491re.overlay`, `prj.conf`, kök `Kconfig`, `src/sensor_hub/ism330_spi_only.c`, `drivers/sensor/mmc5983ma/mmc5983ma_reg.h`, `tests/hw/bus_id/` | MMC sürücüsü, sensor_hub, EKF, thread'ler |
| 2 | MMC sürücüsünün tamamı, `mmc5983ma_decode.h`, `src/sensor_hub/` (toplama, Δt, kalibrasyon, eksen haritası, sensör thread'i), telemetri `raw`/`sensor` modları, `tools/capture.py`, `tools/analyze.py`, `tools/mag_calib.py`, `tests/hw/raw_data/`, `tests/sensor_hub/` | EKF, füzyon thread'i |
| 3 | `src/ekf/` (tamamı), `tests/ekf/`, `tools/ekf_reference.py` | Füzyon thread'i entegrasyonu |
| 4 | Füzyon thread'i, telemetri `fusion` modu, shell komutları, manyetometre ofset tazeleme politikası | — |

### 0.3 Test uygulamalarının proje kaynaklarını kullanması

Donanım testleri (`tests/hw/*`) ana uygulamanın overlay, binding ve sürücüsünü **kopyalamadan** kullanır:

```cmake
cmake_minimum_required(VERSION 3.20.0)
get_filename_component(PROJECT_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)
set(DTC_OVERLAY_FILE ${PROJECT_ROOT}/boards/nucleo_g491re.overlay)
list(APPEND ZEPHYR_EXTRA_MODULES ${PROJECT_ROOT})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hw_bus_id LANGUAGES C)

target_sources(app PRIVATE src/test_raw_bus.c)
target_include_directories(app PRIVATE ${PROJECT_ROOT}/drivers/sensor/mmc5983ma)
target_compile_options(app PRIVATE
	-Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion -Werror)
```

Sürücü, binding ve `include/` dizini uygulamanın Zephyr modülüyle gelir (`ZEPHYR_EXTRA_MODULES`, `MMC5983MA_SPEC.md §9.4`). `find_package(Zephyr)` sonrasında `add_subdirectory()` ile eklenen bir sürücü kütüphanesi bağlanmaz; bu kalıp kullanılmaz. `DTS_ROOT`'a ekleme de gerekmez, modülün `dts_root` ayarı yeterlidir.

Testin kendi `Kconfig` dosyası uygulama sembollerini paylaşır; sürücü sembolleri modülden gelir (`MMC5983MA_SPEC.md §9.3`):

```kconfig
rsource "../../../Kconfig.app"
source "Kconfig.zephyr"
```

`drivers` senaryosu, `HARDWARE_MAP.md §4` init sırasını test edebilmek için `${PROJECT_ROOT}/src/sensor_hub/ism330_spi_only.c` dosyasını da derler:

```cmake
if(CONFIG_ISM330DHCX)
  target_sources(app PRIVATE src/test_drivers.c ${PROJECT_ROOT}/src/sensor_hub/ism330_spi_only.c)
endif()
```

### 0.4 Test arayüzü gereksinimleri (firmware)

Faz 2 ve 4'teki elle yürütülen testler şu shell komutlarına dayanır. Bu komutlar ilgili fazın kapsamındadır:

| Komut | Faz | İşlev |
|---|---|---|
| `telemetry mode <raw\|sensor\|fusion\|off>` | 2 (raw, sensor), 4 (fusion) | §4.0 kayıt tipini seçer |
| `telemetry rate <1..50>` | 2 | Hz cinsinden çıkış hızı (desimasyon) |
| `mag offset` | 2 | Güncel köprü ofsetini basar (`MMC5983MA_ATTR_BRIDGE_OFFSET`) |
| `mag refresh` | 2 | Durağanlık kontrolüyle ofset tazeleme ister |
| `ekf align` | 4 | Füzyonu `ALIGNING` durumuna döndürür |
| `ekf stats` | 4 | `struct ekf_stats` ve durum bayraklarını basar |
| `ekf mag <on\|off>` | 4 | Manyetometre güncellemesini çalışma anında açar/kapatır (§4.2 testi için) |

### 0.5 Kanıt düzeni

```
docs/evidence/
├── phase0/jumper_check.md
├── phase1/<TEST-ID>.log
├── phase2/<TEST-ID>.{log,csv}
├── phase3/twister_report/        (twister.json, coverage)
├── phase4/<TEST-ID>.{csv,md}
└── phaseN/SUMMARY.md             test ID | sonuç | tarih | git commit | not
```

### 0.6 Araçlar

| Araç | Görev |
|---|---|
| `west build`, `west flash` | Varsayılan flash runner `stm32cubeprogrammer`'dır. Kurulu değilse `west flash --runner openocd` veya `--runner pyocd` |
| `west twister` | Ztest koşucusu. Donanımda: `--device-testing --device-serial /dev/ttyACM0 -p nucleo_g491re` |
| `tools/capture.py` | Seri porttan §4.0 kayıtlarını CSV'ye yazar: `--port --baud 115200 --type T --duration 600 --out …`. Kayda başlamadan firmware'i tipe uyan telemetri moduna alır (`--keep-mode` ile kapatılır); `--rate` verilirse `telemetry rate` gönderir |
| `tools/analyze.py` | Alt komutlar: `static`, `six_face`, `gyro_turn`, `mag_sign`, `accel_sign`, `step`, `timing`. Eşikleri bu belgeden alır, PASS/FAIL basar; `--report DOSYA.md` ile sonucu kanıt dosyası olarak yazar |
| `tools/telemetry_records.py` | §4.0 kayıt düzenleri; diğer araçların ortak modülü |

Araçlar `numpy` ve `pyserial` ister. Zephyr sanal ortamında `numpy` yoksa sistem Python'u kullanılır (`env -u PYTHONPATH /usr/bin/python3 tools/…`).
| `tools/mag_calib.py` | `MATH_SPEC.md §3.4` elipsoid uydurma, `mag_cal.h` üretir |
| `tools/ekf_reference.py` | `MATH_SPEC.md`'nin float64 referans uygulaması. §15 vektörlerini ve Faz 3 senaryo beklentilerini üretir |
| Lojik analizör (önerilir) | SCK/CS/MOSI/MISO ve INT zamanlaması |

---

## Faz 0 — Donanım hazırlığı

| ID | Adım | Kabul |
|---|---|---|
| T-HW-01 | `HARDWARE_MAP.md §7` jumper kesimleri | §7.1 ölçüm tablosunun tamamı beklenen aralıkta, `docs/evidence/phase0/jumper_check.md` yazıldı |
| T-HW-02 | Kablolama (`HARDWARE_MAP.md §2`) | Her sinyal için Nucleo pini ↔ breakout pini süreklilik ölçüldü (< 1 Ω); komşu pinler arasında kısa devre yok |
| T-HW-03 | Besleme | Nucleo USB'den beslenirken breakout `3V3`–`GND` arası 3.20–3.40 V; güç LED'i yanıyor (LED jumper'ı kesilmediyse) |
| T-HW-04 | Qwiic | Her iki Qwiic konnektörü boş |

---

## Faz 1 — Bus ve WHO_AM_I / PRODUCT_ID

**Amaç:** SPI bus'ının, iki CS hattının ve iki çipin elektriksel olarak doğru çalıştığını, EKF ve sürücü karmaşıklığından bağımsız olarak kanıtlamak.

**Uygulama:** `tests/hw/bus_id/`, iki senaryo:

```yaml
common:
  tags: hw
  platform_allow: nucleo_g491re
  harness: ztest
  timeout: 180
tests:
  app.hw.bus_id.raw:
    extra_configs:
      - CONFIG_SENSOR=n   # no drivers bound: pure SPI
  app.hw.bus_id.drivers:
    extra_configs:
      - CONFIG_SENSOR=y
      - CONFIG_ISM330DHCX_TRIGGER_NONE=y
```

Çalıştırma:

```sh
west twister -T app/tests/hw/bus_id -p nucleo_g491re --device-testing --device-serial /dev/ttyACM0
# veya tek senaryoyu elle (testcase.yaml'daki extra_configs uygulanır):
west build -b nucleo_g491re -d build/hw_bus_raw -T app.hw.bus_id.raw app/tests/hw/bus_id
west flash -d build/hw_bus_raw
```

`testcase.yaml` dosyalarında tab kullanılmaz: twister kabul eder ama `west build -T` YAML'ı ayrıştıramaz. `drivers` senaryosu ham testleri de koşar; böylece T-BUS-01…09 sürücüler bağlıyken de doğrulanır.

Ham SPI erişimi, sürücülerin kullandığı DT düğümlerinden alınır:

```c
#define BUS_OP (SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
static const struct spi_dt_spec ism = SPI_DT_SPEC_GET(DT_NODELABEL(ism330dhcx), BUS_OP);
static const struct spi_dt_spec mmc = SPI_DT_SPEC_GET(DT_NODELABEL(mmc5983ma), BUS_OP);
```

### Testler

| ID | Senaryo | Test | Kabul |
|---|---|---|---|
| T-BUS-01 | raw | `spi_is_ready_dt(&ism)` ve `spi_is_ready_dt(&mmc)` (bus ve CS GPIO'ları) | İkisi de `true` |
| T-BUS-02 | raw | ISM330DHCX `WHO_AM_I` (`0x0F`) 1000 kez oku | 1000/1000 = `0x6B` |
| T-BUS-03 | raw | MMC5983MA `PRODUCT_ID` (`0x2F`, komut `0xAF`) 1000 kez oku | 1000/1000 = `0x30` |
| T-BUS-04 | raw | MMC yazılımsal reset: W `0x0A = 0x80`, 10 ms bekle, R `STATUS` | Bit 4 (`OTP_Rd_Done`) = 1 ve ardından PRODUCT_ID = `0x30` |
| T-BUS-05 | raw | MMC tek ölçüm: W `0x09 = 0x01`, STATUS yokla, 7 bayt burst oku | ≤ 20 ms içinde `Meas_M_Done` = 1. Üç eksenin $\lvert s\rvert$ değeri < 16384 sayım (< 1 G, makul Dünya alanı + hard-iron). Hiçbir eksen doymamış |
| T-BUS-06 | drivers | ISM `CTRL4_C` (`0x13`) ham oku | Bit 2 (`I2C_disable`) = 1; bit 7, 4, 3, 0 = 0 (7, 4, 0 rezerv; 3 `DRDY_MASK`). `HARDWARE_MAP.md §4` init sırasının kanıtı |
| T-BUS-07 | raw | **Bus soak:** 100 000 döngü; her döngüde sırayla ISM WHO_AM_I, MMC PRODUCT_ID, ISM `0x0F–0x12` 4 bayt burst, MMC `0x00–0x06` 7 bayt burst, MMC W `CTRL0 = TM_M` | Kimlik uyuşmazlığı **0**, SPI hata dönüşü **0**. Süre ve işlem/s loglanır |
| T-BUS-08 | drivers | `device_is_ready(DEVICE_DT_GET(DT_NODELABEL(ism330dhcx)))` | `true` |
| T-BUS-09 | raw | MMC PRODUCT_ID'yi Mode 3 ve Mode 0 ile ayrı ayrı 100'er kez oku | Mode 3: 100/100. Mode 0 sonucu yalnızca loglanır. **Mode 3 başarısız, Mode 0 başarılıysa** faz durur; bulgu `HARDWARE_MAP.md §3` ve `MMC5983MA_SPEC.md §10 #8`'e işlenir ve karar yeniden verilir |
| T-BUS-10 | elle | Lojik analizör (önerilir): PRODUCT_ID okuması | SCK = 5.3125 MHz ±%1; CS düşükken SCK boşta yüksek (CPOL=1); MOSI ilk bayt `0xAF`; MISO ikinci bayt `0x30`; CS çerçevesi tam 16 saat |

T-BUS-07, `HARDWARE_MAP.md §4`'teki "ilk bayt kuralı"nın ve I²C kalıntısı analizinin deneysel kanıtıdır.

### Çıkış kriterleri

- T-BUS-01…09 iki senaryoda da PASS. T-BUS-10 yapıldıysa ekran görüntüsü kanıtta.
- `docs/evidence/phase1/SUMMARY.md` dolduruldu.

### Sorun giderme

| Belirti | Olası neden |
|---|---|
| WHO_AM_I = `0xFF` | MISO yüzüyor: `POCI` kablosu yok, `ACS` bağlı değil (çip I²C modunda kaldı) veya 3V3 yok |
| WHO_AM_I = `0x00` | MISO GND'ye kısa; SCK yok (PA5 başka bir sürücüye geçmiş: overlay'de `&pwm2` disabled mı, `CONFIG_LED=n` mi?) |
| `0xD6` veya `0x35` (beklenen `0x6B`'nin bir bit kaymışı) | SPI modu/faz hatası; overlay'deki `operation` bayraklarını kontrol et |
| ISM doğru, MMC `0x00`/`0xFF` | `MCS` kablosu (PC7 / D9) veya MMC'ye özgü mod sorunu (T-BUS-09) |
| MMC doğru, ISM yanlış | `ACS` kablosu (PB6 / D10) |
| İkisi de yanlış | SCK/MOSI yer değiştirmiş, pinctrl yanlış, ortak GND yok |
| Soak testinde seyrek hata | Kablo > 10 cm, GND dönüşü zayıf, `I2C_PU` kesilmemiş, Qwiic takılı. Geçici önlem: `spi-max-frequency = <5000000>` (2.656 MHz) ile tekrarla ve sonucu kaydet |
| T-BUS-06'da bit 2 = 0 | `SYS_INIT` önceliği yanlış veya `ism330dhcx` sürücüsü sonradan reset attı |

---

## Faz 2 — Ham veri, ölçekleme, zamanlama ve kalibrasyon

**Amaç:** Sensör verisinin mühendislik birimlerinde, doğru eksen işaretleriyle ve doğru zamanlamayla `sensor_hub`'dan çıktığını kanıtlamak; kalibrasyon katsayılarını üretmek.

**Uygulamalar:**

- `tests/hw/raw_data/` — kart **düz ve bileşen yüzü yukarı**, X oku kuzeye yakın, metal nesnelerden ≥ 30 cm uzakta, hareketsizken koşan otomatik ztest'ler (T-RAW-01…08)
- Ana uygulama, `telemetry mode raw|sensor` — elle yürütülen testler (T-RAW-09…12)

`tests/hw/raw_data` Kconfig'i: `CONFIG_SENSOR=y`, `CONFIG_ISM330DHCX_TRIGGER_OWN_THREAD=y`, `CONFIG_MMC5983MA=y` (DT'den varsayılan). `sensor_hub` modülü test içine derlenir ve uygulamadakiyle aynı yığın boyutu ve öncelikle çalıştırılır. Testler `sensor_hub_get_sample()` ile `struct imu_sample`, `sensor_hub_get_snapshot()` ile sensör çerçevesindeki kalibrasyonsuz değerleri ve sayaçları okur.

| Senaryo | Kapsam | Ek Kconfig |
|---|---|---|
| `app.hw.raw_data.hub` | T-RAW-00…05, 07, 08 | `CONFIG_APP_MAG_OFFSET_REFRESH_S=86400` (ofset tazeleme bir DRDY tiki yutar; zamanlama istatistiğine karışmasın) |
| `app.hw.raw_data.int_diag` | T-RAW-06 | `CONFIG_MMC5983MA_INT_DIAG=y`; sensor_hub çalıştırılmaz, sürücü doğrudan sürülür |
| `app.hw.raw_data.settle100` | T-RAW-00, 05 (bilgi amaçlı ek koşu) | `CONFIG_MMC5983MA_SR_SETTLE_US=100`, `CONFIG_RAW_DATA_OFFSET_ONLY=y` |

Test thread'i testler arasında kuyruğu okumaz; her ölçüm penceresi kuyruğu boşaltıp akış düzene girdikten sonra sayaçların taban değerini alır ve yalnızca pencere içindeki artışı değerlendirir.

### Otomatik testler (statik)

Her test 1040 örnek (10 s) toplar. Eşikler **kalibrasyonsuz** değerler içindir.

| ID | Test | Kabul |
|---|---|---|
| T-RAW-00 | `device_is_ready(mmc5983ma)`; sürücü init'i hatasız | `true`; loglarda `-ERANGE` uyarısı yok. Otomatik karşılığı: init sonrası köprü ofseti üç eksende birden tam sıfır değil (sürücü `-ERANGE`'de ofseti sıfır bırakır) |
| T-RAW-01 | İvme, A çerçevesi ortalama ve gürültü | $\lvert\bar f_x\rvert, \lvert\bar f_y\rvert < 0.8$ m/s²; $\bar f_z \in [9.0, 10.6]$; $\lVert\bar f\rVert \in [9.3, 10.3]$; her eksende std < 0.02 m/s² |
| T-RAW-02 | İvme, gövde çerçevesi işareti | $\bar f_{b,z} < -9.0$ (`MATH_SPEC.md §2.8` ilk satır) |
| T-RAW-03 | Jiroskop, durağan | Her eksende $\lvert\bar\omega\rvert < 0.06$ rad/s (datasheet maks. sıfır-hız seviyesi 3 dps) ve std < 0.005 rad/s |
| T-RAW-04 | Manyetometre, akla yatkınlık | $\lVert\bar m_M\rVert \in [0.2, 0.8]$ G; her eksende std < 2 mG (BW=01 spec 0.6 mG RMS); `-EBUSY` sayısı 0; `-ERANGE` sayısı 0 |
| T-RAW-05 | Ofset tekrarlanabilirliği: 1 s arayla 20 kez `MMC5983MA_ATTR_BRIDGE_OFFSET_REFRESH`, sonra `MMC5983MA_ATTR_BRIDGE_OFFSET` | Her eksende std < 1 mG (datasheet SET/RESET tekrarlanabilirliği 1 mG), maks.–min. < 4 mG. Ek koşu: `CONFIG_MMC5983MA_SR_SETTLE_US=100` ile tekrar (sonuç `MMC5983MA_SPEC.md §8.2`'ye bilgi olarak yazılır) |
| T-RAW-06 | Dönüşüm süresi, senaryo `CONFIG_MMC5983MA_INT_DIAG=y`: TM_M yazımından INT yükselişine kadar `k_cycle_get_32()` ile ölç (INT pini sıkı döngüde yoklanır), 200 ölçüm | BW=01'de maks. ≤ 6 ms; STATUS'a `0x01` yazıldıktan sonra INT düşük. Lojik analizör varsa ekran görüntüsü |
| T-RAW-07 | Zamanlama (`HARDWARE_MAP.md §10` bütçesi) | Ortalama Δt = çipin öngördüğü periyot ±%2 (aşağıdaki not); std(Δt) < 50 µs; `dt_fault` = 0; `msgq_drop` = 0; sensör thread'i tik süresi (DRDY zaman damgasından `k_msgq_put`'a) p99 < 300 µs (periyodik SET tikleri hariç, onlar < 1.5 ms). Ölçüm: kooperatif test thread'i kuyrukta bekler ve örnek kuyruğa girer girmez çalışır; o anda alınan döngü sayacı tik süresinin üst sınırıdır. ≥ 900 µs süren tikler SET tiki sayılır ve sayıları `pencere / PERIODIC_SET_INTERVAL + 2`'yi aşamaz. `msgq_drop` = `frame_drops + queue_drops` |
| T-RAW-08 | Uzun koşu (10 dk) | `dt_fault`, `msgq_drop`, SPI hatası = 0; MMC `-EBUSY` = 0 |

**T-RAW-07 beklenen periyot.** ISM330DHCX veri hızını kendi iç osilatöründen üretir; "104 Hz" ayarı nominal 6667 Hz / 64 = 104.17 Hz'dir (9.600 ms) ve gerçek hız çipten çipe değişir. Çip sapmasını `INTERNAL_FREQ_FINE` (0x63, işaretli, adım %0.15) register'ında bildirir. Test beklenen periyodu buradan hesaplar: $T = 9600\,\mu s / (1 + 0.0015 \cdot \text{FREQ\_FINE})$. Eski eşik sabit 9.615 ms'ydi ve çip toleransını hesaba katmıyordu. Bu projedeki kartta FREQ_FINE = +45 (%+6.75), öngörülen 8992 µs, ölçülen 8995 µs (`docs/evidence/phase2/SUMMARY.md`). ±%2 payı değişmedi. `tools/analyze.py timing --type R` için aynı değer `--expected-dt-us` ile verilir; `gyro_turn` zamanı kayıttaki ölçülmüş `dt_us`'tan alır ve hız varsaymaz.

### Elle yürütülen testler

**T-RAW-09 — Jiroskop işareti ve ölçeği**

Kart düz, bir gönye veya masa kenarına dayalı. Her eksen için pozitif yönde (sağ el kuralı, FRD) 90° döndür ve geri getir. `tools/analyze.py gyro_turn` integrali hesaplar.

| Hareket | Pozitif eksen | Beklenen |
|---|---|---|
| Yukarıdan bakınca saat yönünde dönüş | $\omega_{b,z}$ | $\int \omega_z\,dt = +1.571 \pm 0.05$ rad |
| X oku (burun) yukarı | $\omega_{b,y}$ | $+1.571 \pm 0.05$ rad |
| Header B tarafı (sağ kanat) aşağı | $\omega_{b,x}$ | $+1.571 \pm 0.05$ rad |

±0.05 rad, datasheet ±%2 hassasiyet artı el hizalama payıdır.

**T-RAW-10 — Eksen işaret testi**

*İvme (6 duruş):* Her duruşta 3 s bekle. `MATH_SPEC.md §2.8` tablosundaki $f_A$ ve $f_b$ işaretleri, baskın eksen > 0.9 g ve diğerleri < 0.15 g olacak şekilde sağlanmalı.

*Manyetometre (hard-iron'dan bağımsız fark yöntemi):* Kart düz. Telefon pusulasıyla X okunu sırasıyla K, D, G, B'ye çevir (her biri 3 s). Farklar hard-iron ofsetini yok eder:

| Ölçüm | Beklenen işaret | Anlamı |
|---|---|---|
| $m_{b,x}(K) - m_{b,x}(G)$ | > 0 (≈ $2B_h$) | $X_M$ ve $R_{bM}$'nin x satırı doğru |
| $m_{b,y}(D) - m_{b,y}(B)$ | < 0 (≈ $-2B_h$) | $Y_M$ ve $R_{bM}$'nin y satırı doğru |
| $m_{M,y}(D) - m_{M,y}(B)$ | > 0 | Ham çerçevede beklenen |
| $m_{b,z}(\text{düz}) - m_{b,z}(\text{ters, X kuzeye})$ | > 0 (kuzey yarımküre) | $Z_M$ aşağı ve $\det R_{bM} = -1$ doğru |

Bu tablonun herhangi bir satırı tutmazsa `MATH_SPEC.md §2.3` ve §2.6 yanlıştır. Faz durur, düzeltme önce belgeye sonra koda yapılır.

**T-RAW-11 — Manyetometre elipsoid kalibrasyonu**

1. `telemetry mode sensor`, `telemetry rate 20`.
2. `tools/capture.py --type S --duration 90`, kartı metal nesnelerden uzakta tüm yönlere yavaşça çevir.
3. `tools/mag_calib.py` (`MATH_SPEC.md §3.4`) — kapsama kriteri ve kabul kriterleri (std(‖m_cal‖)/B_ref < 0.02, cond(W) < 1.5).
4. Üretilen `mag_cal.h` commit edilir. CSV ve araç çıktısı kanıta eklenir.

**T-RAW-12 — İvmeölçer 6 yüz kalibrasyonu**

Kart, kenarları dik bir kutuya sabitlenir; 6 yüzün her birinde 5 s kayıt alınır (`--type S`). $o_a$ ve $S_a$, `MATH_SPEC.md §3.3`'e göre hesaplanır. Kabul: kalibrasyon sonrası 6 yüzün hepsinde $\bigl\lvert \lVert f_{cal}\rVert - g_0\bigr\rvert  < 0.02\,g_0$.

### Çıkış kriterleri

- T-RAW-00…12 PASS.
- `mag_cal.h` (`MAG_CAL_VALID 1`) ve ivme kalibrasyon sabitleri commit'li.
- `tests/sensor_hub` (Faz 3 tablosundaki T-HUB testleri) native_sim'de PASS.

### Sorun giderme

| Belirti | Olası neden |
|---|---|
| $f_z \approx -g$ (düz dururken) | Kart ters tutuluyor veya `ism_to_body` yerine ham değer okunuyor |
| $\lVert f\rVert \approx 2g$ ya da $g/2$ | `accel-range` DT ile sürücünün ölçek tablosu uyuşmuyor |
| Jiroskop integrali ≈ 57.3 kat fazla | dps ↔ rad/s karışıklığı |
| MMC sürekli `-EBUSY` | Boru hattı yeniden başlatılmıyor (`meas_in_flight` mantığı), BW=00 seçili veya TM_M yazımı gölgeyi bozuyor |
| MMC değerleri her ölçümde ≈ ±aynı büyüklükte işaret değiştiriyor | RESET sonrası SET yapılmıyor (`MMC5983MA_SPEC.md §8.3` adım 8) |
| Δt std > 50 µs | DRDY zaman damgası tetik işleyicisinde değil thread'de alınıyor; `CONFIG_PM=y`; log backend'i senkron |
| T-RAW-10 mag satırları tutarsız | Ölçüm masasında demir; telefon karta çok yakın; hard-iron çok büyük (önce T-RAW-11) |

---

## Faz 3 — Ztest matematik testleri (donanımsız)

**Amaç:** EKF ve saf yardımcı modüllerin `MATH_SPEC.md`'ye birebir uyduğunu, sentetik verilerle ve deterministik olarak kanıtlamak.

### 3.0 Düzen ve çalıştırma

```
tests/ekf/            suites: ekf_math, ekf_core
tests/sensor_hub/     suites: axis_map, mag_cal, sensor_cal, mmc_decode
```

`testcase.yaml` (her iki dizin için aynı yapı; `platform_allow` senaryo düzeyinde yazılır):

```yaml
common:
  tags: ekf
tests:
  app.ekf.unit:
    platform_allow:
      - native_sim/native
      - native_sim/native/64
      - mps2/an385
      - mps3/corstone300/an547
    integration_platforms:
      - native_sim/native
    timeout: 300
  app.ekf.build.nucleo:
    platform_allow: nucleo_g491re
    build_only: true
```

| Platform | Neden |
|---|---|
| `native_sim/native` | Hızlı geliştirme döngüsü, kapsama ölçümü. 32 bit host derlemesi `gcc-multilib` ister; yoksa `native_sim/native/64` |
| `mps2/an385` | Cortex-M3 QEMU, FPU yok: Thumb-2 ve yazılımsal float yolu |
| `mps3/corstone300/an547` | Cortex-M55 QEMU, donanım FPU: tek duyarlıklı donanım yolu |
| `nucleo_g491re` (yalnızca derleme) | Gerçek hedefte `-Wdouble-promotion -Werror` ile derleniyor mu |

```sh
west twister -T app/tests -p native_sim/native -p mps2/an385 -p mps3/corstone300/an547
west twister -T app/tests -p native_sim/native --coverage --coverage-platform native_sim/native \
	--coverage-tool gcovr --gcov-tool gcov -O build/twister-cov
west build -b native_sim/native app/tests/ekf -d build/test_ekf -t run      # tek koşu
```

Twister'ın kapsama raporu kökü `ZEPHYR_BASE`'dir ve ağaç dışındaki proje kaynaklarını dışarıda bırakır. Proje dosyalarının kapsaması aynı `.gcda` dosyalarından ayrıca çıkarılır:

```sh
gcovr -r . --gcov-executable gcov -f 'app/src/.*' -f 'app/drivers/.*' \
	--txt build/coverage_app.txt build/twister-cov/native_sim_native
```

`prj.conf`: `CONFIG_ZTEST=y`, `CONFIG_CMSIS_DSP=y`, `CONFIG_CMSIS_DSP_MATRIX=y`, `CONFIG_FPU=y` (FPU'lu platformlarda). Test kaynakları `src/ekf/*.c` dosyalarını doğrudan derler. Sabitler `ekf_config.h`'den gelir, testler kopyalamaz.

### 3.1 Simülasyon düzeneği kuralları (`tests/ekf/src/sim.c`)

1. **Gerçek değer test edilen koddan bağımsız üretilir.** Gerçek tutum, Euler açılarından testin kendi içinde yazılmış formüllerle kurulur. Gerçek ölçümler $R_z R_y R_x$ çarpımıyla hesaplanır. `ekf_math.c`'deki `R(q)` veya `quat_from_euler` gerçek değer üretiminde **kullanılmaz**.
2. Gürültü deterministiktir. `xorshift32`, tohum `0x12345678`; normal dağılım 12 uniform örneğin toplamı eksi 6 ile yaklaşıklanır:

```c
static uint32_t xorshift32(uint32_t *s)
{
	*s ^= *s << 13;
	*s ^= *s >> 17;
	*s ^= *s << 5;
	return *s;
}

static float randn(uint32_t *s)
{
	float acc = 0.0f;

	for (int i = 0; i < 12; i++) {
		acc += (float)(xorshift32(s) >> 8) * (1.0f / 16777216.0f);
	}
	return acc - 6.0f;
}
```

3. Sentetik ortam:

| Parametre | Değer |
|---|---|
| $\Delta t$ | 1/104 s |
| Jiroskop gürültüsü σ | 6.3·10⁻⁴ rad/s (5 mdps/√Hz × √52 Hz) |
| İvme gürültüsü σ | 4.2·10⁻³ m/s² (60 µg/√Hz × √52 Hz) |
| Manyetometre gürültüsü σ | 2·10⁻³ G |
| Gerçek bias | (0.01, −0.02, 0.015) rad/s |
| Manyetik alan | $m_n = (0.25, 0, 0.40)$ G, $D = 0$, $I = 57.99°$, $B_{ref} = 0.4717$ G |

4. Hata metriği: Euler farkı (roll, pitch doğrudan; yaw `wrap` edilerek), derece cinsinden.
5. `tools/ekf_reference.py` aynı senaryoları float64'te koşar. Float32 sonuçları referansın tolerans bandında olmalıdır.

### 3.2 `ekf_math` suite

| ID | Test | Kabul |
|---|---|---|
| T-MATH-01 | Kuaterniyon çarpımı: $q\otimes[1,0,0,0] = q$; $i\otimes j = k$; $q\otimes q^* = [1,0,0,0]$ | 1·10⁻⁶ |
| T-MATH-02 | Normalizasyon: $[2,0,0,0]\to[1,0,0,0]$; sıfır kuaterniyon | Sıfır için `-EDOM` |
| T-MATH-03 | Euler→q ve R(q): `MATH_SPEC.md §15` V1, V2 | 1·10⁻⁵ |
| T-MATH-04 | Gidiş-dönüş ızgarası: roll, yaw ∈ [−170°, 170°], pitch ∈ [−80°, 80°], 10° adım | Maks. hata < 2·10⁻⁴ rad |
| T-MATH-05 | $R(q)v$ ile $q\otimes[0,v]\otimes q^*$, 100 rastgele $(q, v)$ | 1·10⁻⁵ |
| T-MATH-06 | Üstel artım: V7; $\vartheta = 10^{-9}$ (NaN yok); x etrafında π → $[0,1,0,0]$; $10^{-3}$ eşiğinin iki yanında süreklilik | 1·10⁻⁶; eşik farkı < 1·10⁻⁷ |
| T-MATH-07 | Jacobian'lar: V6 değerleri + 50 rastgele $q$'da merkezi sonlu fark (adım 1·10⁻³) | V6: 1·10⁻⁴; sonlu fark: 2·10⁻³ |
| T-MATH-08 | $q\otimes[0,w] = \Omega(w)q = \Xi(q)w$; $\Xi(q)^Tq = 0$ | 1·10⁻⁶ |
| T-MATH-09 | Pitch = ±90° kuaterniyonundan Euler | NaN yok; $\lvert\theta\rvert = \pi/2 \pm 10^{-3}$ |
| T-MATH-10 | `wrap`: $\pi+0.1 \to -\pi+0.1$; $-\pi \to \pi$ | Aralık $(-\pi, \pi]$, 1·10⁻⁶ |

### 3.3 `ekf_core` suite

| ID | Test | Kabul |
|---|---|---|
| T-EKF-01 | `ekf_init` (V1, $b_0=0$, durağan) | $\mathrm{tr}(P_{qq}) = 3\sigma_{q0}^2$ (bağıl 1·10⁻⁵); $P_{qq}q \approx 0$; $\mathrm{diag}(P_{bb}) = \sigma_{b0}^2$ |
| T-EKF-02 | `ekf_align`: V3 + V4 → V5; `mag_ok = false` → yaw = 0 | 1·10⁻⁴ rad |
| T-EKF-03 | V8: 208 tahmin adımı, güncelleme yok | $\psi = 1.0 \pm 10^{-4}$ rad |
| T-EKF-04 | 10 000 tahmin, rastgele $\lVert\omega\rVert < 5$ rad/s | $\bigl\lvert \lVert q\rVert - 1\bigr\rvert  < 10^{-6}$; $\max\lvert P-P^T\rvert \le 10^{-6}\max\lvert P\rvert$; $\mathrm{diag}(P) \ge 0$; $\lVert P_{qq}q\rVert < 10^{-6}\lVert P_{qq}\rVert$ |
| T-EKF-05 | Girdi doğrulama: NaN jiroskop; $\Delta t = 0$; $\Delta t = 0.06$; $\lVert f\rVert < 10^{-3}$; $\lVert m\rVert < 10^{-6}$ | Hepsi `-EINVAL`; filtre durumu bayt düzeyinde değişmemiş (`memcmp`) |
| T-EKF-06 | Durağan yakınsama: gerçek (10°, −5°, 120°), gerçek tutumdan başla, $b_0 = 0$, hareketli $P_0$, 30 s | Roll/pitch hatası < 0.1°, yaw < 0.3°; her eksende bias hatası < 5·10⁻⁴ rad/s; $\mathrm{diag}(P_{bb}) < 10^{-6}$ |
| T-EKF-07 | Dinamik: roll 20°·sin(2π·0.2t), pitch 15°·sin(2π·0.13t), yaw 30°/s; 30 s; hizalamayla başla | 5 s sonrası RMS < 0.5°, maks. < 1.5° (her eksen); `acc_rejected + mag_rejected` < örneklerin %1'i |
| T-EKF-08 | Doğrusal ivme: (a) $x_b$ boyunca 0.5 g, t=10–12 s; (b) $x$: 0.3 g·sin(2π·1t), $y$: 0.2 g·cos(2π·0.7t), t=10–15 s | (a) roll/pitch sapması < 0.5°, `tilt_resets` = 0. (b) roll/pitch sapması < 3° |
| T-EKF-09 | Manyetik bozulma: (a) $m_b$'ye +0.3 G $x_b$, t=10–15 s; (b) aynı bozulma t ≥ 10 s kalıcı, 60 s | (a) yaw sapması < 1°, `yaw_resets` = 0. (b) `yaw_resets` tam olarak 1 ve reset anı 39–42 s arası (`MATH_SPEC.md §9` durum makinesi) |
| T-EKF-10 | Kötü başlangıç: roll/pitch doğru, yaw 0° (gerçek 120°), `heading_converged = false` | `yaw_resets` = 1; t = 5 s'de yaw hatası < 1°; t = 10 s'de `heading_converged = true` |
| T-EKF-11 | Heading→roll sızıntısı: gerçek (20°, −15°, 50°), filtre yaw 49°, tek mag güncellemesi, başlangıç $P$ | $\lvert\Delta\phi\rvert, \lvert\Delta\theta\rvert < 0.25°$; $\Delta\psi > 0$ (doğru yöne) |
| T-EKF-12 | Jiroskop doyması: t=10 s'de 0.35 s'de 360° roll (1028 dps), ölçülen jiroskop ±8.727 rad/s'de kırpılmış | `gyro_sat` > 0; hareket bittikten (t=10.35 s) 0.5 s sonra tüm eksenlerde hata < 2° |
| T-EKF-13 | Sayısal bozulma enjeksiyonu: $P_{11}$'e NaN yaz, `ekf_predict` çağır | `-EDOM`; `numeric_faults` = 1; `state = ALIGNING` |

Eşikler, `MATH_SPEC.md §16`'daki float64 prototip sonuçlarının üzerine float32 ve farklı gürültü dizisi için 3–25 kat pay bırakılarak seçilmiştir.

**Senaryo kurulumları** (tabloda açık bırakılan noktalar; `tests/ekf/src/test_ekf_core.c` ve `tools/ekf_reference.py` aynısını uygular):

| Test | Kurulum |
|---|---|
| T-EKF-07 | Hizalama ilk örnekten yapılır (`ekf_align`), $b_0 = 0$, hareketli $P_0$: kart baştan beri hareket halindedir |
| T-EKF-08, 09, 12 | Füzyon thread'inin yaptığı gibi başlar: 100 örneklik pencere ortalaması → `ekf_align` → `ekf_init(durağan, b_0 = \bar\omega)`. "Sapma", $t \ge 10$ s (T-EKF-12'de $t \ge 10.85$ s) için gerçek değere göre en büyük mutlak hatadır |
| T-EKF-08, 09, 10 | Gerçek tutum (10°, −5°, 120°). T-EKF-09 için bu seçim zorunludur: $x_b$ boyunca +0.3 G bozulma bu tutumda alanın normunu (%7.8 sapma, kapı %15) ve eğimini (1.1° sapma, kapı 10°) kapıların içinde bırakır, böylece bozulmayı yalnızca NIS kapısı görür. Başka heading'lerde (0°, 45°, ±90°, 180° için norm sapması %22–47 veya eğim sapması > 10°) aynı bozulma norm ya da eğim kapısına takılır (`mag_gated`), ret sayacı ilerlemez ve §9 durum makinesi sınanmamış olur. T-EKF-09(a) bu yüzden `mag_rejected > 0` da ister |
| T-EKF-10 | $b_0$ 100 örneklik jiroskop ortalamasıdır (durağan $P_0$); süreler filtrenin başlatıldığı andan ölçülür |
| T-EKF-12 | Taban tutum (0°, 0°, 30°); hata $t \in [10.85, 12.85]$ s aralığının tamamında < 2° |

Tablodaki 13 testin yanında `ekf_core` suite'i §8–§9 kapı yollarını da sınar: sert ve yumuşak ivme kapısı, 520 ret sonrası tilt reset, manyetometre norm/eğim/tekillik/etkin kapıları, `ekf_init` ve `ekf_align` argüman hataları.

### 3.4 `sensor_hub` suite

| ID | Test | Kabul |
|---|---|---|
| T-HUB-01 | `ism_to_body`: $(0,0,+9.80665) \to (0,0,-9.80665)$; $(1,2,3) \to (1,-2,-3)$ | Tam eşitlik |
| T-HUB-02 | `mmc_to_body`: $(1,2,3) \to (1,-2,3)$ | Tam eşitlik |
| T-HUB-03 | Eksen haritalarının determinantı | `ism_to_body` = +1, `mmc_to_body` = −1 |
| T-HUB-04 | `mag_cal_apply`: $V = (0.1, -0.05, 0.02)$, $W = \mathrm{diag}(1.1, 0.9, 1.0)$, $m = (0.6, 0.15, 0.42)$ → $(0.55, 0.18, 0.40)$ | 1·10⁻⁶ |
| T-HUB-05 | `mmc5983ma_decode_18bit` + `counts_to_ug`: `MMC5983MA_SPEC.md §6.5` vektörü; doyma vektörü | Tam eşitlik; doyma `true` |
| T-HUB-06 | Δt hesabı: $t_{k-1} = 0xFFFF0000$, $t_k = 0x0018F0A0$ (taşma); aralık dışı Δt | $\Delta t = 1\,700\,000 / 170\,\text{MHz} = 10$ ms ± 1 ns; aralık dışında `-EINVAL` ve çıktı yazılmaz |

### Çıkış kriterleri

- Tüm suite'ler dört platformda (nucleo yalnızca derleme) PASS.
- `native_sim/native` satır kapsaması: `src/ekf/*.c` ≥ %90, `src/sensor_hub` saf modülleri ≥ %85.
- `tools/ekf_reference.py` ile V1–V9 yeniden üretildi; `MATH_SPEC.md §15` ile fark yok.
- Twister raporu `docs/evidence/phase3/`.

---

## Faz 4 — Dinamik entegrasyon (donanım)

**Amaç:** Tam sistemin gerçek el hareketlerinde gürültü, kayma, doğruluk, tepki ve kaynak kullanımını ölçmek.

### 4.0 Telemetri biçimi

Tüm değerler ölçeklenmiş tamsayılardır (`MATH_SPEC.md §12`: `printk` ile float basılmaz). Satır başına bir kayıt, virgülle ayrılmış, ilk alan kayıt tipi.

| Tip | Alanlar |
|---|---|
| `R` (raw, gövde çerçevesi, kalibre) | `R,seq,dt_us,ax_mm_s2,ay,az,gx_urad_s,gy,gz,mx_ug,my,mz,mag_valid` |
| `S` (sensör çerçevesi, kalibrasyonsuz) | `S,seq,fax_mm_s2,fay,faz,wax_urad_s,way,waz,mmx_ug,mmy,mmz,mag_valid` |
| `T` (füzyon) | `T,seq,t_ms,state,roll_mdeg,pitch_mdeg,yaw_mdeg,q0_e6,q1_e6,q2_e6,q3_e6,bgx_urad_s,bgy,bgz,acc_soft,acc_gated,acc_rejected,mag_gated,mag_rejected,yaw_resets,tilt_resets,gyro_sat,numeric_faults,dt_fault,msgq_drop,ekf_cyc,lat_cyc,flags` |

`state`: 0 `ALIGNING`, 1 `RUNNING`. `seq`: `R` ve `S` kayıtlarında füzyon kuyruğuna verilen örnek sayısı, `T` kaydında füzyonun işlediği örnek sayısı (104 Hz'de artar; telemetri desimasyonunda atlar). `R` kaydındaki `dt_us` örnek aralığıdır, kayıt aralığı değildir. `msgq_drop` = `frame_drops + queue_drops`. `flags`: bit 0 `heading_converged`, bit 1 `yaw_valid`, bit 2 bu tikte mag güncellemesi kabul edildi. `ekf_cyc`: tahmin + güncellemelerin döngü sayısı. `lat_cyc`: DRDY zaman damgasından EKF çıkışına kadar geçen döngü. Kuaterniyon telemetride $q_0 \ge 0$ olacak şekilde kanonikleştirilir.

115200 baud'da `T` kaydı (~150 karakter) en fazla 50 Hz ile basılır (≈ %65 hat doluluğu).

### 4.1 Testler

Her testten önce: güç verilir, kart 10 s düz ve hareketsiz bırakılır (hizalama ve bias yakınsaması), `ekf stats` kaydedilir.

| ID | Prosedür | Kabul |
|---|---|---|
| T-DYN-01 | **Statik gürültü:** 60 s hareketsiz | Std: roll, pitch < 0.05°; yaw < 0.2° |
| T-DYN-02 | **Statik kayma, mag açık:** 10 dk hareketsiz | İlk ve son dakika ortalamaları farkı: roll/pitch < 0.2°, yaw < 1.0° |
| T-DYN-03 | **Statik kayma, mag kapalı** (`ekf mag off`, 60 s yakınsamadan sonra): 10 dk | Yaw kayması < 5° (bilgi amaçlı; bias kestiriminin kalitesini gösterir); roll/pitch < 0.2° |
| T-DYN-04 | **Tilt doğruluğu:** düz yüzey (su terazisiyle) ve bilinen açılı bir blok (ör. 30°). Her yüzeyde kart 0° ve 180° heading'de ölçülür; ortalama yüzey eğimini yok eder | Mutlak hata < 1.0° (ivme kalibrasyonundan sonra) |
| T-DYN-05 | **Göreli heading:** düz yüzeyde gönyeye dayalı 4 × 90° adım | Her adım 90° ± 2°; 360° sonunda kapanma hatası < 2° |
| T-DYN-06 | **Adım tepkisi:** yaw ekseninde 90° hızlı dönüş (≈ 0.5 s), gönyeye dayalı | Son değer 90° ± 2°; hareket bittikten sonra ($\lVert\omega\rVert < 0.05$ rad/s) son değerin ±1°'sine oturma < 0.5 s; aşma < 1° |
| T-DYN-07 | **Tepki gecikmesi:** ani dönüş başlangıcı | Jiroskopun $\lVert\omega\rVert > 0.2$ rad/s eşiğini geçtiği örnekten yaw'ın 1° değiştiği örneğe ≤ 3 örnek (≈ 29 ms); `lat_cyc` p99 < 340 000 (2 ms) |
| T-DYN-08 | **Başlangıca dönüş:** 60 s rastgele el hareketi (hızlı bilek hareketleri dahil), sonra başlangıç fikstürüne geri koy | 5 s içinde roll/pitch < 1°, yaw < 2° |
| T-DYN-09 | **Doğrusal ivme reddi:** kartı masada düz tutarak 10 s ileri-geri kaydır; ayrıca masaya sert tıklat | Roll/pitch sapması < 2°; tıklatma geçici sapması < 1°; `acc_soft + acc_gated + acc_rejected` arttı; `tilt_resets` = 0 |
| T-DYN-10 | **Manyetik bozulma:** tornavida veya küçük mıknatısı karta ≈ 5 cm'ye 5 s yaklaştır, uzaklaştır | Bozulma sırasında yaw sapması < 3° (yatay ve zayıf bozulmalar `MATH_SPEC.md §16`'ya göre ≈ 6°'ye kadar çıkabilir; sonuç ve mesafe kaydedilir); `mag_gated + mag_rejected` arttı; uzaklaştırdıktan sonra 2 s içinde bozulma öncesi heading'in ±1°'sine dönüş; `yaw_resets` artmadı |
| T-DYN-11 | **Ofset tazeleme:** `mag refresh` hareketsizken; hareket halindeyken | Hareketsizken kabul (log), yaw sıçraması < 0.5°; hareket halindeyken reddedilir |
| T-DYN-12 | **Zamanlama ve kaynak:** 10 dk karışık kullanım. `CONFIG_THREAD_ANALYZER=y`, `CONFIG_SCHED_THREAD_USAGE=y` | `ekf_cyc` p99 < 170 000 (1 ms); `msgq_drop`, `dt_fault`, `numeric_faults` = 0; her thread yığın kullanımı < %75; toplam CPU < %30 |
| T-DYN-13 | **Uzun koşu:** 1 saat masada | `numeric_faults` = 0, `tilt_resets` = 0, `yaw_resets` ≤ 1 (yalnızca açılış), `heading_converged` sürekli 1 |

### 4.2 Analiz

```sh
python tools/capture.py --port /dev/ttyACM0 --type T --duration 600 \
	--out docs/evidence/phase4/T-DYN-02.csv
python tools/analyze.py static docs/evidence/phase4/T-DYN-02.csv
```

`analyze.py` her test için bu belgedeki eşikleri kullanır ve PASS/FAIL ile ölçülen değerleri `T-DYN-xx.md` olarak yazar.

### Çıkış kriterleri

- T-DYN-01…13 PASS (T-DYN-03 bilgi amaçlı; sonuç kaydedilir).
- `docs/evidence/phase4/SUMMARY.md` dolduruldu.

### Sorun giderme

| Belirti | Olası neden |
|---|---|
| Durağanken yaw yavaşça kayıyor, `mag_rejected` artıyor | Kalibrasyon eski veya masada demir; `MAG_CAL_FIELD_NORM_G` yanlış; inclination Kconfig'i konuma uymuyor |
| Yaw 90° dönüşte 80° veya 100° gösteriyor | Soft-iron kalibrasyonu yetersiz kapsama ile yapılmış (T-RAW-11 tekrarı) |
| Roll, yaw değiştikçe salınıyor | Eksen haritası hatası (T-RAW-10) veya ivme ofseti kalibre edilmemiş |
| Hızlı bilek hareketinden sonra tilt birkaç saniye hatalı | `gyro_sat` artmıyor mu? `EKF_GYRO_SAT` ile `gyro-range` uyumsuz |
| `heading_converged` 0'da kalıyor | `MAG_CAL_VALID = 0` veya `use_mag = false`; norm kapısı sürekli tetikleniyor (`B_ref`) |
| `ekf_cyc` beklenenden 10 kat fazla | FPU kapalı (`CONFIG_FPU=n`) veya bir yerde `double` sızıyor (`-Wdouble-promotion` bayrağı hedefe eklenmemiş) |

---

## 5. Regresyon matrisi

Bir dosya değiştiğinde yeniden koşulması zorunlu testler:

| Değişen | Yeniden koşulacak |
|---|---|
| `boards/nucleo_g491re.overlay`, `prj.conf` (SPI, GPIO, sensör) | Faz 1 tamamı, T-RAW-01…08 |
| `drivers/sensor/mmc5983ma/*` | T-BUS-03…05, T-BUS-07, T-RAW-00, T-RAW-04…08, T-HUB-05 |
| `src/sensor_hub/axis_map.h` | T-HUB-01…03, T-RAW-10, Faz 4 tamamı |
| `src/sensor_hub/mag_cal.h` (yeni kalibrasyon) | T-RAW-11 kabul kriterleri, T-DYN-05, T-DYN-10 |
| `src/ekf/*`, `ekf_config.h` | Faz 3 tamamı, T-DYN-01, 02, 06, 08, 09, 10, 12 |
| Thread öncelikleri, yığın boyutları, msgq derinliği | T-RAW-07, T-DYN-12, T-DYN-13 |
| Zephyr sürümü yükseltmesi | Her şey |
