# MMC5983MA_SPEC.md — Özel Sürücü Register Kılavuzu

| Alan | Değer |
|---|---|
| Çip | MEMSIC MMC5983MA, 3 eksen AMR manyetometre, LGA-16 3×3 mm |
| Kaynak | MEMSIC MMC5983MA Rev A datasheet (resmi yayın 4/3/2019), SparkFun MMC5983MA Arduino kütüphanesi (referans uygulama) |
| Kod karşılığı | `drivers/sensor/mmc5983ma/`, `dts/bindings/sensor/memsic,mmc5983ma.yaml`, `include/app/drivers/sensor/mmc5983ma.h` |
| Bağlı belgeler | `HARDWARE_MAP.md` (bus, pin), `MATH_SPEC.md §2.3, §3.4` (eksen, kalibrasyon), `TEST_PLAN.md` |

Upstream Zephyr'de MMC5983MA sürücüsü **yoktur**; yalnızca farklı register haritasına sahip `memsic,mmc56x3` vardır ve kopyalanmaz. Bu dosyadaki adres, bit ve zamanlama değerleri datasheet'ten okunmuş, belirsiz noktalar SparkFun referans kütüphanesiyle çapraz kontrol edilmiştir. Burada olmayan bir register veya bit kullanılmaz. Datasheet'in kendi içindeki çelişkiler ve alınan kararlar §10'dadır.

---

## 1. Özet

| Özellik | Değer |
|---|---|
| Besleme | VDD ve VDDIO 2.8–3.6 V (projede 3.3 V) |
| Ölçüm aralığı | ±8 G |
| Çözünürlük | 18 bit: 16384 sayım/G (0.0625 mG/LSB); 16 bit: 4096 sayım/G (0.25 mG/LSB) |
| Sıfır alan çıkışı | 131072 (18 bit), 32768 (16 bit). Çıkış **işaretsiz** |
| Gürültü (RMS) | BW=00: 0.4 mG, 01: 0.6 mG, 10: 0.8 mG, 11: 1.2 mG |
| Arayüz | SPI 4 telli, en fazla 10 MHz, Mode 3; I²C adresi 0x30 (bu projede kullanılmaz) |
| CS | Aktif-düşük. Breakout'ta 100 kΩ ile 3V3'e çekili |
| INT | Aktif-yüksek. `INT_meas_done_en` set edilene kadar hi-Z |
| Açılış / yazılımsal reset süresi | 10 ms |
| PRODUCT_ID | `0x2F` adresinde `0x30` |
| Rahatsız edici alan | > 10 G'ye maruz kalırsa SET/RESET gerekir |
| Eksen hizalama hatası | Tipik 1°, maks. 3° |

---

## 2. SPI protokolü

### 2.1 Komut baytı

Datasheet bitleri **telden çıkış sırasına göre** numaralar ("bit 0" ilk gönderilen bittir). MSB-first SPI'da telden ilk çıkan bit, komut baytının bit 7'sidir. Kodda kullanılacak karşılık:

| Datasheet ("bit n", zaman sırası) | Komut baytı biti | Anlamı |
|---|---|---|
| bit 0 | **bit 7** | R/W: `1` = okuma, `0` = yazma |
| bit 1 | bit 6 | Don't care → her zaman `0` gönderilir |
| bit 2–7 | bit 5:0 | Register adresi AD[5:0] |
| bit 8–15 | 2. bayt | Veri, MSB first |

```c
#define MMC5983MA_SPI_READ	BIT(7)
#define MMC5983MA_SPI_ADDR_MASK	GENMASK(5, 0)
```

Örnek: PRODUCT_ID okuma komutu `0x80 | 0x2F = 0xAF`, yanıt `0x30`.

Bu kodlama SparkFun kütüphanesindeki `READ_REG(x) (0x80 | x)` ile birebir aynıdır. Adresler 6 bittir; `0x3F` üstü bir adres yazılamaz.

### 2.2 Çerçeveler

| İşlem | MOSI | MISO | Saat sayısı |
|---|---|---|---|
| Tek okuma | `[0x80\|a] [0x00]` | `[xx] [veri]` | 16 |
| Burst okuma, N bayt | `[0x80\|a] [0x00]×N` | `[xx] [a] [a+1] …` | 8 + 8N |
| Tek yazma | `[a] [veri]` | yok sayılır | 16 |

- Okumada çip, SDO'yu 8. saatten itibaren sürer; ilk alınan bayt çöptür.
- Datasheet burst okumada adres artışını I²C için açıkça yazar, SPI için "adding blocks of 8 clocks" der. SparkFun referans sürücüsü `0x00–0x06` aralığını (7 bayt) tek CS çerçevesinde okur. Bu projede burst **yalnızca `0x00–0x06` aralığında** kullanılır; STATUS (0x08) ayrı okunur.
- Çoklu bayt **yazma** kullanılmaz; her register ayrı çerçevede yazılır.
- 3 telli SPI (`CTRL3.Spi_3w`) kullanılmaz.

### 2.3 Zephyr uygulama kalıbı

```c
/* The pinned Zephyr tree has no SPI_OP_MODE_CONTROLLER; the controller mode is ..._MASTER. */
#define MMC5983MA_SPI_OPERATION \
	(SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

static int mmc5983ma_read(const struct spi_dt_spec *bus, uint8_t reg, uint8_t *dst, size_t len)
{
	uint8_t cmd = MMC5983MA_SPI_READ | (reg & MMC5983MA_SPI_ADDR_MASK);
	const struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf rx_bufs[] = {
		{ .buf = NULL, .len = 1 },	/* clocked in while the command byte goes out */
		{ .buf = dst, .len = len },
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs) };

	return spi_transceive_dt(bus, &tx, &rx) ? -EIO : 0;
}

static int mmc5983ma_write(const struct spi_dt_spec *bus, uint8_t reg, uint8_t val)
{
	uint8_t frame[2] = { reg & MMC5983MA_SPI_ADDR_MASK, val };
	const struct spi_buf tx_buf = { .buf = frame, .len = sizeof(frame) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	return spi_write_dt(bus, &tx) ? -EIO : 0;
}
```

- TX tamponu RX'ten kısa olduğunda Zephyr SPI sürücüsü eksik baytları dummy olarak gönderir. Bu, upstream `ism330dhcx_spi.c` ile aynı kalıptır.
- SPI spec'i `SPI_DT_SPEC_INST_GET(inst, MMC5983MA_SPI_OPERATION)` ile alınır (iki argümanlı güncel imza).
- `len` en fazla 7 olabilir. Daha büyük istek `-EINVAL` döner.

---

## 3. Register haritası

| Adres | Ad | Erişim | Reset | İçerik |
|---|---|---|---|---|
| `0x00` | Xout0 | R | — | Xout[17:10] |
| `0x01` | Xout1 | R | — | Xout[9:2] |
| `0x02` | Yout0 | R | — | Yout[17:10] |
| `0x03` | Yout1 | R | — | Yout[9:2] |
| `0x04` | Zout0 | R | — | Zout[17:10] |
| `0x05` | Zout1 | R | — | Zout[9:2] |
| `0x06` | XYZout2 | R | — | Xout[1:0], Yout[1:0], Zout[1:0] |
| `0x07` | Tout | R | — | Sıcaklık (kullanılmaz) |
| `0x08` | Status | R/W | `0x00` | Ölçüm bitti bayrakları, OTP durumu |
| `0x09` | Internal Control 0 | **W** | `0x00` | Ölçüm tetikleme, SET/RESET, INT |
| `0x0A` | Internal Control 1 | **W** | `0x00` | Bant genişliği, kanal kapatma, SW reset |
| `0x0B` | Internal Control 2 | **W** | `0x00` | Sürekli mod, periyodik SET (kullanılmaz) |
| `0x0C` | Internal Control 3 | **W** | `0x00` | 3 telli SPI, self-test akımları (kullanılmaz) |
| `0x2F` | Product ID 1 | R | `0x30` | Kimlik |

`0x0D–0x2E` aralığına erişilmez.

---

## 4. Bit tanımları

Makrolar `drivers/sensor/mmc5983ma/mmc5983ma_reg.h` dosyasında tanımlanır.

### 4.1 XYZout2 (`0x06`)

| Bit | 7:6 | 5:4 | 3:2 | 1:0 |
|---|---|---|---|---|
| Alan | Xout[1:0] | Yout[1:0] | Zout[1:0] | 0 |

### 4.2 Status (`0x08`, R/W)

| Bit | Ad | Makro | Anlamı |
|---|---|---|---|
| 0 | Meas_M_Done | `MMC5983MA_STATUS_MEAS_M_DONE` = `BIT(0)` | Manyetik ölçüm bitti. Yeni ölçüm komutuyla 0 olur, bir sonraki ölçüme kadar 1 kalır. **1 yazmak INT'i temizler** |
| 1 | Meas_T_Done | `MMC5983MA_STATUS_MEAS_T_DONE` = `BIT(1)` | Sıcaklık ölçümü bitti (kullanılmaz) |
| 4 | OTP_Rd_Done | `MMC5983MA_STATUS_OTP_READ_DONE` = `BIT(4)` | Fabrika OTP verisi okundu. Reset sonrası 1 olmalı |
| 7:5, 3:2 | — | — | Rezerv |

### 4.3 Internal Control 0 (`0x09`, yalnızca yazma)

| Bit | Ad | Makro | Tip | Anlamı |
|---|---|---|---|---|
| 0 | TM_M | `MMC5983MA_CTRL0_TM_M` = `BIT(0)` | Strobe | Manyetik ölçüm başlat; bitince kendiliğinden 0 olur |
| 1 | TM_T | `MMC5983MA_CTRL0_TM_T` = `BIT(1)` | Strobe | Sıcaklık ölçümü. TM_M ile aynı anda 1 **olamaz** (kullanılmaz) |
| 2 | INT_meas_done_en | `MMC5983MA_CTRL0_INT_MEAS_DONE_EN` = `BIT(2)` | **Kalıcı** | Ölçüm bitince INT pinini sürer |
| 3 | Set | `MMC5983MA_CTRL0_SET` = `BIT(3)` | Strobe | Bobinden 500 ns SET akımı; kendiliğinden temizlenir |
| 4 | Reset | `MMC5983MA_CTRL0_RESET` = `BIT(4)` | Strobe | Bobinden 500 ns RESET akımı; kendiliğinden temizlenir |
| 5 | Auto_SR_en | `MMC5983MA_CTRL0_AUTO_SR_EN` = `BIT(5)` | Kalıcı | Otomatik SET/RESET (**kullanılmaz**, §8.5) |
| 6 | OTP Read | `MMC5983MA_CTRL0_OTP_READ` = `BIT(6)` | Strobe | OTP'yi yeniden oku (kullanılmaz) |
| 7 | — | — | — | Rezerv, 0 |

### 4.4 Internal Control 1 (`0x0A`, yalnızca yazma)

| Bit | Ad | Makro | Anlamı |
|---|---|---|---|
| 1:0 | BW[1:0] | `MMC5983MA_CTRL1_BW_MASK` = `GENMASK(1, 0)` | Desimasyon filtresi, yani ölçüm süresi (§7) |
| 2 | X-inhibit | `MMC5983MA_CTRL1_X_INHIBIT` = `BIT(2)` | X kanalını kapatır (kullanılmaz) |
| 4:3 | YZ-inhibit | `MMC5983MA_CTRL1_YZ_INHIBIT` = `GENMASK(4, 3)` | İki bit birlikte Y ve Z kanallarını kapatır (kullanılmaz) |
| 6:5 | — | — | Rezerv, 0 |
| 7 | SW_RST | `MMC5983MA_CTRL1_SW_RST` = `BIT(7)` | Açılış gibi reset: tüm register'lar sıfırlanır, OTP yeniden okunur, 10 ms sürer |

### 4.5 Internal Control 2 (`0x0B`, yalnızca yazma) — bu projede hep `0x00`

| Bit | Ad | Makro | Anlamı |
|---|---|---|---|
| 2:0 | Cm_freq[2:0] | `MMC5983MA_CTRL2_CM_FREQ_MASK` | Sürekli mod hızı: 000 kapalı, 001 1 Hz, 010 10 Hz, 011 20 Hz, 100 50 Hz, 101 100 Hz, 110 200 Hz (BW=01), 111 1000 Hz (BW=11) |
| 3 | Cmm_en | `MMC5983MA_CTRL2_CMM_EN` = `BIT(3)` | Sürekli ölçüm modu (Cm_freq ≠ 000 gerektirir) |
| 6:4 | Prd_set[2:0] | `MMC5983MA_CTRL2_PRD_SET_MASK` = `GENMASK(6, 4)` | Kaç ölçümde bir otomatik SET: 1, 25, 75, 100, 250, 500, 1000, 2000 |
| 7 | En_prd_set | `MMC5983MA_CTRL2_EN_PRD_SET` = `BIT(7)` | Periyodik SET (Auto_SR_en ve Cmm_en gerektirir) |

### 4.6 Internal Control 3 (`0x0C`, yalnızca yazma) — bu projede hep `0x00`

| Bit | Ad | Makro | Anlamı |
|---|---|---|---|
| 1 | St_enp | `MMC5983MA_CTRL3_ST_ENP` = `BIT(1)` | Self-test bobin akımı (+ → −) |
| 2 | St_enm | `MMC5983MA_CTRL3_ST_ENM` = `BIT(2)` | Self-test bobin akımı (− → +) |
| 6 | Spi_3w | `MMC5983MA_CTRL3_SPI_3W` = `BIT(6)` | 3 telli SPI |
| 7, 5:3, 0 | — | — | Rezerv |

### 4.7 Product ID 1 (`0x2F`)

`MMC5983MA_REG_PRODUCT_ID = 0x2F`, beklenen değer `MMC5983MA_PRODUCT_ID = 0x30`.

---

## 5. Yalnızca yazılabilir kontrol register'ları — gölge kuralı

`0x09–0x0C` register'ları **geri okunamaz**. Oku-değiştir-yaz yapılamaz; sürücü her biri için RAM'de bir gölge kopya tutar.

1. Gölge register yalnızca **kalıcı** bitleri tutar. Bu projede bu bitler `CTRL0.INT_meas_done_en` ve `CTRL1.BW` alanıdır.
2. Strobe bitleri (`TM_M`, `TM_T`, `SET`, `RESET`, `OTP_READ`, `SW_RST`) gölgeye **asla** yazılmaz. Gölgede kalırsa sonraki her CTRL0 yazımı istenmeyen bir ölçüm veya SET tetikler.
3. Her CTRL0 yazımı şu biçimdedir: `ctrl0_shadow | STROBE`. `INT_meas_done_en` açıkken yalnızca `TM_M` yazmak kesmeyi kapatır. SparkFun kütüphanesi de aynı nedenle gölge bellek kullanır.
4. `SW_RST` sonrası tüm gölgeler `0x00` olur, sonra init sırasında yeniden kurulur.

```c
static inline int mmc5983ma_ctrl0_strobe(const struct device *dev, uint8_t strobe)
{
	const struct mmc5983ma_config *cfg = dev->config;
	struct mmc5983ma_data *data = dev->data;

	return mmc5983ma_write(&cfg->bus, MMC5983MA_REG_CTRL0, data->ctrl0 | strobe);
}
```

---

## 6. Veri formatı ve dönüşüm

### 6.1 18 bit okuma (projede kullanılan)

`0x00–0x06` arası 7 bayt tek burst'te okunur (`b[0..6]`):

```c
x = ((uint32_t)b[0] << 10) | ((uint32_t)b[1] << 2) | ((b[6] >> 6) & 0x3);
y = ((uint32_t)b[2] << 10) | ((uint32_t)b[3] << 2) | ((b[6] >> 4) & 0x3);
z = ((uint32_t)b[4] << 10) | ((uint32_t)b[5] << 2) | ((b[6] >> 2) & 0x3);
```

İşaretli sayım: $s = u - 131072$ (aralık $[-131072, +131071]$).

### 6.2 16 bit okuma (yalnızca bilgi)

`0x00–0x05` arası 6 bayt okunur: $u_{16} = (b_0 \ll 8) \mid b_1$, sıfır = 32768, 4096 sayım/G. $u_{16} = u_{18} \gg 2$ olduğundan 16 bit mod ayrı bir çip ayarı **değildir**; yalnızca XYZout2 baytının okunmamasıdır. BW=01'de gürültü (0.6 mG) 16 bit LSB'sinden (0.25 mG) büyüktür, yani 16 bitle kayıp küçüktür. Bu proje tutarlılık için 18 bit kullanır; maliyeti işlem başına 1 bayttır.

### 6.3 Birim dönüşümü (tamsayı, float yok)

Zephyr `SENSOR_CHAN_MAGN_*` birimi Gauss'tur.

$$1\ \text{sayım} = \frac{10^6}{16384}\ \mu\text{G} = \frac{15625}{256}\ \mu\text{G}\ (\text{tam})$$

```c
int64_t ug = ((int64_t)counts * 15625) / 256;
val->val1 = (int32_t)(ug / 1000000);
val->val2 = (int32_t)(ug % 1000000);	/* same sign as val1, as sensor_value requires */
```

### 6.4 Doyma

Herhangi bir eksende $u = 0$ veya $u = 262143$ ise o eksen doymuştur (|alan| ≥ 8 G). Davranış:

1. O tikin verisi **güncellenmez**, `sample_fetch` `-ERANGE` döner.
2. Sonraki ölçümden önce zorunlu SET yapılır (`force_set = true`). Datasheet: 10 G'yi aşan alan sonrası sensör karakteristiğini geri getirmek için SET/RESET gerekir.

### 6.5 Saf fonksiyonlar (birim test edilebilir)

`drivers/sensor/mmc5983ma/mmc5983ma_decode.h` dosyası yalnızca `static inline` fonksiyonlar içerir. SPI ve Zephyr bağımlılığı yoktur; `tests/sensor_hub` doğrudan include eder.

```c
static inline void mmc5983ma_unpack_18bit(const uint8_t raw[7], uint32_t unsigned_counts[3]);
static inline void mmc5983ma_decode_18bit(const uint8_t raw[7], int32_t counts[3]);
static inline bool mmc5983ma_raw_saturated(const uint8_t raw[7]);
static inline int64_t mmc5983ma_counts_to_ug(int32_t counts);
```

`mmc5983ma_unpack_18bit` §6.1'deki işaretsiz $u$ değerlerini verir; diğer ikisi onun üzerine kuruludur.

**Test vektörü:**

| Girdi `b[0..6]` | `90 00 7F FF 78 00 30` |
|---|---|
| $u$ | (147456, 131071, 122880) |
| $s$ (sayım) | (+16384, −1, −8192) |
| µG | (1 000 000, −61, −500 000) |
| `sensor_value` | X {1, 0}, Y {0, −61}, Z {0, −500000} |
| Doyma | Hayır |

Doyma vektörü: `00 00 80 00 80 00 00` → X için $u = 0$ → doymuş.

---

## 7. Ölçüm süresi ve bant genişliği

| BW[1:0] | Register tablosundaki ölçüm süresi | Bant genişliği | Spec tablosu maks. ODR | RMS gürültü |
|---|---|---|---|---|
| 00 | 8 ms (güç bölümünde 10 ms) | 100 Hz | 50 Hz | 0.4 mG |
| **01** | **4 ms** | 200 Hz | **100 Hz** | **0.6 mG** |
| 10 | 2 ms | 400 Hz | 225 Hz | 0.8 mG |
| 11 | 0.5 ms | 800 Hz | 580 Hz | 1.2 mG |

**Karar: `CONFIG_MMC5983MA_BANDWIDTH = 1` (BW=01).**

- Sensör thread'i 104 Hz'de çalışır (9.615 ms). BW=00'ın 8–10 ms süresi ve 50 Hz ODR sınırı bu periyoda sığmaz.
- BW=01 4 ms ile ≥ 2× pay bırakır ve spec tablosundaki 100 Hz ODR ile uyumludur.
- `src/sensor_hub/` derleme zamanında bunu zorlar:

```c
BUILD_ASSERT(CONFIG_MMC5983MA_BANDWIDTH >= 1,
	     "BW=00 needs up to 10 ms per measurement and cannot keep up with 104 Hz");
```

Gerçek dönüşüm süresi Faz 2'de INT tanı moduyla ölçülür (`TEST_PLAN.md` T-RAW-06); kabul eşiği ≤ 6 ms.

---

## 8. SET / RESET bobin döngüsü

### 8.1 Neden

AMR sensörlerin ince film mıknatıslanması büyük alanlarla (> 10 G) veya zamanla bozulur; bu da histerezis ve ofset kaymasına yol açar. SET ve RESET, iç bobinden kısa ve yüksek bir akım geçirerek mıknatıslanmayı iki zıt yönden birine zorlar:

- **SET** sonrası çıkış: $O_1 = +H + O_{köprü}$
- **RESET** sonrası çıkış: $O_2 = -H + O_{köprü}$

Buradan:

$$H = \frac{O_1 - O_2}{2},\qquad O_{köprü} = \frac{O_1 + O_2}{2}$$

Köprü ofseti sıcaklıkla kayar. Datasheet, SET/RESET ile düzeltildiğinde −40…105 °C aralığında sıfır alan kaymasını 3 mG olarak verir. Datasheet'teki eksen işaretleri SET sonrası polariteye göredir (`MATH_SPEC.md §2.3`). Bu yüzden normal ölçümler **her zaman SET durumunda** alınır.

### 8.2 Zamanlama sabitleri

| Sabit | Değer | Kaynak |
|---|---|---|
| SET / RESET darbe süresi | **0.5 µs** | Datasheet (CTRL0 Set/Reset açıklaması) |
| SET/RESET sonrası bekleme `CONFIG_MMC5983MA_SR_SETTLE_US` | **1000 µs** | Datasheet süre vermiyor. SparkFun referans sürücüsü 1 ms bekliyor. Proje bu güvenli değeri benimser |
| Açılış / SW_RST sonrası bekleme | **10 000 µs** | Datasheet: "The power on time is 10 mS" |
| Ölçüm süresi, BW=01 | **4000 µs** (Faz 2'de ölçülür) | §7 |
| Durum yoklama aralığı | **500 µs** | Proje |
| Ölçüm zaman aşımı `CONFIG_MMC5983MA_MEAS_TIMEOUT_MS` | **20 000 µs** | En kötü BW=00 süresinin 2 katı |

Bekleme süresinin 1000 µs'tan kısaltılması yalnızca `TEST_PLAN.md` T-RAW-05 ofset tekrarlanabilirlik testi kısa süreyle de geçerse yapılır; sonuç bu tabloya yazılır.

### 8.3 Köprü ofseti kalibrasyon sekansı

**Ne zaman:** init sırasında ve `MMC5983MA_ATTR_BRIDGE_OFFSET_REFRESH` istendiğinde. İşlem bloklayıcıdır, toplam ≈ 11 ms sürer. `sensor_hub` bunu yalnızca kart durağanken çağırır (§9.9).

| Adım | SPI işlemi | Sonraki bekleme |
|---|---|---|
| 0 | `meas_in_flight` ise STATUS'u Meas_M_Done=1 olana kadar yokla (sonuç atılır) | ≤ 20 ms, aşılırsa `-ETIMEDOUT` |
| 1 | W `CTRL0 = ctrl0 \| SET` | 1000 µs |
| 2 | W `CTRL0 = ctrl0 \| TM_M` | STATUS yokla (500 µs aralık, 20 ms zaman aşımı) |
| 3 | R burst `0x00–0x06` → $s_{SET}$ | — |
| 4 | W `CTRL0 = ctrl0 \| RESET` | 1000 µs |
| 5 | W `CTRL0 = ctrl0 \| TM_M` | STATUS yokla |
| 6 | R burst `0x00–0x06` → $s_{RESET}$ | — |
| 7 | `offset[i] = (s_SET[i] + s_RESET[i]) / 2` | — |
| 8 | W `CTRL0 = ctrl0 \| SET` (mıknatıslanmayı SET durumuna geri getir) | 1000 µs |
| 9 | W `CTRL0 = ctrl0 \| TM_M` (boru hattını yeniden başlat), `meas_in_flight = true`, `meas_since_set = 0` | — |

- Adım 3 veya 6'da doyma görülürse ofset güncellenmez, `-ERANGE` döner, adım 8–9 yine yapılır.
- Tamsayı bölmesinin sıfıra doğru yuvarlaması en fazla 0.5 sayım (31 µG) hata yapar; 0.6 mG gürültünün yanında ihmal edilir.
- $|offset| > 16384$ sayım (1 G) ise `LOG_WRN` basılır ama değer kabul edilir. Datasheet beklenen köprü ofseti aralığını net vermez.

### 8.4 Periyodik SET

Her `CONFIG_MMC5983MA_PERIODIC_SET_INTERVAL` (varsayılan 100, yani ≈ 1 s) ölçümde bir veya `force_set` ise, `sample_fetch` yeni ölçümü başlatmadan önce:

| Adım | SPI işlemi | Bekleme |
|---|---|---|
| 1 | W `CTRL0 = ctrl0 \| SET` | 1000 µs (`k_usleep`, CPU'yu bırakır) |
| 2 | W `CTRL0 = ctrl0 \| TM_M` | — |

RESET yapılmaz. Periyodik SET mıknatıslanmayı tazeler; köprü ofseti §8.3 ile güncellenir.

### 8.5 Neden sürekli mod ve Auto_SR kullanılmıyor

| Seçenek | Sorun |
|---|---|
| Sürekli mod (`Cmm_en`) | Çipte blok veri güncelleme (BDU) yok. Çip, 7 baytlık burst okuma sırasında çıkış register'larını güncelleyebilir; bu da yırtık örnek demektir. Ayrıca ölçüm anı MCU'nun IMU saatinden bağımsız kayar |
| `Auto_SR_en` + `En_prd_set` | Yalnızca sürekli modda çalışır, yalnızca SET yapar (köprü ofsetini ölçmez). Davranışı datasheet'te ayrıntılı tanımlanmamış |
| Projenin seçimi | Tek-atım modu, IMU DRDY'sine kilitli boru hattı: ölçüm her zaman host tarafından başlatılır ve bitmeden okunmaz. Yırtılma yapısal olarak imkânsız, zamanlama deterministik |

### 8.6 Alan hesabı

Normal ölçüm SET durumundadır:

$$h_i = s_i - \texttt{offset}[i]\quad(\text{sayım, M çerçevesi}),\qquad \mu G_i = h_i \cdot 15625 / 256$$

Hard-iron/soft-iron düzeltmesi sürücüde **yapılmaz**; `sensor_hub` uygular (`MATH_SPEC.md §3.4`).

---

## 9. Sürücü mimarisi

### 9.1 Dosyalar

```
dts/bindings/sensor/memsic,mmc5983ma.yaml
drivers/sensor/mmc5983ma/
├── CMakeLists.txt
├── Kconfig
├── mmc5983ma.c          init, sample_fetch, channel_get, attr_set/attr_get
├── mmc5983ma.h          config/data struct'ları, iç prototipler
├── mmc5983ma_reg.h      §3–§4 adres ve bit makroları
└── mmc5983ma_decode.h   §6.5 saf fonksiyonlar
include/app/drivers/sensor/mmc5983ma.h   özel attribute enum'u (uygulamaya açık)
```

Binding dosyası sürücü dizininde değil `dts/bindings/sensor/` altındadır: Zephyr binding'leri yalnızca bir DTS kökünün `dts/bindings/` dizinlerinde arar. Uygulama dizini hem uygulamanın kendi DTS köküdür hem de `zephyr/module.yml` içindeki `dts_root: .` ayarıyla modülün DTS köküdür; ikincisi sayesinde `tests/hw/*` uygulamaları da binding'i bulur.

### 9.2 Devicetree binding

```yaml
description: MEMSIC MMC5983MA 3-axis AMR magnetometer on SPI

compatible: "memsic,mmc5983ma"

include: [sensor-device.yaml, spi-device.yaml]

properties:
  int-gpios:
    type: phandle-array
    description: |
      INT output, active high, hi-Z until INT_meas_done_en is set.
      Only configured when CONFIG_MMC5983MA_INT_DIAG=y; needs a pull-down.
```

### 9.3 Kconfig

```kconfig
menuconfig MMC5983MA
	bool "MEMSIC MMC5983MA magnetometer (SPI)"
	default y
	depends on SENSOR
	depends on DT_HAS_MEMSIC_MMC5983MA_ENABLED
	select SPI

if MMC5983MA

config MMC5983MA_INIT_PRIORITY
	int "Init priority"
	default 92
	help
	  Must run after the ISM330DHCX I2C_disable hook
	  (CONFIG_APP_ISM330_SPI_ONLY_INIT_PRIORITY). See HARDWARE_MAP.md sec. 4.

config MMC5983MA_BANDWIDTH
	int "BW[1:0] decimation setting"
	range 0 3
	default 1
	help
	  0: 8 ms, 1: 4 ms, 2: 2 ms, 3: 0.5 ms per measurement.

config MMC5983MA_PERIODIC_SET_INTERVAL
	int "Measurements between SET pulses"
	range 1 2000
	default 100

config MMC5983MA_SR_SETTLE_US
	int "Wait after SET/RESET before measuring (us)"
	default 1000

config MMC5983MA_MEAS_TIMEOUT_MS
	int "Polling timeout for a single measurement (ms)"
	default 20

config MMC5983MA_INT_DIAG
	bool "Drive INT on measurement done (logic analyzer aid)"
	select GPIO

endif # MMC5983MA
```

Sürücü Kconfig'i modül üzerinden gelir: `zephyr/module.yml` → `kconfig: drivers/Kconfig` → `rsource "sensor/mmc5983ma/Kconfig"`. Uygulamanın ve testlerin kök `Kconfig` dosyaları onu ayrıca `rsource` **etmez**; aynı semboller iki kez tanımlanırdı.

Uygulamaya ait Kconfig sembolleri `Kconfig.app` dosyasında toplanır. Böylece donanım testleri (`tests/hw/*`) aynı sembolleri kopyalamadan `rsource` edebilir.

```kconfig
# Kconfig (uygulama kökü)
mainmenu "9-DoF EKF"

rsource "Kconfig.app"

source "Kconfig.zephyr"
```

```kconfig
# Kconfig.app (ilgili kısım; diğer APP_* sembolleri PROJECT_SPEC'te)
config APP_ISM330_SPI_ONLY_INIT_PRIORITY
	int "ISM330DHCX I2C_disable hook priority"
	default 91
```

### 9.4 CMake

Sürücü bir Zephyr modülü olarak bağlanır. `find_package(Zephyr)` çağrısından **sonra** `add_subdirectory()` ile oluşturulan bir `zephyr_library` bağlanmaz: Zephyr kütüphane listesini `find_package` sırasında kapatır, kütüphane derlenir ama imaja girmez ve cihaz nesnesi sessizce kaybolur. Modülün CMake dosyaları ise `find_package` içinde işlenir.

```yaml
# zephyr/module.yml
name: imu-ekf
build:
  cmake: drivers
  kconfig: drivers/Kconfig
  settings:
    dts_root: .
```

```cmake
# drivers/CMakeLists.txt
zephyr_include_directories(../include)
add_subdirectory_ifdef(CONFIG_MMC5983MA sensor/mmc5983ma)
```

```cmake
# drivers/sensor/mmc5983ma/CMakeLists.txt
zephyr_library_named(mmc5983ma)
zephyr_library_sources(mmc5983ma.c)
target_compile_options(mmc5983ma PRIVATE
	-Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion -Werror)
```

```cmake
# CMakeLists.txt (uygulama kökü ve tests/hw/*; find_package(Zephyr)'den ÖNCE)
list(APPEND ZEPHYR_EXTRA_MODULES <app dizini>)
```

Doğrulama: `zephyr.elf` içinde `mmc5983ma_init` sembolü ve cihazın `__init___device_dts_ord_<N>` girdisi bulunmalı, init sırası `ism330dhcx` → `ism330_disable_i2c` → `mmc5983ma` olmalıdır.

Uyarı bayrakları `zephyr_compile_options()` ile **global verilmez**. Global verilirse Zephyr çekirdek kaynakları da etkilenir ve `-Werror` ile derleme kırılabilir. Bayraklar yalnızca projenin kendi hedeflerine (`app`, `mmc5983ma`) eklenir.

### 9.5 Veri yapıları

```c
struct mmc5983ma_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec int_gpio;	/* .port == NULL when absent */
};

struct mmc5983ma_data {
	struct k_mutex lock;
	uint8_t ctrl0;			/* persistent bits only (sec. 5) */
	uint8_t ctrl1;
	int32_t offset[3];		/* bridge offset, counts, frame M */
	int32_t field_ug[3];		/* last field, micro-gauss, frame M */
	uint16_t meas_since_set;
	bool meas_in_flight;		/* a TM_M has been issued and not yet read */
	bool force_set;
};
```

Örnek tanımlama:

```c
#define DT_DRV_COMPAT memsic_mmc5983ma

#define MMC5983MA_DEFINE(inst)							\
	static struct mmc5983ma_data mmc5983ma_data_##inst;			\
	static const struct mmc5983ma_config mmc5983ma_config_##inst = {	\
		.bus = SPI_DT_SPEC_INST_GET(inst, MMC5983MA_SPI_OPERATION),	\
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),	\
	};									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, mmc5983ma_init, NULL,		\
				     &mmc5983ma_data_##inst,			\
				     &mmc5983ma_config_##inst, POST_KERNEL,	\
				     CONFIG_MMC5983MA_INIT_PRIORITY,		\
				     &mmc5983ma_api);

DT_INST_FOREACH_STATUS_OKAY(MMC5983MA_DEFINE)
```

API tablosu `static DEVICE_API(sensor, mmc5983ma_api) = { ... };` ile tanımlanır. Bu makro bir `struct sensor_driver_api` üretir ve upstream sürücülerin güncel kalıbıdır.

### 9.6 Init sırası

| # | İşlem | Hata |
|---|---|---|
| 1 | `spi_is_ready_dt(&cfg->bus)` | `-ENODEV` |
| 2 | `k_mutex_init(&data->lock)` | — |
| 3 | W `CTRL1 = SW_RST`, sonra `k_msleep(10)` | `-EIO` |
| 4 | R `PRODUCT_ID`, `0x30` olmalı | `-EIO` (SPI) / `-ENODEV` (yanlış kimlik; değer loglanır) |
| 5 | R `STATUS`, `OTP_Rd_Done` (bit 4) = 1 olmalı | `-EIO` |
| 6 | `ctrl1 = FIELD_PREP(BW_MASK, CONFIG_MMC5983MA_BANDWIDTH)`, W `CTRL1` | `-EIO` |
| 7 | `CONFIG_MMC5983MA_INT_DIAG` ise: `int_gpio` hazır mı, `gpio_pin_configure_dt(GPIO_INPUT)`, `ctrl0 \|= INT_MEAS_DONE_EN` | `-ENODEV` / `-EIO` |
| 8 | §8.3 köprü ofseti sekansı (boru hattını başlatır) | `-ETIMEDOUT` / `-EIO` init'i başarısız kılar. `-ERANGE` (açılışta yakında mıknatıs) init'i durdurmaz: `LOG_WRN` basılır, `offset = 0` kalır, init `0` döner. `sensor_hub`'ın ilk durağan refresh'i ofseti düzeltir |

`CTRL2` ve `CTRL3` reset değerinde (`0x00`) bırakılır ve yazılmaz.

### 9.7 `sample_fetch` — boru hattı algoritması

Kanal `SENSOR_CHAN_ALL` veya `SENSOR_CHAN_MAGN_XYZ` olmalı, aksi halde `-ENOTSUP`. Tüm gövde `data->lock` altında çalışır.

```
if (!meas_in_flight):
    W CTRL0 = ctrl0 | TM_M ; meas_in_flight = true
    return -EBUSY                       # stale data must never be returned

R STATUS
if (!(STATUS & MEAS_M_DONE)):
    return -EBUSY                       # measurement still running; retry next tick

R burst 0x00..0x06 → raw[7]
meas_in_flight = false
if (INT_DIAG): W STATUS = MEAS_M_DONE   # release INT

if (saturated(raw)):
    force_set = true ; rc = -ERANGE
else:
    field_ug = counts_to_ug(decode(raw) - offset) ; rc = 0

if (++meas_since_set >= PERIODIC_SET_INTERVAL || force_set):
    W CTRL0 = ctrl0 | SET ; k_usleep(SR_SETTLE_US)
    meas_since_set = 0 ; force_set = false

W CTRL0 = ctrl0 | TM_M ; meas_in_flight = true
return rc
```

- Bir SPI hatasında anında `-EIO` döner. `meas_in_flight` false kalırsa bir sonraki çağrı boru hattını yeniden başlatır.
- `meas_in_flight` bayrağı gereklidir. `Meas_M_Done` bir sonraki ölçüm komutuna kadar 1 kaldığı için, yalnızca STATUS'a bakan bir sürücü aynı örneği iki kez döndürür.
- Normal tik maliyeti 3 SPI işlemidir (STATUS, burst, TM_M); bloklama yoktur. Periyodik SET tikinde fazladan 1 ms uyku vardır.

### 9.8 `channel_get`

- `SENSOR_CHAN_MAGN_X/Y/Z/XYZ` desteklenir; diğerleri `-ENOTSUP`.
- Değerler **sensör çerçevesi M**'dedir (eksen eşlemesi `sensor_hub`'da yapılır) ve §6.3'e göre `sensor_value`'ya çevrilir.

### 9.9 Özel attribute'lar

`include/app/drivers/sensor/mmc5983ma.h`:

```c
#include <zephyr/drivers/sensor.h>

enum mmc5983ma_attribute {
	/* attr_set, SENSOR_CHAN_MAGN_XYZ: run the blocking SET/RESET offset sequence */
	MMC5983MA_ATTR_BRIDGE_OFFSET_REFRESH = SENSOR_ATTR_PRIV_START,
	/* attr_get, SENSOR_CHAN_MAGN_X/Y/Z (one value) or MAGN_XYZ (three): bridge offset in gauss */
	MMC5983MA_ATTR_BRIDGE_OFFSET,
};
```

**Çağıranın sorumluluğu (`sensor_hub`):** refresh yalnızca şu koşulların hepsi sağlandığında istenir:

- Son 0.5 s (52 örnek) boyunca $\lVert\omega_b\rVert < 0.05$ rad/s
- Son refresh'ten bu yana en az `CONFIG_APP_MAG_OFFSET_REFRESH_S` (varsayılan 60) s geçmiş, **veya** shell'den `mag refresh` istenmiş (istek de durağanlık koşuluna bağlıdır; kart hareketliyse reddedilir ve loglanır)

Sekans ≈ 11 ms sürdüğü için bir DRDY kaçırılır ve bir sonraki örneğin Δt'si ≈ 19 ms olur. Bu `MATH_SPEC.md §4`'teki geçerli aralıktadır; kart durağan olduğundan tahmin hatası ihmal edilir.

### 9.10 Hata kodları

| Kod | Durum |
|---|---|
| `0` | Başarılı |
| `-ENODEV` | Bus hazır değil, PRODUCT_ID ≠ `0x30`, INT GPIO hazır değil |
| `-EIO` | SPI aktarımı başarısız, `OTP_Rd_Done = 0` |
| `-EBUSY` | `sample_fetch`: ölçüm sürüyor veya boru hattı yeni başlatıldı (yeni veri yok) |
| `-ETIMEDOUT` | Bloklayıcı sekansta ölçüm 20 ms içinde bitmedi |
| `-ERANGE` | Eksen doymuş; veri güncellenmedi, sonraki ölçümden önce SET yapılacak |
| `-ENOTSUP` | Desteklenmeyen kanal veya attribute |
| `-EINVAL` | Okuma uzunluğu > 7 |

`sensor_hub` `-EBUSY` ve `-ERANGE` durumlarında o örnekte `IMU_SAMPLE_MAG_VALID` bayrağını koymaz; bunlar hata sayılmaz, sayaçla izlenir.

### 9.11 Eşzamanlılık

- `sample_fetch` ve `attr_set` normalde yalnızca sensör thread'inden çağrılır.
- `CONFIG_SENSOR_SHELL` açıksa shell de çağırabilir. Bu yüzden her API fonksiyonu `k_mutex` ile korunur.
- ISR bağlamından çağrı yasaktır (`k_usleep`, `k_msleep` ve SPI kullanır).

---

## 10. Datasheet tutarsızlıkları ve proje kararları

| # | Tutarsızlık | Karar |
|---|---|---|
| 1 | SPI bit numaralandırması zaman sırasına göre ("bit 0 = RW") | RW = komut baytının bit 7'si (§2.1). SparkFun ile doğrulandı |
| 2 | BW=00 ölçüm süresi: register tablosunda 8 ms, güç bölümünde 10 ms; maks. ODR 50 Hz | BW=00 kullanılmaz; BW=01 seçildi (§7) |
| 3 | Güç bölümü "BW=00, 16-bit mode" diyor | Veri formatı her zaman 18 bittir; 16 bit yalnızca XYZout2'nin okunmamasıdır (§6.2) |
| 4 | Status tablosunda isimler hizasız basılmış | OTP_Rd_Done = bit 4 (sütun hizası + SparkFun `OTP_READ_DONE (1 << 4)`) |
| 5 | CTRL3 tablosunda isimler hizasız | St_enp = bit 1, St_enm = bit 2, Spi_3w = bit 6 (SparkFun ile aynı) |
| 6 | SET/RESET sonrası bekleme süresi verilmemiş | 1000 µs (§8.2) |
| 7 | SPI burst adres artışı açıkça yazılmamış | Yalnızca `0x00–0x06` burst'ü kullanılır; STATUS ayrı okunur (§2.2) |
| 8 | SPI modu adıyla verilmemiş. SparkFun kütüphanesi yorumu: "Mode 3 according to datasheet. In practice SPI_MODE0 is what worked" | Mode 3 kullanılır (ISM330DHCX ile aynı bus polaritesi). Faz 1 doğrular (`TEST_PLAN.md` T-BUS-09) |
| 9 | Cm_freq tablosu BW=00 varsayıyor ama 100 Hz BW=00'ın maks. ODR'sini aşıyor | Sürekli mod kullanılmaz (§8.5) |
| 10 | Spec tablosundaki "Null Field Output … 0.5 G" satırı belirsiz | Ofset için hata eşiği konmaz, yalnızca uyarı verilir (§8.3) |

---

## 11. Kaynaklar

- MEMSIC, *MMC5983MA Rev A Datasheet* (resmi yayın 4/3/2019): SPI protokolü s.5–8, pin açıklaması ve eksen çizimi s.9–11, register haritası s.13–16, SET/RESET kullanımı s.18
- SparkFun, `SparkFun_MMC5983MA_Magnetometer_Arduino_Library` (`SparkFun_MMC5983MA_Arduino_Library_Constants.h`, `SparkFun_MMC5983MA_IO.cpp`, `SparkFun_MMC5983MA_Arduino_Library.cpp`)
- Zephyr `drivers/sensor/st/ism330dhcx/ism330dhcx_spi.c` (SPI okuma kalıbı), `include/zephyr/drivers/sensor.h`
