# HARDWARE_MAP.md — Pinout ve Donanım Konfigürasyonu

| Alan | Değer |
|---|---|
| Hedef kart | NUCLEO-G491RE (MB1367), Zephyr board adı `nucleo_g491re` |
| Sensör kartı | SparkFun 9DoF IMU Breakout — ISM330DHCX + MMC5983MA (SEN-19895, rev. v10) |
| Bağlantı | Tek SPI bus (SPI1), iki ayrı aktif-düşük CS |
| Bağlı belgeler | `PROJECT_SPEC.md`, `MATH_SPEC.md`, `MMC5983MA_SPEC.md`, `TEST_PLAN.md` |

Bu dosya pin, bus ve jumper kararlarının **tek kaynağıdır**. `boards/nucleo_g491re.overlay` bu dosyanın birebir karşılığıdır; biri değişirse diğeri aynı commit'te güncellenir. Burada yazmayan bir pini, AF numarasını veya bus'ı kod içinde "varsaymak" yasaktır.

---

## 1. Saat ağacı ve SPI hızı

Upstream `nucleo_g491re.dts` saat konfigürasyonu (değiştirilmeyecek):

| Parametre | Değer |
|---|---|
| HSE | 24 MHz |
| PLL | M=6, N=85, R=2 → **SYSCLK = 170 MHz** |
| AHB / APB1 / APB2 bölücü | 1 / 1 / 1 |
| SPI1 çekirdek saati | PCLK2 = 170 MHz |

STM32 SPI yalnızca 2'nin kuvveti bölücüleri destekler. Zephyr `spi_stm32` sürücüsü, `spi-max-frequency` değerini **aşmayan ilk** bölücüyü seçer:

| Bölücü | SCK | Not |
|---|---|---|
| /16 | 10.625 MHz | İki sensörün 10 MHz sınırını aşar — seçilmez |
| **/32** | **5.3125 MHz** | `spi-max-frequency = <10000000>` ile elde edilen gerçek hız |
| /64 | 2.656 MHz | Sinyal bütünlüğü sorunu olursa geri çekilme değeri (`5000000` yaz) |

Her iki sensör için `spi-max-frequency = <10000000>` yazılır; gerçek SCK **5.3125 MHz** olur. Kodda veya testte "10 MHz SPI" varsayımı yapılmaz. 10 cm'den uzun jumper kablolarda ilk adım `5000000` (→ 2.656 MHz) denemektir.

**Zamanlayıcı notu:** `zephyr,system-timer = &lptim1` olarak seçili görünse de LPTIM sistem zamanlayıcısı `CONFIG_PM=y` gerektirir. Bu projede `CONFIG_PM=n` tutulur; böylece sistem zamanlayıcısı SysTick olur ve `k_cycle_get_32()` 170 MHz'de sayar (çözünürlük 5.9 ns, taşma 25.3 s). `CONFIG_PM` açılırsa `k_cycle_get_32()` 32 768 Hz'e düşer ve Δt çözünürlüğü 30.5 µs olur. Bu yüzden `src/sensor_hub/` içinde şu derleme kontrolü zorunludur:

```c
BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC >= 1000000,
	     "dt timestamping needs a >= 1 MHz cycle counter; keep CONFIG_PM=n");
```

---

## 2. Ana bağlantı tablosu

Kablolama Nucleo'nun **Arduino konnektörleri** üzerinden yapılır. Arduino etiketleri (D13, D12, …) ve MCU pin adları kesin referanstır.

| Sinyal | SparkFun pin (serigrafi) | Breakout net | Nucleo Arduino | Morpho (bkz. §2.2) | MCU pini | Fonksiyon | Devicetree |
|---|---|---|---|---|---|---|---|
| SCK | `SCL` | `SCL/SCK` | D13 | CN10-11 | **PA5** | SPI1_SCK (AF5) | `&spi1` pinctrl `spi1_sck_pa5` |
| MOSI | `SDA` | `SDA/PICO` | D11 | CN10-15 | **PA7** | SPI1_MOSI (AF5) | `spi1_mosi_pa7` |
| MISO | `POCI` | `POCI/SA0` | D12 | CN10-13 | **PA6** | SPI1_MISO (AF5) | `spi1_miso_pa6` |
| CS_ISM | `ACS` | `!ISM_CS!` | D10 | CN10-17 | **PB6** | GPIO çıkış, aktif-düşük | `cs-gpios[0]` → `reg = <0>` |
| CS_MMC | `MCS` | `!MAG_CS!` | D9 | CN10-19 | **PC7** | GPIO çıkış, aktif-düşük | `cs-gpios[1]` → `reg = <1>` |
| ISM_INT1 (DRDY) | `INT1` | `ISM_INT1` | D2 | CN10-33 | **PA10** | GPIO giriş, EXTI10 | `ism330dhcx` → `drdy-gpios` |
| ISM_INT2 | `INT2` | `ISM_INT2` | D4 | CN10-29 | **PB5** | Bağlı, yapılandırılmaz (rezerv) | — |
| MAG_INT | `MINT` | `MAG_INT` | D7 | CN10-23 | **PA8** | GPIO giriş, EXTI8 (yalnızca tanı) | `mmc5983ma` → `int-gpios` |
| 3V3 | `3V3` | `3.3V` | Güç konnektörü `3V3` | — | — | Besleme | — |
| GND | `GND` | `GND` | Güç konnektörü `GND` | — | — | Toprak | — |

Kurallar:

- Sensör kartı **yalnızca 3V3** ile beslenir. MMC5983MA'nın VDD aralığı 2.8–3.6 V'tur; `5V` pini kartı bozar.
- `SCX` ve `SDX` pinleri **bağlanmaz** (ISM330DHCX Mode 2 sensör hub hatlarıdır).
- Breakout üzerindeki iki **Qwiic konnektörü boş kalır**. Bu konnektörler `SCL/SCK` ve `SDA/PICO` hatlarına doğrudan bağlıdır; SPI modunda oraya takılan her I²C cihazı bus'ı bozar.
- Kablo boyu ≤ 10 cm, ortak GND kablosu SCK kablosunun yanından geçmeli.

### 2.1 Breakout header düzeni

Eagle kart dosyasından (`SparkFun_9DoF_ISM330DHCX-MMC5983MA.brd`) çıkarılmıştır. Kart bileşen yüzü yukarıda, CAD üst görünüşünde:

```
            Header B (JP3, üst kenar, +y_B)
      INT1   INT2   MINT   MCS    SDX    SCX
   ┌──────────────────────────────────────────┐
   │                                          │
 J2│ Qwiic        [MMC5983MA  U2, rot 0°]     │J1 Qwiic
   │              [ISM330DHCX U1, rot 180°]   │
   │                                          │
   └──────────────────────────────────────────┘
      GND    3V3    SDA    SCL    ACS    POCI
            Header A (JP1, alt kenar, −y_B)
   −x_B ◄───────────────────────────────► +x_B
```

`x_B`, `y_B`, `z_B` (z_B bileşen yüzünden dışarı) **kart çerçevesi B**'dir. Sensör eksenleri ve gövde çerçevesi bu çerçeveye göre `MATH_SPEC.md §2`'de tanımlanır.

### 2.2 Morpho numaraları hakkında uyarı

Yukarıdaki CN10 numaraları standart Nucleo-64 (MB1367) yerleşimidir ve aynı PCB'yi kullanan upstream `nucleo_g474re/st_morpho_connector.dtsi` ile uyumludur. Upstream `nucleo_g491re/st_morpho_connector.dtsi` (2026, main) ise `PA5/PA6/PA7` ile `PB13/PB14/PB15` için CN10-11/13/15 ↔ CN10-30/28/26 atamalarını ters veriyor. İki dosya aynı PCB için çelişiyor.

Sonuç:

- Kablolama Arduino D-etiketleriyle yapılır.
- Devicetree'de `st_morpho_header` gpio-map'i **kullanılmaz**; pinler doğrudan `&gpioX N` ile referanslanır.
- Morpho üzerinden kablolama zorunlu olursa pin, UM2505 (NUCLEO-G4 kullanıcı kılavuzu) tablosundan ve süreklilik ölçümüyle doğrulanır.

---

## 3. SPI bus parametreleri

| Parametre | ISM330DHCX | MMC5983MA |
|---|---|---|
| SPI modu | Mode 3 (CPOL=1, CPHA=1). Datasheet Mode 0 ve 3'ü destekler; Zephyr sürücüsü Mode 3'ü sabit kodlar | Mode 3. Datasheet: "SCK is stopped high when CS is high", veri düşen kenarda sürülür, yükselen kenarda örneklenir |
| Bit sırası | MSB first | MSB first |
| Kelime boyu | 8 bit | 8 bit |
| Okuma bayrağı | Komut baytının bit 7'si = 1 | Komut baytının bit 7'si = 1 (datasheet buna "bit 0" diyor, çünkü telden **ilk** çıkan bit) |
| Adres alanı | bit 6:0 | bit 5:0; bit 6 "don't care" (0 gönderilir) |
| Burst artış | `CTRL3_C.IF_INC` (reset değeri 1) | 0x00→0x06 sıralı okuma (SparkFun referans sürücüsü 7 baytı tek CS çerçevesinde okur) |
| f_SCK maks. | 10 MHz | 10 MHz |
| tsu(CS) / th(CS) | 5 ns / 20 ns | 6 ns / 8 ns |
| Zephyr `operation` | `SPI_OP_MODE_MASTER \| SPI_MODE_CPOL \| SPI_MODE_CPHA \| SPI_WORD_SET(8) \| SPI_TRANSFER_MSB` | Aynısı |

İki cihaz da Mode 3 kullandığı için SCK boşta hep yüksektir ve CS geçişlerinde saat polaritesi değişmez. MMC5983MA için Mode 0'a geçmek yasaktır. Tek istisna, `TEST_PLAN.md` T-BUS-09'da Mode 3'ün başarısız ve Mode 0'ın başarılı olduğunun kanıtlanmasıdır; bu durumda sonuç bu dosyaya not edilir.

MMC5983MA protokol ayrıntıları: `MMC5983MA_SPEC.md §2`.

---

## 4. SPI modu, I²C kalıntısı ve bus bütünlüğü

İki çip de CS yüksekken I²C arayüzünü açık tutar. Breakout'ta CS hatları 100 kΩ ile 3V3'e çekilidir (R3 → `ACS`, R7 → `MCS`). Bu, açılışta MCU GPIO'ları yapılandırılmadan önce çiplerin sahte SPI trafiği görmemesini sağlar. Aynı nedenle, bir çip seçili değilken diğerinin SPI trafiğini (SCK=SCL, MOSI=SDA) I²C olarak dinler.

| Çip | I²C adresi | Hayalet I²C başlatabilecek ilk bayt | Önlem |
|---|---|---|---|
| ISM330DHCX | 0x6B (SA0 jumper kesilince adres belirsiz) | 0xD4–0xD7 | ST başlatma prosedürü: SPI kullanımında `CTRL4_C (0x13)` bit 2 `I2C_disable = 1` yazılır. Zephyr `ism330dhcx` sürücüsü bunu **yapmaz** (yalnızca `DEVICE_CONF` ve `BDU` ayarlar). |
| MMC5983MA | 0x30 | 0x60 / 0x61 | Çipte I²C kapatma biti yok. Tasarım kuralıyla çözülür (aşağıda). |

**Zorunlu init sırası (POST_KERNEL):**

| Öncelik | Birim | İş |
|---|---|---|
| 50 | `spi1` (`CONFIG_SPI_INIT_PRIORITY`) | Bus hazır |
| 90 | `ism330dhcx` (`CONFIG_SENSOR_INIT_PRIORITY`) | Upstream sürücü: reset, `DEVICE_CONF`, BDU, ODR/FS |
| 91 | `src/sensor_hub/ism330_spi_only.c` → `SYS_INIT(..., POST_KERNEL, CONFIG_APP_ISM330_SPI_ONLY_INIT_PRIORITY)` | `CTRL4_C` oku-değiştir-yaz: `\|= BIT(2)`; rezerv bit 7, 4 ve 0 her zaman 0 yazılır (ST `ism330dhcx_reg.h`: `not_used_01..03`). Bit 3 `DRDY_MASK`'tir, dokunulmaz ve reset değeri 0'dır |
| 92 | `mmc5983ma` (`CONFIG_MMC5983MA_INIT_PRIORITY`) | MMC trafiği ancak ISM'in I²C bloğu kapandıktan sonra başlar |

`SYS_INIT` önceliği bağlayıcı bölüm adına girdiği için tamsayı literal olmalıdır. `CONFIG_SENSOR_INIT_PRIORITY + 1` **yazılamaz**; uygulamanın `Kconfig.app` dosyasında `CONFIG_APP_ISM330_SPI_ONLY_INIT_PRIORITY` (varsayılan 91) tanımlanır (`MMC5983MA_SPEC.md §9.3`).

**İlk bayt kuralı:** Mode 3'te MOSI yalnızca SCK düşükken değişir. Bu yüzden I²C START koşulu (SCL yüksekken SDA düşüşü) yalnızca iki SPI işlemi arasında oluşabilir. START'tan sonra bir I²C alıcısının gördüğü ilk "adres" baytı, bir sonraki SPI işleminin **komut baytıdır**. Bu projede:

- MMC5983MA komut baytları: okuma `0x80 | 0x00..0x2F`, yazma `0x00..0x0C`.
- ISM330DHCX komut baytları: Zephyr sürücüsünün eriştiği register'lar.
- **Hiçbir SPI işleminin ilk baytı `0x60` veya `0x61` olamaz.** ISM330DHCX'in `0x60` ve `0x61` adreslerine yazma yapılmaz.

Faz 1'deki 100 000 döngülük bus soak testi (`TEST_PLAN.md` T-BUS-07) bu kuralın kanıtıdır.

---

## 5. Kesme hatları ve EXTI planı

STM32'de EXTI hattı numarası pin numarasıdır ve tüm portlar arasında paylaşılır: PA8 ile PB8 aynı anda kesme kaynağı olamaz. Projede kullanılan EXTI hatları:

| EXTI | Pin | Kaynak | Pull | Kenar | Durum |
|---|---|---|---|---|---|
| 5 | PB5 | ISM INT2 | — (analog/giriş) | — | Rezerv; kesme yapılandırılmaz |
| 8 | PA8 | MMC INT | `GPIO_PULL_DOWN` | Yükselen | Yalnızca `CONFIG_MMC5983MA_INT_DIAG=y` iken (lojik analizör noktası) |
| 10 | PA10 | ISM INT1 (DRDY) | `GPIO_PULL_DOWN` | Yükselen (`GPIO_INT_EDGE_TO_ACTIVE`) | **Aktif** — sensör thread'inin saat kaynağı |
| 13 | PC13 | Nucleo B1 USER butonu | Kart | — | Upstream DTS'de tanımlı; uygulama kullanmaz |

Elektriksel kurallar:

- **ISM INT1'e asla pull-up uygulanmaz.** ST datasheet DS13012 §6: *"INT1: Leave unconnected or connect with external pull-down during power-on. Pull-up must be avoided on this pin."* STM32 pinleri reset sonrası analog/yüksek empedanstır, bu güvenlidir. DT'de yalnızca `GPIO_PULL_DOWN` kullanılır.
- ISM INT2'ye de pull-up uygulanmaz (datasheet önerisi).
- ISM DRDY, Zephyr sürücüsünde **darbe modunda** (`ISM330DHCX_DRDY_PULSED`) ve aktif-yüksek çalışır.
- MMC INT aktif-yüksektir ve `INT_meas_done_en` set edilene kadar **hi-Z** kalır. Bu yüzden STM32 tarafında pull-down zorunludur, yoksa pin yüzer.

Yeni bir kesme eklenecekse önce bu tablo güncellenir; pin numarası 5, 8, 10 ve 13 ile çakışamaz.

---

## 6. Nucleo tarafındaki çakışmalar

| Kaynak | Çakışma | Karar |
|---|---|---|
| **PA5 = LD2 (yeşil LED)** | Upstream DTS `green_led` (`gpio-leds`) ve `pwm2` (`tim2_ch1_pa5`) PA5'i kullanır. PWM sürücüsü SPI'dan sonra init olursa PA5'in AF'sini TIM2_CH1'e çevirir ve SCK kaybolur. | Overlay'de `&pwm2 { status = "disabled"; };`. `CONFIG_LED=n`. Kodda `DT_ALIAS(led0)` ve `DT_ALIAS(pwm_led0)` kullanılmaz. SPI trafiği sırasında LD2'nin sönük yanıp sönmesi normaldir. |
| PB4 = `pwm3` (TIM3_CH1) | D5 pini PWM'e ayrılmış | Kullanılmaz |
| PB3 = SWO (D3) | GPIO yapılırsa SWO izleme kaybolur | Kullanılmaz |
| PB6 | Upstream DTS'de `spi1` tek CS olarak tanımlı | Overlay `cs-gpios` listesini **tamamen** yeniden yazar (iki eleman) |
| PA2/PA3 = LPUART1 | Zephyr konsolu ve shell (ST-LINK VCP, 115200) | Telemetri bu hattan çıkar |
| PC4/PC5 = USART1 (D1/D0) | Upstream'de etkin | Dokunulmaz, kullanılmaz |
| PB8/PB9 = I2C1 (D15/D14) | Upstream'de etkin | Dokunulmaz, kullanılmaz |

---

## 7. SparkFun kartı fiziksel jumper ayarları

Kartın arka yüzündeki lehim köprüleri (adlar Eagle şemasındaki parça adlarıdır):

| Jumper (arka serigrafi) | Yapı | Fabrika durumu | Bağladığı devre | SPI için | Gerekçe |
|---|---|---|---|---|---|
| `I2C_PU` ("I2C") | 3 pad, iki iz | İki taraf **kapalı** | Pad 2 = 3V3; pad 1 → R1 2.2 kΩ → `SCL/SCK`; pad 3 → R2 2.2 kΩ → `SDA/PICO` | **İki izi de kes** | SCK ve MOSI push-pull sürülür; 2.2 kΩ pull-up her hatta düşük seviyede ≈1.5 mA boşa akım çeker ve bu projenin kuralı push-pull SPI hatlarında pull-up bulundurmamaktır. |
| `6DOF_ADDR` ("0X6A" / "0X6B") | 3 pad, tek iz | Orta pad ↔ güç tarafı **kapalı** (0x6B) | Pad 2 = `POCI/SA0`; pad 3 → R5 100 kΩ → 3V3; pad 1 → R6 100 kΩ → GND | **Güç tarafı izini kes, toprak tarafını köprüleme** (iki taraf da açık) | SparkFun: "switch to SPI mode (both jumpers open)". `POCI` iki çipin ortak MISO hattıdır ve üzerinde bias direnci kalmamalı. |
| `SCX`, `SDX` | 2 pad, iz | Kapalı (`SCX`/`SDX` → GND) | ISM330DHCX SCx/SDx → GND | **Dokunma** | Mode 1'de SCx/SDx GND veya VDDIO'ya bağlı olmalı (DS13012 Tablo 18). |
| `LED` | 2 pad, iz | Kapalı | Güç LED'i + R4 1 kΩ | İsteğe bağlı | Yalnızca güç tüketimi. Fonksiyonel etkisi yok. |

Kartta kalan ve korunacak dirençler:

- **R3 100 kΩ** `ACS` → 3V3
- **R7 100 kΩ** `MCS` → 3V3
- C2 10 µF `CAP` (MMC SET/RESET kapasitörü)

### 7.1 Kesim sonrası ölçüm kontrolü

Güç kapalıyken multimetre direnç kademesinde ölçülür (çiplerin ESD diyotları diyot kademesinde okuma verebilir; direnç kademesini kullan):

| Ölçüm | Kesimden önce | Kesimden sonra (beklenen) |
|---|---|---|
| `SCL` ↔ `3V3` | ≈ 2.2 kΩ | 2.2 kΩ yolu **yok** (≫ 100 kΩ) |
| `SDA` ↔ `3V3` | ≈ 2.2 kΩ | ≫ 100 kΩ |
| `POCI` ↔ `3V3` | ≈ 100 kΩ | ≫ 100 kΩ |
| `POCI` ↔ `GND` | ≫ 100 kΩ | ≫ 100 kΩ (toprak tarafı köprülenmemeli) |
| `ACS` ↔ `3V3` | ≈ 100 kΩ | ≈ 100 kΩ (**değişmemeli**) |
| `MCS` ↔ `3V3` | ≈ 100 kΩ | ≈ 100 kΩ (**değişmemeli**) |

Sonuçlar `docs/evidence/phase0/jumper_check.md` dosyasına yazılır. Faz 1 bu kayıt olmadan başlamaz.

### 7.2 SPI moduna geçiş kuralı (özet)

1. Jumper'lar §7'ye göre ayarlanır ve §7.1 ölçümleri kaydedilir.
2. Hiçbir Qwiic kablosu takılı değildir.
3. CS hatları yalnızca MCU tarafından sürülür; 100 kΩ pull-up'lar açılışta iki çipi de seçilmemiş tutar.
4. Bir çipin CS'i düşük olduğu sürece o çip SPI modundadır. ISM330DHCX için I²C bloğu firmware tarafından kalıcı olarak kapatılır (§4).
5. MMC5983MA'nın I²C bloğu kapatılamaz; §4'teki ilk bayt kuralıyla zararsız hale getirilir.

---

## 8. Devicetree overlay — `boards/nucleo_g491re.overlay`

```dts
/*
 * SparkFun 9DoF (ISM330DHCX + MMC5983MA) on SPI1.
 * Pin decisions live in HARDWARE_MAP.md; keep both in sync.
 */
#include <zephyr/dt-bindings/sensor/ism330dhcx.h>

/* PA5 is SPI1_SCK here. The board DTS also claims it for TIM2_CH1 (pwm2). */
&pwm2 {
	status = "disabled";
};

&spi1 {
	pinctrl-0 = <&spi1_sck_pa5 &spi1_miso_pa6 &spi1_mosi_pa7>;
	pinctrl-names = "default";
	cs-gpios = <&gpiob 6 GPIO_ACTIVE_LOW>,	/* reg 0: D10 -> ACS */
		   <&gpioc 7 GPIO_ACTIVE_LOW>;	/* reg 1: D9  -> MCS */
	status = "okay";

	ism330dhcx: ism330dhcx@0 {
		compatible = "st,ism330dhcx";
		reg = <0>;
		spi-max-frequency = <10000000>;	/* /32 -> 5.3125 MHz */
		/* INT1 must never be pulled up (DS13012 sec. 6). */
		drdy-gpios = <&gpioa 10 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
		int-pin = <1>;
		accel-odr = <ISM330DHCX_DT_ODR_104Hz>;
		gyro-odr = <ISM330DHCX_DT_ODR_104Hz>;
		accel-range = <4>;
		gyro-range = <500>;
	};

	mmc5983ma: mmc5983ma@1 {
		compatible = "memsic,mmc5983ma";
		reg = <1>;
		spi-max-frequency = <10000000>;
		/* Hi-Z until INT_meas_done_en is set; only used by CONFIG_MMC5983MA_INT_DIAG. */
		int-gpios = <&gpioa 8 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
	};
};
```

Seçimlerin gerekçeleri:

| Özellik | Değer | Neden |
|---|---|---|
| `accel-odr`, `gyro-odr` | 104 Hz | Sensör thread'i DRDY ile tetiklenir: Δt ≈ 9.615 ms. PROJECT_SPEC'teki "100 Hz" hedefinin karşılığı. 208 Hz'e çıkmak yalnızca bu iki satır ve `MATH_SPEC.md` sabitlerinin değişmesini gerektirir. |
| `accel-range` | ±4 g (0.122 mg/LSB) | Elle döndürmede ±2 g doyabilir. ±8 g gereksiz çözünürlük kaybı. |
| `gyro-range` | ±500 dps (17.5 mdps/LSB) | Elle hızlı dönüş 250 dps'i aşar. Zephyr binding'i en fazla 2000 dps kabul eder (4000 dps yok). |
| `int-pin` | 1 | DRDY INT1'den alınır. INT2 rezerv. |

`memsic,mmc5983ma` binding'i upstream'de yoktur (upstream'de yalnızca `memsic,mmc56x3` var). Binding dosyası `dts/bindings/sensor/memsic,mmc5983ma.yaml` konumundadır, ayrıntı `MMC5983MA_SPEC.md §9`'da.

---

## 9. Donanıma bağlı Kconfig ayarları

`prj.conf` içinde bu dosyanın kararlarına bağlı satırlar (tam liste `PROJECT_SPEC.md`'dedir):

```ini
CONFIG_GPIO=y
CONFIG_SPI=y
CONFIG_SENSOR=y
CONFIG_ISM330DHCX_TRIGGER_OWN_THREAD=y
CONFIG_PM=n
CONFIG_LED=n
CONFIG_MMC5983MA_INIT_PRIORITY=92
CONFIG_APP_ISM330_SPI_ONLY_INIT_PRIORITY=91
```

| Ayar | Neden |
|---|---|
| `ISM330DHCX_TRIGGER_OWN_THREAD` | DRDY geri çağrısı sistem iş kuyruğuna (log, shell) bağımlı kalmaz. Sürücü thread'i kooperatif öncelikte çalışır. Geri çağrı zaman damgasını alır, **ivme ve jiroskopu okur** ve çerçeveyi kuyruğa koyar; okumadan dönen bir geri çağrı sürücü thread'ini sonsuz döngüye sokar (`PROJECT_SPEC.md §4.3`). |
| `PM=n` | §1: `k_cycle_get_32()` 170 MHz'de kalır. |
| `LED=n` | §6: PA5'in GPIO olarak yeniden yapılandırılmasını önler. |

---

## 10. SPI zaman bütçesi (104 Hz, tek tik)

| İşlem | Bayt (komut dahil) |
|---|---|
| ISM DRDY durum okuması (sürücü) | 2 |
| ISM accel okuması | 7 |
| ISM gyro okuması | 7 |
| MMC STATUS okuması | 2 |
| MMC veri burst (0x00–0x06) | 8 |
| MMC `CTRL0 = TM_M` yazması | 2 |
| **Toplam** | **≈ 28 bayt ≈ 42 µs** @ 5.3125 MHz |

ISM satırları yaklaşıktır; upstream sürücünün accel ve gyro'yu kaç işlemde okuduğu sürüme göre değişebilir. Bütçe ölçümle doğrulanır.

Zephyr işlem başına ek yükü (CS, bağlam kilidi, polling) ile birlikte tik başına bütçe **< 200 µs** hedeflenir. Ölçüm: `TEST_PLAN.md` T-RAW-07. Periyodik SET olan tiklerde (her 100 ölçümde bir) MMC sürücüsü 1 ms uyur (`MMC5983MA_SPEC.md §8.2`), bu bütçeye dahil değildir.

---

## 11. Kaynaklar

- Zephyr `boards/st/nucleo_g491re/nucleo_g491re.dts`, `arduino_r3_connector.dtsi`, `st_morpho_connector.dtsi` (main)
- Zephyr `drivers/sensor/st/ism330dhcx/`, `dts/bindings/sensor/st,ism330dhcx-*.yaml`, `drivers/spi/spi_stm32.c`
- SparkFun, *Qwiic 9DoF – ISM330DHCX, MMC5983MA Hookup Guide*; şema `SparkFun_9DoF_ISM330DHCX-MMC5983MA.pdf`; Eagle dosyaları (GitHub `sparkfun/SparkFun_Qwiic_9DoF-ISM330DHCX-MMC5983MA`)
- ST DS13012 Rev 7 — ISM330DHCX datasheet (§3, §4.4.1, §6, §9.14–9.15)
- MEMSIC MMC5983MA Rev A datasheet (4/3/2019)
