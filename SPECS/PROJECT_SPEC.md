# PROJECT_SPEC.md — 9-DoF EKF, Zephyr RTOS, NUCLEO-G491RE

| Alan | Değer |
|---|---|
| Hedef MCU | STM32G491RE — ARM Cortex-M4F, tek duyarlıklı FPU (FPv4-SP-D16), 170 MHz, 512 KB flash, 112 KB RAM |
| Kart | NUCLEO-G491RE (MB1367), Zephyr board adı `nucleo_g491re` |
| Sensör kartı | SparkFun 9DoF Breakout: ISM330DHCX (ivme + jiroskop) ve MMC5983MA (manyetometre) |
| Sensör bus'ı | SPI1, Mode 3, iki bağımsız aktif-düşük CS |
| Çıktı | Kuaterniyon, roll/pitch/yaw ve jiroskop bias kestirimi; LPUART1 (ST-LINK VCP, 115200) üzerinden telemetri ve shell |
| Zephyr tabanı | `main`, `VERSION` 4.4.99, commit `8076e9d493a` (2026-06-13); Zephyr SDK 1.0.1 |
| Bağlı belgeler | `DOCS/HARDWARE_MAP.md`, `DOCS/MATH_SPEC.md`, `DOCS/MMC5983MA_SPEC.md`, `DOCS/TEST_PLAN.md` |

Bu belge mimariyi, kod standardını, dizin düzenini, thread modelini ve çalışma kurallarını belirler. Projede kod yazan herkes (insan ya da ajan) için bağlayıcıdır.

---

## 0. Belge hiyerarşisi

| Belge | Tek kaynak olduğu konu |
|---|---|
| `PROJECT_SPEC.md` | Mimari, kod standardı, dizin düzeni, thread modeli, Kconfig, derleme ve çalışma kuralları |
| `DOCS/HARDWARE_MAP.md` | Pinler, jumper'lar, SPI parametreleri, init sırası, devicetree overlay |
| `DOCS/MATH_SPEC.md` | Koordinat çerçeveleri, kalibrasyon modelleri, EKF denklemleri, sabitler, referans vektörler |
| `DOCS/MMC5983MA_SPEC.md` | MMC5983MA register haritası, ölçüm sırası, SET/RESET, binding |
| `DOCS/TEST_PLAN.md` | Test fazları, kabul eşikleri, telemetri satır biçimi |

Kurallar:

- Bu belgedeki §1 ve §5 birer **özettir**. Bir sayı, pin veya denklem ayrıntı belgesiyle çelişirse ayrıntı belgesi geçerlidir ve çelişki aynı commit'te giderilir.
- Thread modeli, kod standardı ve dizin düzeni konusunda bu belge geçerlidir.
- Karar değişikliği önce ilgili belgeye yazılır, sonra koda girer. Belgede karşılığı olmayan bir pin, register, sabit veya dosya yolu kodda varsayılmaz.
- Yollar: `src/…`, `boards/…`, `drivers/…`, `dts/…`, `tests/…`, `prj.conf` uygulama köküne (`app/`) görelidir. `SPECS/`, `tools/` ve `docs/` depo kökündedir.

---

## 1. Donanım ve veri yolu mimarisi

### 1.1 Bus ve pinler

Tek SPI controller (**SPI1**) iki sensörü taşır. SCK, MISO ve MOSI ortaktır; her çipin kendi CS hattı vardır. SPI2 kullanılmaz: SPI1 pinleri Arduino konnektöründe yan yanadır (D10–D13) ve upstream board DTS'inde zaten etkindir.

| Sinyal | MCU pini | Arduino | Devicetree |
|---|---|---|---|
| SCK | PA5 | D13 | `spi1_sck_pa5` |
| MISO | PA6 | D12 | `spi1_miso_pa6` |
| MOSI | PA7 | D11 | `spi1_mosi_pa7` |
| `cs_ism` (ISM330DHCX) | PB6 | D10 | `cs-gpios[0]`, `reg = <0>` |
| `cs_mmc` (MMC5983MA) | PC7 | D9 | `cs-gpios[1]`, `reg = <1>` |
| ISM INT1 (DRDY) | PA10 | D2 | `drdy-gpios`, EXTI10, yükselen kenar, pull-down |
| MMC INT | PA8 | D7 | `int-gpios`, yalnızca tanı (`CONFIG_MMC5983MA_INT_DIAG`) |

Tam tablo, jumper ayarları ve çakışma analizi: `HARDWARE_MAP.md §2–§7`.

### 1.2 SPI parametreleri

| Parametre | ISM330DHCX | MMC5983MA |
|---|---|---|
| Mod | Mode 3 (CPOL=1, CPHA=1) | Mode 3 |
| Çerçeve | 8 bit, MSB önce | 8 bit, MSB önce |
| Okuma bayrağı | Komut baytı bit 7 | Komut baytı bit 7 |
| Datasheet f_SCK üst sınırı | 10 MHz | 10 MHz |
| `spi-max-frequency` | 10 000 000 | 10 000 000 |
| Gerçek SCK | 5.3125 MHz (170 MHz / 32) | 5.3125 MHz |

STM32 SPI bölücüsü yalnızca 2'nin kuvvetidir; /16 seçeneği 10.625 MHz verip sınırı aştığı için sürücü /32'ye iner. "10 MHz SPI" varsayımı kodda ve testte yapılmaz.

### 1.3 `struct spi_dt_spec` kullanımı

SPI'a dokunan üç birim vardır ve üçü de bus'a yalnızca `struct spi_dt_spec` üzerinden erişir:

| Birim | Düğüm | Not |
|---|---|---|
| Upstream `ism330dhcx` sürücüsü | `ism330dhcx@0` | Değiştirilmez |
| `drivers/sensor/mmc5983ma/` | `mmc5983ma@1` | Bu projenin sürücüsü |
| `src/sensor_hub/ism330_spi_only.c` | `ism330dhcx@0` | Tek seferlik `CTRL4_C.I2C_disable` yazımı (`HARDWARE_MAP.md §4`) |

```c
#define SENSOR_SPI_OPERATION \
	(SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

/* sürücü içinde */
.spi = SPI_DT_SPEC_INST_GET(inst, SENSOR_SPI_OPERATION),

/* uygulama içinde */
static const struct spi_dt_spec ism330_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ism330dhcx), SENSOR_SPI_OPERATION);
```

Kurallar:

- Controller modu `SPI_OP_MODE_MASTER` ile yazılır. Sabitlenen ağaçta `SPI_OP_MODE_CONTROLLER` tanımlı değildir.
- `SPI_DT_SPEC_GET` iki argümanla çağrılır; üçüncü (gecikme) argümanı verilmez.
- Her işlemden önce değil, init'te bir kez `spi_is_ready_dt()` kontrol edilir.
- Bir register erişimi tek `spi_transceive_dt()` / `spi_write_dt()` çağrısıdır: CS o çağrı boyunca düşük kalır, çağrı bitince yükselir. `SPI_HOLD_ON_CS` ve `SPI_LOCK_ON` kullanılmaz; iki çipin işlemleri SPI bağlam kilidiyle sıralanır.
- CS pinleri elle sürülmez; `cs-gpios` ve `reg` eşleşmesi yeterlidir.
- Hiçbir SPI işleminin ilk baytı `0x60` veya `0x61` olamaz (MMC5983MA'nın kapatılamayan I²C bloğu, `HARDWARE_MAP.md §4`).

### 1.4 Overlay

`boards/nucleo_g491re.overlay` dosyasının bağlayıcı sürümü `HARDWARE_MAP.md §8`'dedir. İskeleti:

```dts
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
		spi-max-frequency = <10000000>;
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
		int-gpios = <&gpioa 8 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
	};
};
```

Örnekleme hızı nominal 104 Hz'dir (Δt ≈ 9.615 ms). Bu, "100 Hz" hedefinin ISM330DHCX ODR tablosundaki karşılığıdır. Gerçek hız çipin iç osilatörüne bağlıdır ve karttan karta yüzde birkaç değişir (`MATH_SPEC.md §4`); kod sabit bir hız varsaymaz. 208 Hz'e geçiş iki ODR satırını ve `MATH_SPEC.md §14`'teki örnek sayısına bağlı sabitleri değiştirmeyi gerektirir.

---

## 2. Kod standartları

Ölçüt: kodu ilk kez açan deneyimli bir gömülü C mühendisi, dosyayı yukarıdan aşağı bir kez okuyup ne yaptığını ve neden öyle yaptığını anlamalıdır.

### 2.1 İsimlendirme

- Fonksiyon adı bir fiil öbeğidir ve ne yaptığını söyler: `accel_raw_to_mps2()`, `ekf_predict()`, `mag_apply_iron_correction()`.
- Dışa açık semboller modül önekini taşır: `ekf_`, `quat_`, `sensor_hub_`, `fusion_`, `telemetry_`, `mmc5983ma_`. Dosyaya özel her şey `static`'tir ve öneksiz olabilir.
- Fiziksel büyüklük taşıyan değişken birimini adında taşır: `dt_s`, `timestamp_cyc`, `gyro_rads`, `accel_mps2`, `field_gauss`, `yaw_mdeg`.
- Tek harfli isim yalnızca iki yerde kabul edilir: kısa döngü sayaçları (`i`, `row`, `col` tercih edilir) ve `MATH_SPEC.md` sembolleriyle birebir eşleşen matris adları (`P`, `F`, `Q`, `H`, `K`, `S`). İkinci durumda boyut ve anlam bildirimin yanında yazılır.
- Sihirli sayı olmaz. Register adresleri ve bit alanları sürücü başlığında, filtre sabitleri `ekf_config.h`'de adlandırılır.
- Kod, tanımlayıcılar, yorumlar ve commit mesajları İngilizcedir. Şartname belgeleri Türkçedir.

### 2.2 Yorumlar

Yorum yalnızca kodun kendisinin söyleyemediği şeyi söyler:

| Yazılır | Yazılmaz |
|---|---|
| Neden bu yol seçildi, hangi alternatif neden elendi | Satırın ne yaptığının tekrarı (`/* increment counter */`) |
| Donanım ve sürücü gariplikleri, datasheet/errata atfıyla | Fonksiyon adını yeniden söyleyen başlık blokları |
| Matematik adımının `MATH_SPEC.md` karşılığı (`/* MATH_SPEC 7.4: F uses q before the update */`) | Bölüm ayırıcı süs satırları, ASCII çerçeveler |
| Birim, çerçeve, sahiplik ve çağrı bağlamı kısıtları | Değişiklik günlüğü, yazar/tarih notu, "bu fonksiyon şunu yapar" özetleri |

Başlık dosyalarındaki dışa açık API'ler kısa bir Doxygen bloğu alır: ne yapar, birimler, dönüş kodları. `static` yardımcılar için ad yeterli değilse ad düzeltilir, yorum eklenmez.

### 2.3 Fonksiyon ve modül yapısı

- Bir fonksiyon tek iş yapar ve bir ekrana sığar (kılavuz: ≤ 50 satır, iç içe en fazla 3 seviye). Daha uzunu, adı konabilen adımlara bölünür. İstisna yalnızca düz tablo doldurma kodudur (ör. Jacobian elemanları).
- Modüller arası veri yalnızca başlıkta bildirilen fonksiyonlar ve kuyruklar üzerinden akar. `extern` değişken yoktur.
- Değişmeyen her işaretçi parametresi ve tablo `const`'tur.
- Biçim Zephyr'in `.clang-format` dosyasıdır: tab ile girinti (8), 100 sütun, K&R ayraçları. Tek satırlık gövdeler de ayraç alır.
- Başlık sırası: C standardı → Zephyr → CMSIS → proje. Her başlık kendi başına derlenebilir.

### 2.4 Bellek

Heap yoktur.

| Yasak | Yerine |
|---|---|
| `malloc`, `calloc`, `realloc`, `free`, `k_malloc`, `k_calloc`, `k_free`, `k_heap_alloc`, `K_HEAP_DEFINE` | Dosya kapsamında `static` nesneler |
| Çalışma zamanında thread/yığın oluşturma, `CONFIG_DYNAMIC_THREAD` | `K_THREAD_DEFINE` |
| `k_msgq_alloc_init`, `k_queue_alloc_append`, `k_fifo_alloc_put` | `K_MSGQ_DEFINE` |
| VLA, `alloca` | Derleme zamanı sabit boyutlu diziler |
| Yığında matris (`float32_t tmp[49]` yerel değişken) | `struct ekf_workspace` içindeki statik tamponlar |

`prj.conf` bunu zorlar: `CONFIG_HEAP_MEM_POOL_SIZE=0`, `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=0` ve `CONFIG_SHELL_HISTORY=n` (shell geçmişi kendi `k_heap`'ini kullanır ve imaja `k_heap_alloc`'u çeker). Bağlanmış imajda (`zephyr.elf` sembol tablosu) `malloc`, `k_malloc` veya `k_heap_alloc` sembolü görünmesi derleme hatası sayılır. Zephyr'in `LOG_*` makroları içeride VLA kullandığı için `-Wvla` bayrağı açılamaz; VLA yasağı kod incelemesiyle denetlenir.

### 2.5 Tipler ve kayan nokta

- Genişliği önemli olan her tamsayı `<stdint.h>` tipidir. `int` yalnızca errno dönüşleri ve küçük döngü sayaçları içindir.
- Kayan nokta yalnızca tek duyarlıklıdır. CMSIS-DSP'ye verilen tamponlar `float32_t`, geri kalan her şey `float`'tır (aynı tip). `double` hiçbir yerde geçmez.
- Sabitler `f` son eki alır (`0.5f`, `9.80665f`). Matematik fonksiyonları `f` son eklidir: `sqrtf`, `sinf`, `cosf`, `atan2f`, `asinf`, `fabsf`.
- `float` değişken argümanlı fonksiyona verilmez (`printk`, `LOG_*`, `shell_print`): argüman `double`'a terfi eder ve M4F'de yazılım kayan noktasına düşer. Çıktılar ölçeklenmiş `int32_t` olarak basılır (`MATH_SPEC.md §12`).
- `float` eşitlik karşılaştırması yapılmaz. Sıfıra bölme ve tanım kümesi dışı girişler (`asinf`, `sqrtf`) çağrıdan önce kontrol edilir.
- `-ffast-math` ve türevleri yasaktır: `isfinite()` kontrollerini derleyicinin silmesine izin verir.
- FPU kullanan thread'ler `K_FP_REGS` seçeneğiyle tanımlanır.

Uygulama hedefinin derleme bayrakları (`app/CMakeLists.txt`):

```cmake
target_compile_options(app PRIVATE
	-Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion -Werror)
```

Bu küme sabitlenen Zephyr ağacının başlıklarıyla uyarısız derlenir. Uyarı bastırmak için `#pragma` veya bayrak gevşetme yapılmaz; kod düzeltilir.

### 2.6 Hata yönetimi

Fonksiyonlar `0` veya negatif errno döndürür. Ön koşullar guard clause ile en başta elenir; ana akış girintisiz kalır.

```c
static int sample_interval_s(const struct sensor_hub *hub, uint32_t timestamp_cyc, float *dt_s)
{
	uint32_t elapsed_cyc;
	float interval_s;

	if (!hub->has_reference) {
		return -EAGAIN;
	}

	/* Unsigned subtraction stays correct across the 25 s counter wrap. */
	elapsed_cyc = timestamp_cyc - hub->reference_cyc;
	interval_s = (float)elapsed_cyc / (float)sys_clock_hw_cycles_per_sec();

	if (interval_s <= EKF_DT_MIN || interval_s > EKF_DT_MAX) {
		return -EINVAL;
	}

	*dt_s = interval_s;
	return 0;
}
```

| Kod | Anlamı |
|---|---|
| `0` | Başarılı |
| `-EIO` | Bus hatası, beklenmeyen çip kimliği |
| `-ENODEV` | Cihaz veya bus hazır değil |
| `-EINVAL` | Geçersiz argüman, sonlu olmayan girdi, aralık dışı Δt |
| `-EBUSY` | Ölçüm henüz hazır değil (MMC5983MA `Meas_M_Done` = 0) |
| `-ETIMEDOUT` | Beklenen DRDY veya ölçüm süresinde gelmedi |
| `-EAGAIN` | Kapılama nedeniyle atlandı; hata değildir, sayaç artar |
| `-EDOM` | Sayısal hata (tekil matris, sıfır norm, sonlu olmayan durum) |
| `-ENOTSUP` | Desteklenmeyen kanal veya öznitelik |

Kurallar:

- Dönüş değeri yok sayılmaz. Bilerek yok sayılan nadir durumda `(void)` ve tek satırlık gerekçe yazılır.
- Hata kodu yukarı taşınırken değiştirilmez; yalnızca alt katmanın kodu çağırana anlamsızsa çevrilir.
- `goto` yalnızca tek bir temizlik etiketine ileri atlama için kullanılır.
- `__ASSERT` programcı hatalarını yakalar (ihlal edilemeyecek iç değişmezler). Çalışma zamanında gerçekten olabilecek durumlar errno ile döner.
- Örnek başına tekrarlayan olaylar log yerine **sayaç** artırır (§4.6). `LOG_ERR` durum geçişlerinde bir kez basılır.

### 2.7 Zephyr deyimleri

- Cihazlar derleme zamanında çözülür: `DEVICE_DT_GET(DT_NODELABEL(...))`, `SPI_DT_SPEC_GET`, `GPIO_DT_SPEC_GET`. `device_get_binding()` ve `DT_LABEL` kullanılmaz.
- Sürücü API tablosu `static DEVICE_API(sensor, mmc5983ma_api) = { ... };` ile, örnekler `SENSOR_DEVICE_DT_INST_DEFINE` ve `DT_INST_FOREACH_STATUS_OKAY` ile tanımlanır. Tablonun tipi `struct sensor_driver_api`'dir.
- Her `.c` dosyası kendi log modülünü kaydeder: `LOG_MODULE_REGISTER(sensor_hub, CONFIG_APP_LOG_LEVEL);`.
- Derleme zamanında doğrulanabilen her varsayım `BUILD_ASSERT` ile yazılır (§3.4).

---

## 3. Dizin düzeni ve Zephyr mimarisi

### 3.1 Ağaç

```
imu-kalman-filter/
├── SPECS/
│   ├── PROJECT_SPEC.md
│   └── DOCS/                         HARDWARE_MAP, MATH_SPEC, MMC5983MA_SPEC, TEST_PLAN
├── app/                              Zephyr uygulaması (west build hedefi)
│   ├── CMakeLists.txt
│   ├── Kconfig                       Kök menü: Kconfig.app + Kconfig.zephyr
│   ├── Kconfig.app                   CONFIG_APP_* sembolleri (tests/hw ile ortak)
│   ├── prj.conf
│   ├── debug.conf                    Geliştirme eki (assert, thread analyzer)
│   ├── zephyr/module.yml             drivers/ dizinini Zephyr modülü olarak kaydeder
│   ├── boards/
│   │   └── nucleo_g491re.overlay
│   ├── dts/bindings/sensor/
│   │   └── memsic,mmc5983ma.yaml
│   ├── include/app/drivers/sensor/
│   │   └── mmc5983ma.h               Sürücünün uygulamaya açık özel attribute'ları
│   ├── drivers/
│   │   ├── CMakeLists.txt
│   │   ├── Kconfig
│   │   └── sensor/mmc5983ma/
│   │       ├── CMakeLists.txt
│   │       ├── Kconfig
│   │       ├── mmc5983ma.c           init, sample_fetch, channel_get, attr_set/attr_get
│   │       ├── mmc5983ma.h           config/data yapıları
│   │       ├── mmc5983ma_reg.h       Register ve bit tanımları
│   │       └── mmc5983ma_decode.h    Saf çıkış biçimi dönüşümleri
│   ├── src/
│   │   ├── main.c                    Hazırlık kontrolleri, thread tanımları ve başlatma
│   │   ├── sensor_hub/
│   │   │   ├── sensor_hub.[ch]       DRDY işleyicisi, sensör thread'i, kuyruk üretimi
│   │   │   ├── sensor_cal.[ch]       Kalibrasyon, eksen haritası, Δt (saf fonksiyonlar)
│   │   │   ├── imu_sample.h          sensor_hub → füzyon arayüz sözleşmesi
│   │   │   ├── axis_map.h            İşaretli permütasyon tabloları
│   │   │   ├── accel_cal.h           6 yüz kalibrasyon katsayıları
│   │   │   ├── mag_cal.h             tools/mag_calib.py çıktısı
│   │   │   └── ism330_spi_only.c     CTRL4_C.I2C_disable init kancası
│   │   ├── ekf/
│   │   │   ├── ekf_core.[ch]         Durum, kovaryans, tahmin ve düzeltme adımları
│   │   │   ├── ekf_math.[ch]         Kuaterniyon, Euler, Ω/Ξ/R kurucuları, sayısal kontroller
│   │   │   └── ekf_config.h          MATH_SPEC §14 sabitleri
│   │   ├── fusion/
│   │   │   └── fusion.[ch]           Füzyon thread'i, hizalama durum makinesi, anlık görüntü
│   │   └── telemetry/
│   │       └── telemetry.[ch]        Telemetri thread'i ve `telemetry`, `mag`, `ekf` shell komutları
│   └── tests/
│       ├── ekf/                      Ztest: ekf_math ve ekf_core
│       ├── sensor_hub/               Ztest: eksen haritası, kalibrasyon, Δt, MMC çözümleme
│       └── hw/
│           ├── bus_id/               Faz 1: ham SPI ve init sırası (yalnızca hedefte)
│           └── raw_data/             Faz 2: sensör hattı istatistikleri ve zamanlama (yalnızca hedefte)
├── tools/
│   ├── telemetry_records.py          Telemetri kayıt düzenleri (araçların ortak modülü)
│   ├── capture.py                    Seri porttan CSV kaydı
│   ├── analyze.py                    Kayıtları TEST_PLAN eşikleriyle değerlendirir
│   ├── mag_calib.py                  Elipsoid uydurma, mag_cal.h üretimi
│   └── ekf_reference.py              MATH_SPEC'in float64 referans uygulaması
└── docs/evidence/                    Faz kanıtları (ölçüm kayıtları, lojik analizör çıktıları)
```

Binding dosyası `drivers/` altında değil `dts/bindings/` altındadır: Zephyr binding'leri yalnızca bir DTS kökünün `dts/bindings/` dizininde arar.

### 3.2 Bağımlılık yönü

```
main ──► sensor_hub ──► (Zephyr sensor API) ──► ism330dhcx, mmc5983ma
  │          │
  │          └─ imu_sample.h ─┐
  ├────► fusion ◄─────────────┘
  │          └──► ekf_core ──► ekf_math
  └────► telemetry ──► fusion, sensor_hub (yalnızca anlık görüntü ve istek API'leri)
```

- `ekf/` Zephyr çekirdeğine, cihaz modeline ve log altyapısına bağlı **değildir**. Yalnızca `<errno.h>`, `<math.h>`, `<stdint.h>`, `<stdbool.h>` ve `arm_math.h` içerir. Böylece aynı kaynak `native_sim` üzerinde test edilir.
- `sensor_cal.c` de aynı kurala uyar; cihaz G/Ç'si `sensor_hub.c`'de kalır.
- `ekf/` düz diziler alır (`const float gyro_b[3]`), `struct imu_sample`'ı bilmez.
- Sürücü uygulamayı bilmez; kalibrasyon ve eksen dönüşümü sürücüde yapılmaz.
- Okların tersine `#include` yapılmaz.

### 3.3 Derleme sistemi

Uygulama bağımsız (freestanding) bir Zephyr uygulamasıdır. `drivers/` dizini aynı zamanda bir Zephyr modülüdür; böylece sürücü ağaç içi bir sürücü gibi kendi kütüphanesi ve Kconfig menüsüyle bağlanır.

`app/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.20.0)

# The MMC5983MA driver is a Zephyr module so that it links like an in-tree driver.
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(imu_ekf LANGUAGES C)

target_sources(app PRIVATE
	src/main.c
	src/sensor_hub/sensor_hub.c
	src/sensor_hub/sensor_cal.c
	src/sensor_hub/ism330_spi_only.c
	src/ekf/ekf_core.c
	src/ekf/ekf_math.c
	src/fusion/fusion.c
	src/telemetry/telemetry.c
)
target_include_directories(app PRIVATE src)
target_compile_options(app PRIVATE
	-Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion -Werror)
```

`app/zephyr/module.yml`:

```yaml
name: imu-ekf
build:
  cmake: drivers
  kconfig: drivers/Kconfig
  settings:
    dts_root: .
```

Sürücü kaynakları `app` hedefine `target_sources` ile eklenmez ve `find_package(Zephyr)` çağrısından sonra `zephyr_library()` ile de oluşturulmaz; ikinci durumda kütüphane bağlanmaz ve cihaz nesnesi sessizce kaybolur.

Modül üç şeyi birlikte getirir: sürücü kütüphanesi (`drivers/CMakeLists.txt`), sürücü Kconfig menüsü (`drivers/Kconfig`) ve binding kökü (`dts_root`). Bu yüzden uygulamanın kök `Kconfig`'i sürücü Kconfig'ini ayrıca `rsource` **etmez** (sembol iki kez tanımlanırdı). `tests/hw/*` uygulamaları aynı modülü `list(APPEND ZEPHYR_EXTRA_MODULES <app dizini>)` ile bağlar ve overlay'i `DTC_OVERLAY_FILE` ile alır (`TEST_PLAN.md §0.3`).

### 3.4 Kconfig

Uygulama sembolleri (`app/Kconfig.app`; kök `app/Kconfig` bunu `rsource` eder, ardından `source "Kconfig.zephyr"`):

| Sembol | Tip | Varsayılan | Anlamı |
|---|---|---|---|
| `CONFIG_APP_LOG_LEVEL` | log seviyesi | INF | Uygulama modüllerinin log seviyesi |
| `CONFIG_APP_ISM330_SPI_ONLY_INIT_PRIORITY` | int | 91 | `CTRL4_C.I2C_disable` kancasının `POST_KERNEL` önceliği |
| `CONFIG_APP_EKF_USE_MAG` | bool | y | Manyetometre heading düzeltmesini etkinleştirir |
| `CONFIG_APP_MAG_DECLINATION_MDEG` | int | 0 | Sapma $D$, miliderece, doğu pozitif. 0 → yaw manyetik kuzeye göre |
| `CONFIG_APP_MAG_INCLINATION_MDEG` | int | 56000 | Eğim $I$, miliderece, aşağı pozitif. WMM ile doğrulanır |
| `CONFIG_APP_TELEMETRY_RATE_HZ` | int, 1–50 | 20 | Açılıştaki telemetri akış hızı (`telemetry rate` ile değişir) |
| `CONFIG_APP_MAG_OFFSET_REFRESH_S` | int, 1–86400 | 60 | MMC köprü ofseti tazeleme aralığı, saniye (`MMC5983MA_SPEC.md §9.9`) |

Sürücü sembolleri (`drivers/sensor/mmc5983ma/Kconfig`, tam liste `MMC5983MA_SPEC.md §9.3`): `CONFIG_MMC5983MA` (`DT_HAS_MEMSIC_MMC5983MA_ENABLED`'a bağlı, varsayılan y, `SPI` seçer), `CONFIG_MMC5983MA_INIT_PRIORITY` (92), `CONFIG_MMC5983MA_BANDWIDTH` (1), `CONFIG_MMC5983MA_PERIODIC_SET_INTERVAL` (100), `CONFIG_MMC5983MA_SR_SETTLE_US` (1000), `CONFIG_MMC5983MA_MEAS_TIMEOUT_MS` (20), `CONFIG_MMC5983MA_INT_DIAG` (n).

`prj.conf` (tam liste):

```ini
# Kernel
CONFIG_PM=n
CONFIG_TIMESLICING=n
CONFIG_HW_STACK_PROTECTION=y
CONFIG_THREAD_NAME=y
CONFIG_THREAD_STACK_INFO=y
CONFIG_INIT_STACKS=y

# No heap
CONFIG_HEAP_MEM_POOL_SIZE=0
CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=0

# Single-precision FPU and CMSIS-DSP
CONFIG_FPU=y
CONFIG_FPU_SHARING=y
CONFIG_CMSIS_DSP=y
CONFIG_CMSIS_DSP_MATRIX=y
CONFIG_CMSIS_DSP_MATRIXCHECK=y

# Bus and sensors
CONFIG_GPIO=y
CONFIG_SPI=y
CONFIG_SENSOR=y
CONFIG_ISM330DHCX_TRIGGER_OWN_THREAD=y
CONFIG_APP_ISM330_SPI_ONLY_INIT_PRIORITY=91
CONFIG_MMC5983MA_INIT_PRIORITY=92
CONFIG_LED=n

# Logging and shell on LPUART1
CONFIG_LOG=y
CONFIG_LOG_MODE_DEFERRED=y
CONFIG_LOG_BUFFER_SIZE=2048
CONFIG_SHELL=y
CONFIG_SHELL_STACK_SIZE=2048
CONFIG_SHELL_HISTORY=n
CONFIG_SHELL_THREAD_PRIORITY_OVERRIDE=y
CONFIG_SHELL_THREAD_PRIORITY=5
```

| Ayar | Neden |
|---|---|
| `PM=n` | Board DTS sistem zamanlayıcısı olarak LPTIM1'i seçer, ama o sürücü `PM` ister. `PM=n` iken SysTick kullanılır ve `k_cycle_get_32()` 170 MHz'de sayar |
| `TIMESLICING=n` | Aynı öncelikteki thread'ler (telemetri, shell) birbirini zaman dilimiyle kesmez; zamanlama yalnızca önceliğe bağlıdır |
| `HW_STACK_PROTECTION` | MPU yığın koruması; taşma sessiz bellek bozulması yerine hata üretir |
| `CMSIS_DSP_MATRIXCHECK` | Bu olmadan CMSIS boyut uyuşmazlığını kontrol etmez ve `ARM_MATH_SIZE_MISMATCH` hiç dönmez |
| `LOG_MODE_DEFERRED` | `LOG_*` çağrısı yalnızca kuyruğa yazar; biçimleme en düşük öncelikli log thread'inde yapılır |
| `LED=n` | PA5 SPI1_SCK'dir; LED sürücüsü onu GPIO'ya çevirmemelidir |
| `SHELL_HISTORY=n` | Shell geçmişi kendi `k_heap`'inden yer ayırır; §2.4'teki heap yasağı gereği kapalıdır |

`debug.conf` (`-DEXTRA_CONF_FILE=debug.conf` ile eklenir): `CONFIG_ASSERT=y`, `CONFIG_THREAD_ANALYZER=y`, `CONFIG_SCHED_THREAD_USAGE=y`, `CONFIG_KERNEL_SHELL=y`.

Zorunlu derleme zamanı kontrolleri:

```c
BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC >= 1000000,
	     "dt timestamping needs a >= 1 MHz cycle counter; keep CONFIG_PM=n");
BUILD_ASSERT(CONFIG_APP_ISM330_SPI_ONLY_INIT_PRIORITY > CONFIG_SENSOR_INIT_PRIORITY,
	     "I2C_disable must be written after the ISM330DHCX driver resets the chip");
BUILD_ASSERT(CONFIG_MMC5983MA_INIT_PRIORITY > CONFIG_APP_ISM330_SPI_ONLY_INIT_PRIORITY,
	     "MMC5983MA traffic must start after the ISM330DHCX I2C block is disabled");
```

### 3.5 MMC5983MA sürücü sözleşmesi

Upstream'de bu çip için sürücü yoktur (`memsic,mmc56x3` farklı bir çiptir ve yalnızca I²C'dir). Bağlayıcı ayrıntı `MMC5983MA_SPEC.md`'dedir; buradaki tablo sürücünün dış davranışını sabitler.

| Konu | Sözleşme |
|---|---|
| Compatible / bus | `memsic,mmc5983ma`, yalnızca SPI. Binding `sensor-device.yaml` ve `spi-device.yaml`'ı içerir, isteğe bağlı `int-gpios` tanımlar |
| API | `sample_fetch`, `channel_get` ve iki özel attribute: `MMC5983MA_ATTR_BRIDGE_OFFSET_REFRESH` (`attr_set`), `MMC5983MA_ATTR_BRIDGE_OFFSET` (`attr_get`). Tetikleyici API'si yoktur |
| Kanallar | `SENSOR_CHAN_MAGN_X/Y/Z/XYZ`, birim **Gauss**, 18 bit çözünürlük, sensör çerçevesi M. Diğer kanallar `-ENOTSUP` |
| Ölçüm modeli | Boru hattı: her `sample_fetch` bir önceki çağrıda başlatılan ölçümü okur ve bir sonrakini başlatır. Çağrı ölçüm süresince beklemez |
| Hazır değil | `Meas_M_Done` = 0 ise `-EBUSY` döner, veri güncellenmez; süren ölçüm bir sonraki çağrıda okunur. Uçuşta ölçüm yoksa çağrı ölçümü başlatır ve `-EBUSY` döner. Init boru hattını başlattığı için normalde ilk çağrı da veri döndürür |
| Doyma | Bir eksen 8 G sınırındaysa `-ERANGE` döner, veri güncellenmez, sonraki ölçümden önce SET yapılır |
| Köprü ofseti | SET/RESET ile sürücüde ölçülür (init'te ve attribute ile istenince, ≈ 11 ms bloklar) ve her örnekten çıkarılır; periyodik SET yapılan çağrı 1 ms uyur (`MMC5983MA_SPEC.md §8`) |
| Init | Bus hazır mı → yazılım reset → Product ID (`0x2F` = `0x30`) doğrulama → OTP kontrolü → konfigürasyon → köprü ofseti. Bus hazır değilse veya kimlik uyuşmazsa `-ENODEV`, SPI hatası veya OTP yüklenmemişse `-EIO` (`MMC5983MA_SPEC.md §9.6`) |
| Bellek ve bağlam | Heap yok, iş kuyruğu yok, kesme yok. Yalnızca thread bağlamından çağrılır; API çağrıları bir `k_mutex` ile sıralanır |
| INT pini | Yalnızca `CONFIG_MMC5983MA_INT_DIAG=y` iken yapılandırılır; veri yolu ona bağlı değildir |

---

## 4. RTOS thread modeli ve determinizm

### 4.1 Yürütme bağlamları

Örnekleme saati sensörün kendisidir: ISM330DHCX INT1 üzerindeki DRDY darbesi her 9.615 ms'de bir zinciri başlatır. `k_timer` ile periyodik yoklama yapılmaz; iki bağımsız saat arasındaki kayma örnek kaybı veya tekrarı üretir.

| Bağlam | Öncelik | Yığın | İş |
|---|---|---|---|
| EXTI10 ISR | — | ISR yığını | Upstream sürücü: GPIO kesmesini kapat, semafor ver |
| `ism330dhcx` sürücü thread'i | `K_PRIO_COOP(10)` (sürücü varsayılanı) | 1024 | DRDY işleyicisini çağırır: zaman damgası, ISM okuması, `ism_frame_q` |
| Sensör thread'i | `K_PRIO_PREEMPT(2)` | 1024 | MMC okuması, Δt, kalibrasyon, eksen haritası, `imu_sample_q` |
| Füzyon thread'i | `K_PRIO_PREEMPT(3)` | 2048 | Hizalama, EKF tahmin ve düzeltme, anlık görüntü yayını |
| Telemetri thread'i | `K_PRIO_PREEMPT(5)` | 2048 | Anlık görüntüyü okuyup UART'a basar |
| Shell thread'i | 5 (`CONFIG_SHELL_THREAD_PRIORITY`) | 2048 | Komut işleme |
| Log thread'i | En düşük uygulama önceliği | Zephyr varsayılanı | Ertelenmiş log biçimleme |

Üç uygulama thread'i `main.c` içinde `K_THREAD_DEFINE(..., SYS_FOREVER_MS)` ile statik tanımlanır ve `main()` cihaz hazırlık kontrollerinden sonra `k_thread_start()` ile sırayla başlatır: telemetri → füzyon → sensör. Tetikleyici en son kurulur. `main()` bundan sonra döner. Giriş fonksiyonları modüllerdedir: `sensor_hub_run()`, `fusion_run()`, `telemetry_run()`.

Yığın boyutları başlangıç değerleridir. Kabul ölçütü: `debug.conf` ile ölçülen en yüksek kullanım boyutun %70'ini aşmaz.

### 4.2 Veri akışı

```
ISM330DHCX INT1 ──► EXTI10 ISR ──► sürücü thread'i (coop)
                                     on_imu_data_ready():
                                       timestamp_cyc = k_cycle_get_32()
                                       sensor_sample_fetch(imu)          accel + gyro
                                       sensor_channel_get() x2
                                       k_msgq_put(ism_frame_q, K_NO_WAIT)
                                              │  struct ism_frame, derinlik 2
                                              ▼
                                   sensör thread'i (pri 2)
                                       sensor_sample_fetch(mag)          -EBUSY → MAG_VALID yok
                                       Δt, kalibrasyon, eksen haritası
                                       k_msgq_put(imu_sample_q, K_NO_WAIT)
                                              │  struct imu_sample, derinlik 4
                                              ▼
                                   füzyon thread'i (pri 3)
                                       ekf_predict → ekf_update_accel → ekf_update_mag
                                       fusion_publish_snapshot()         k_spinlock altında kopya
                                              │  struct fusion_snapshot
                                              ▼
                                   telemetri thread'i (pri 5) / shell
```

### 4.3 DRDY işleyicisi

Upstream `ism330dhcx` sürücüsü işleyiciyi şu döngüyle çağırır: durum register'ını oku; `XLDA` veya `GDA` set ise işleyiciyi çağır; ikisi de sıfır olana kadar tekrarla. Bayraklar yalnızca ilgili çıkış register'ları okununca temizlenir. Bu yüzden:

- İşleyici **hem ivme hem jiroskop** verisini okumak zorundadır (`sensor_sample_fetch(dev)`, yani `SENSOR_CHAN_ALL`). Yalnızca semafor veren bir işleyici sürücü thread'ini sonsuz döngüye sokar ve daha düşük öncelikli her şey aç kalır.
- Tetikleyici yalnızca `SENSOR_CHAN_ACCEL_XYZ` için kurulur. İki sensör aynı ODR'de eşzamanlıdır; jiroskop için ikinci bir tetikleyici işleyiciyi tik başına iki kez çağırtır.
- İşleyicinin ilk satırı `k_cycle_get_32()`'dir. Öncesindeki gecikme (ISR → thread uyanışı → durum okuması) her tikte aynıdır ve Δt farkında sadeleşir.
- İşleyici kooperatif bağlamda çalışır: kayan nokta kullanmaz, log basmaz, SPI işlemi dışında bloklanmaz. Kalibrasyon ve dönüşümler sensör thread'inin işidir.
- `sensor_sample_fetch` başarısız olursa çerçeve üretilmez, `bus_errors` artar.

`struct ism_frame` zaman damgasını ve sürücünün verdiği altı `struct sensor_value`'yu taşır. Verinin kuyrukla taşınması, sürücünün iç tamponunu iki thread'in aynı anda okuyup yazmasını önler.

### 4.4 Δt üretimi

Zaman kaynağı `k_cycle_get_32()`'dir: SysTick tabanlı, 170 MHz, 5.9 ns çözünürlük, 25.3 s'de bir taşma. `DWT->CYCCNT` kullanılmaz; aynı çözünürlüğü verir ama hata ayıklama biriminin elle açılmasını ister ve `native_sim` testlerinde karşılığı yoktur.

$$\Delta t_k = \frac{(\texttt{uint32\_t})(t_k - t_{ref})}{f_{cyc}}$$

| Durum | Davranış |
|---|---|
| İlk çerçeve | $t_{ref} \leftarrow t_k$, örnek üretilmez |
| $10^{-4} < \Delta t \le 0.05$ s ve kuyruğa girdi | $t_{ref} \leftarrow t_k$ |
| Aralık dışı Δt | Örnek atılır, `dt_fault++`, $t_{ref} \leftarrow t_k$ |
| `imu_sample_q` dolu | Örnek atılır, `queue_drops++`, $t_{ref}$ **ilerletilmez** |

Son satırın sonucu: `dt_s` her zaman füzyon thread'ine ulaşmış bir önceki örneğe göredir. Kuyruk taşması jiroskop entegrasyonunda zaman boşluğu bırakmaz; sonraki örnek iki tiklik aralığı taşır. Kuyruk derinliği (4 × 9.6 ms ≈ 38 ms) bilerek `EKF_DT_MAX`'ın (50 ms) altındadır.

### 4.5 Kuyruklar ve paylaşılan durum

| Nesne | Tanım | Üretici → tüketici | Doluysa / boşsa |
|---|---|---|---|
| `ism_frame_q` | `K_MSGQ_DEFINE`, `struct ism_frame`, derinlik 2 | DRDY işleyicisi → sensör thread'i | Yeni çerçeve atılır, `frame_drops++` |
| `imu_sample_q` | `K_MSGQ_DEFINE`, `struct imu_sample`, derinlik 4 | Sensör thread'i → füzyon thread'i | Yeni örnek atılır, `queue_drops++` |
| `fusion_snapshot` | `static` yapı + `struct k_spinlock` | Füzyon thread'i → telemetri, shell | Okuyucu her zaman son tutarlı kopyayı alır |

- Üreticiler her zaman `K_NO_WAIT` kullanır; yüksek öncelikli taraf düşük öncelikli tarafı beklemez.
- Sensör thread'i `ism_frame_q`'yu `K_MSEC(50)` zaman aşımıyla bekler. Zaman aşımı `-ETIMEDOUT` olarak ele alınır: `drdy_timeouts++`, ilk oluşta `LOG_ERR`.
- Füzyon thread'i `imu_sample_q`'yu `K_FOREVER` ile bekler.
- Anlık görüntü kilidi yalnızca yapı kopyası süresince tutulur. İçinde hesap, log veya çağrı yapılmaz.
- `struct ekf` ve `struct ekf_workspace` yalnızca füzyon thread'ine aittir. Başka bağlamdan EKF fonksiyonu çağrılmaz; shell komutları isteğini bir `atomic_t` bayrağıyla bildirir ve füzyon thread'i tik sınırında uygular.

### 4.6 Füzyon durum makinesi ve sayaçlar

| Durum | Davranış | Geçiş |
|---|---|---|
| `FUSION_ALIGNING` | `EKF_ALIGN_SAMPLES` örnek toplar, durağanlık testi yapar (`MATH_SPEC.md §6`) | Hizalama tamam → `ekf_init()` → `FUSION_RUNNING` |
| `FUSION_RUNNING` | Tik başına `ekf_predict` → `ekf_update_accel` → (`IMU_SAMPLE_MAG_VALID` ise) `ekf_update_mag` | Herhangi bir adım `-EDOM` döndürürse → `FUSION_ALIGNING` |

Dönüş kodlarının ele alınışı: `0` ve `-EAGAIN` normaldir; `-EINVAL` örneği atar ve `ekf_input_faults` artar; `-EDOM` filtreyi yeniden hizalamaya gönderir, `ekf_numeric_faults` artar ve bir kez `LOG_ERR` basılır.

Sayaçlar `uint32_t`'dir ve anlık görüntülerle birlikte yayınlanır (sensör hattı sayaçları `sensor_hub`'ın, diğerleri füzyonun anlık görüntüsünde):

| Kaynak | Sayaçlar |
|---|---|
| Sensör hattı | `frame_drops`, `queue_drops`, `drdy_timeouts`, `bus_errors`, `mag_not_ready` (MMC `-EBUSY`), `mag_saturated` (MMC `-ERANGE`), `dt_fault` |
| EKF | `acc_soft`, `acc_gated`, `acc_rejected`, `mag_gated`, `mag_rejected`, `tilt_resets`, `yaw_resets`, `gyro_sat`, `numeric_faults` |
| Füzyon | `ekf_input_faults`, `ekf_numeric_faults`, `align_restarts` (`RUNNING` → `ALIGNING` dönüşleri: sayısal hata veya `ekf align`) |

Sayaçlar `ekf stats` komutuyla okunur. Telemetri `T` kaydındaki `msgq_drop` alanı `frame_drops + queue_drops` toplamıdır.

### 4.7 Telemetri ve shell

- Telemetri thread'i seçili hızda (açılışta `CONFIG_APP_TELEMETRY_RATE_HZ`) uyanır, seçili moda göre `sensor_hub` veya füzyon anlık görüntüsünü okur ve tek satır basar. Üç kayıt tipi vardır: `R` (gövde çerçevesi, kalibre), `S` (sensör çerçeveleri, kalibrasyonsuz), `T` (füzyon çıkışı). Satır biçimleri `TEST_PLAN.md §4.0`'dadır; açılış modu `fusion`'dır.
- Kuaterniyon işareti yalnızca telemetri kopyasında kanonikleştirilir ($q_0 \ge 0$); filtrenin iç durumuna dokunulmaz.
- Çıktı `shell_fprintf()` ile UART shell arka ucuna yazılır. `printk` kullanılmaz; shell istemiyle ve log satırlarıyla karışır.
- 115200 baud ≈ 11.5 kB/s taşır. 20 Hz × ≈ 150 bayt (`T` kaydı) ≈ 3 kB/s kullanılır; üst sınır 50 Hz'dir.
- Shell komutları (`TEST_PLAN.md §0.4`): `telemetry mode <raw|sensor|fusion|off>`, `telemetry rate <1..50>`, `mag offset`, `mag refresh`, `ekf align`, `ekf stats` (durum, bayraklar, bias, tüm sayaçlar), `ekf mag <on|off>`.
- RTT taban yapılandırmada yoktur; telemetri kanalı ST-LINK sanal seri portudur.

### 4.8 Zaman bütçesi

Periyot 9.615 ms'dir. Her örnek bir sonraki DRDY'den önce filtreden çıkmış olmalıdır.

| Aşama | Hedef | Doğrulama |
|---|---|---|
| Tik başına toplam SPI süresi (ISM + MMC) | < 200 µs | `TEST_PLAN.md §2.8` |
| Füzyon adımı (tahmin + iki düzeltme) | < 1 ms | `TEST_PLAN.md §4.9` |
| DRDY'den anlık görüntü yayınına | < 2 ms | `TEST_PLAN.md §4.9` |
| Δt titreşimi (durağan, standart sapma) | < 20 µs | `TEST_PLAN.md §2.8` |

Bunlar tasarım hedefleridir; ölçüm sonuçları `docs/evidence/` altına yazılır. Periyodik SET yapılan tiklerde MMC sürücüsünün 1 ms uykusu SPI bütçesine dahil değildir.

---

## 5. EKF matematiksel şartnamesi (özet)

Türetmeler, Jacobian'lar, sabitlerin gerekçeleri ve referans vektörler `MATH_SPEC.md`'dedir. Bu bölüm filtrenin biçimini sabitler.

### 5.1 Konvansiyonlar

| Konu | Karar |
|---|---|
| Durum | $x = [q_0, q_1, q_2, q_3, b_{gx}, b_{gy}, b_{gz}]^T$, 7×1 |
| Kuaterniyon | Hamilton, skaler önde, $q \equiv q_{nb}$ (gövde → navigasyon) |
| Çerçeveler | Navigasyon NED, gövde FRD, Euler ZYX |
| Birimler | rad, rad/s, m/s², Gauss, s |
| Kovaryans | $P$, 7×7, satır-öncelikli `float32_t[49]` |

Sensör çerçevelerinden gövdeye geçiş işaretli permütasyon tablolarıyla yapılır (`axis_map.h`). MMC5983MA'nın datasheet çerçevesi sol-ellidir; `mmc_to_body` determinantı −1'dir ve "düzeltilmez" (`MATH_SPEC.md §2.3`).

### 5.2 Tahmin

Girdi: gövde çerçevesinde jiroskop $\omega_m$ ve $\Delta t$.

$$\hat\omega = \omega_m - b_g,\qquad q_{k+1} = q_k \otimes \exp\!\bigl(\tfrac12\hat\omega\,\Delta t\bigr),\qquad b_{g,k+1} = b_{g,k}$$

$$F=\begin{bmatrix} I_4 + \tfrac{\Delta t}{2}\Omega(\hat\omega) & -\tfrac{\Delta t}{2}\,\Xi(q_k)\\ 0_{3\times4} & I_3\end{bmatrix},\qquad P \leftarrow F P F^T + Q$$

$Q$ jiroskop açısal rastgele yürüyüşünü ve bias rastgele yürüyüşünü taşır. Kuaterniyon artımı tam üstel haritadır; küçük açıda seri açılımına geçer (`MATH_SPEC.md §7`).

### 5.3 İvmeölçer düzeltmesi

Ölçüm, özgül kuvvetin birim vektörüdür; tahmin, yerçekimi yönünün gövdedeki izdüşümüdür. Düzeltme roll ve pitch'i sabitler.

Dinamik ivme kapılaması, $e = \bigl|\lVert f_b\rVert - g_0\bigr| / g_0$ üzerinden:

| Koşul | Davranış |
|---|---|
| $e \le 0.05$ | Tam güvenle düzelt: $R_a = \sigma_a^2 I_3$ |
| $0.05 < e < 0.20$ | Zayıflat: $R_a$ doğrusal olarak 100 katına kadar şişirilir |
| $e \ge 0.20$ | Düzeltme atlanır |

Norm kapısını geçen örnek ayrıca $\chi^2_3$ inovasyon kapısından geçer; büyüklüğü ≈ g kalıp yönü değişen ivmeler burada elenir. Uzun süreli ret tilt reset'i tetikler (`MATH_SPEC.md §8`).

### 5.4 Manyetometre düzeltmesi

1. Sensör çerçevesinde hard-iron / soft-iron düzeltmesi: $m_{cal} = W\,(m - V)$. $V$ ve $W$ `tools/mag_calib.py` ile üretilip `mag_cal.h`'ye yazılır.
2. Eksen haritasıyla gövde çerçevesine geçiş.
3. Mevcut tutumla navigasyon çerçevesine döndürme ve yatay düzleme izdüşüm: $m_n = R(q)\,m_b$, düşey bileşen atılır.
4. Skaler heading inovasyonu: $y_\psi = \mathrm{wrap}\bigl(D - \operatorname{atan2}(m_{n,y}, m_{n,x})\bigr)$.

Manyetometre yalnızca **heading**'i düzeltir; roll ve pitch'in kaynağı ivmeölçerdir. Düzeltmeden önce kalibrasyon, norm, eğim ve tekillik kapıları, ardından $\chi^2_1$ inovasyon kapısı uygulanır. `MAG_CAL_VALID = 0` iken düzeltme kapalıdır ve telemetri `yaw_valid = 0` raporlar (`MATH_SPEC.md §9`).

### 5.5 Sayısal kararlılık

Her tahmin ve her kabul edilen düzeltmeden sonra, istisnasız:

1. Kuaterniyon normalizasyonu: $q \leftarrow q / \lVert q\rVert$.
2. Kovaryansın birim küre teğet uzayına izdüşümü (`MATH_SPEC.md §10`).
3. Simetri: $P \leftarrow \tfrac12(P + P^T)$.

Ek kurallar:

- Kovaryans güncellemesi Joseph formundadır: $P \leftarrow (I-KH)P(I-KH)^T + KRK^T$.
- $P$'nin tersi hiçbir yerde alınmaz. Yalnızca 3×3 $S_a$ ters çevrilir; skaler $S_\psi$ doğrudan bölünür.
- Her API çağrısı girdilerini `isfinite` ile, her adım sonu durumu ve $\mathrm{diag}(P)$'yi sonluluk ve negatif olmama açısından denetler (`MATH_SPEC.md §11`).

### 5.6 Kod yapısı

| Dosya | Sorumluluk | Bağımlılık |
|---|---|---|
| `ekf_math.[ch]` | Kuaterniyon çarpımı, üstel harita, normalizasyon, $R(q)$, $\Omega$, $\Xi$, Euler ↔ kuaterniyon, açı sarma, sonluluk kontrolleri | Düz C, `<math.h>` |
| `ekf_core.[ch]` | `struct ekf`, statik çalışma alanı, `ekf_init`, `ekf_predict`, `ekf_update_accel`, `ekf_update_mag`, `ekf_align`, kapılar ve sayaçlar | `ekf_math`, CMSIS-DSP |
| `ekf_config.h` | `MATH_SPEC.md §14` sabitleri, birebir | — |

API imzaları `MATH_SPEC.md §11`'dedir. CMSIS-DSP kuralları (`MATH_SPEC.md §13`):

- Her matris `arm_mat_init_f32()` ile statik bir tampona bağlanır.
- `arm_mat_mult_f32(A, B, C)` çağrısında `C`, `A` veya `B` ile aynı bellek olamaz.
- `arm_mat_inverse_f32()` kaynak matrisi bozar; her zaman bir kopya üzerinde çağrılır.
- Her CMSIS dönüş değeri kontrol edilir; `ARM_MATH_SUCCESS` dışındaki sonuç `-EDOM`'dur.

---

## 6. Geliştirme, doğrulama ve çalışma kuralları

### 6.1 Ortam ve derleme

Zephyr çalışma alanı depo dışındadır (`~/zephyrproject`). `west build`, `ZEPHYR_BASE` tanımlıyken depo kökünden çalışır.

```sh
source ~/zephyrproject/.venv/bin/activate
export ZEPHYR_BASE=~/zephyrproject/zephyr

west build -b nucleo_g491re app
west flash

west build -p always -b nucleo_g491re app                              # temiz derleme
west build -p always -b nucleo_g491re app -- -DEXTRA_CONF_FILE=debug.conf
```

Derleme uyarısız geçmelidir. `build/` ve `twister-out/` sürüm kontrolüne girmez.

### 6.2 Testler

```sh
west twister -T app/tests -p native_sim/native             # tüm birim testleri, host üzerinde

west build -p always -b native_sim app/tests/ekf -d build/test_ekf
./build/test_ekf/zephyr/zephyr.exe                         # tek paket

west build -p always -b nucleo_g491re app/tests/ekf -d build/test_ekf_target
west flash -d build/test_ekf_target                        # aynı testler hedefte, FPU ile
```

| Paket | Kapsam | Platform |
|---|---|---|
| `tests/ekf` | `ekf_math` ve `ekf_core`: `MATH_SPEC.md §15` referans vektörleri V1–V8, Jacobian'ların sonlu farkla karşılaştırması, değişmezler ($\lVert q\rVert = 1$, $P = P^T$, $P_{ii} \ge 0$), kapı davranışları, hata yolları (NaN girdi → `-EINVAL` ve durum değişmez) | `native_sim`, `nucleo_g491re` |
| `tests/sensor_hub` | Eksen haritası (V9), ivme ve manyetometre kalibrasyonu, Δt aralık ve taşma durumları, MMC5983MA çıkış çözümlemesi | `native_sim`, QEMU (`TEST_PLAN.md §3.0`) |
| `tests/hw/bus_id`, `tests/hw/raw_data` | Faz 1 ve Faz 2 donanım testleri (`TEST_PLAN.md`) | yalnızca `nucleo_g491re`, kart bağlıyken |

Kurallar:

- Test adı davranışı söyler: `test_predict_keeps_quaternion_normalized`, `test_accel_update_skipped_above_hard_gate`.
- Karşılaştırmalar toleranslıdır (`zassert_within`); toleranslar `MATH_SPEC.md §15`'ten alınır.
- Test, uygulamanın kaynak dosyasını derler; EKF kodunun test için kopyası tutulmaz.
- `ekf/` veya `sensor_cal.c` altındaki her davranış değişikliği aynı commit'te testini getirir.

Donanım fazları (`TEST_PLAN.md`): Faz 0 jumper ve kablolama kaydı, Faz 1 SPI bus ve sürücü doğrulaması, Faz 2 sensör hattı (işaretler, kalibrasyon, zamanlama), Faz 3 EKF birim ve simülasyon testleri, Faz 4 hedef üzerinde bütünleşik ölçümler. Bir faz, öncekinin kanıtı `docs/evidence/` altına yazılmadan başlamaz.

### 6.3 Çalışma kuralları

1. **Yarım iş teslim edilmez.** `// TODO`, `// FIXME`, boş gövde, sabit değer döndüren yer tutucu, "sonra eklenecek" notu yazılmaz. Bir modül ya şartnamedeki davranışının tamamıyla ve testiyle gelir ya da hiç gelmez. Kapsam küçültülecekse kod değil görev bölünür.
2. **Varsayım yapılmaz.** Pin, register adresi, bit alanı, zamanlama ve sabit ilgili belgeden alınır. Belgede yoksa önce belgeye, kaynağıyla (datasheet bölümü, şema, ölçüm) eklenir.
3. **Zephyr API'si ezberden yazılmaz.** Kullanılan her makro, Kconfig sembolü ve sürücü davranışı sabitlenen ağaçta (`~/zephyrproject/zephyr`) doğrulanır. Sürüm yükseltilirse §6.4 yeniden kontrol edilir.
4. **Upstream ağaç değiştirilmez.** Sürücü eksikleri uygulamada giderilir (ör. `ism330_spi_only.c`).
5. **Devicetree tek doğruluk kaynağıdır.** Pin, bus, frekans ve CS bilgisi koda sabit yazılmaz; `DEVICE_DT_GET`, `SPI_DT_SPEC_GET`, `GPIO_DT_SPEC_GET` ile alınır.
6. **Değişiklik dar tutulur.** İstenmeyen yeniden düzenleme, soyutlama katmanı, yapılandırma seçeneği veya "ileride lazım olur" kodu eklenmez.
7. **§2 her satır için geçerlidir**, test kodu ve araç betikleri dahil.
8. **Rapor dürüsttür.** Derlenmemiş kod "derleniyor", koşmamış test "geçiyor" diye bildirilmez. Donanımda doğrulanamayan her iddia açıkça öyle işaretlenir.

Teslim ölçütü:

- [ ] `west build -p always -b nucleo_g491re app` uyarısız geçer
- [ ] `west twister -T app/tests -p native_sim` tümüyle geçer
- [ ] Yeni veya değişen davranışın testi vardır
- [ ] Heap, `double` ve `float` değişken argüman yoktur
- [ ] İlgili şartname belgesi aynı commit'te güncellenmiştir
- [ ] Donanım davranışını etkileyen değişiklikte ilgili faz kanıtı yenilenmiştir

### 6.4 Bilinen tuzaklar

Zephyr ile ilgili satırlar sabitlenen ağacın kaynak kodunda ve deneme derlemesinde doğrulanmıştır; donanım satırlarının dayanağı `HARDWARE_MAP.md`'de atıf yapılan datasheet bölümleridir.

| Konu | Tuzak | Çözüm |
|---|---|---|
| ISM330DHCX tetikleyici döngüsü | Sürücü, `XLDA` ve `GDA` temizlenene kadar işleyiciyi tekrar çağırır | İşleyici ivme ve jiroskopu birlikte okur (§4.3) |
| ISM330DHCX I²C bloğu | Sürücü SPI modunda `CTRL4_C.I2C_disable` yazmaz | `ism330_spi_only.c`, öncelik 91 |
| MMC5983MA I²C bloğu | Kapatılamaz; ortak hatlardaki SPI trafiğini dinler | İlk bayt kuralı: `0x60` / `0x61` yasak |
| SPI saat bölücüsü | 10 MHz istenir, 5.3125 MHz elde edilir | Zamanlama hesapları gerçek değerle yapılır |
| Sistem zamanlayıcısı | `CONFIG_PM=y` olursa döngü sayacı 32 768 Hz'e düşer | `CONFIG_PM=n` ve `BUILD_ASSERT` |
| PA5 | Board DTS'i LED ve `pwm2` için de kullanır | `&pwm2` kapalı, `CONFIG_LED=n`, `led0` takma adı kullanılmaz |
| ISM INT1 | Pull-up çipi yanlış moda sokar | Yalnızca `GPIO_PULL_DOWN` |
| `SPI_OP_MODE_CONTROLLER` | Bu ağaçta tanımlı değil | `SPI_OP_MODE_MASTER` |
| `float` ve değişken argüman | `double`'a terfi, yazılım kayan noktası | Ölçeklenmiş tamsayı çıktısı |
| `arm_mat_inverse_f32` | Kaynak matrisi bozar | Kopya üzerinde çağır |
| CMSIS boyut kontrolü | `CONFIG_CMSIS_DSP_MATRIXCHECK` olmadan yapılmaz | `prj.conf`'ta açık |
| `-Wvla` | Zephyr `LOG_*` makrolarıyla derlenmez | Bayrak eklenmez; VLA incelemeyle denetlenir |
| Modül dışı `zephyr_library()` | `find_package(Zephyr)` sonrasında oluşturulan kütüphane bağlanmaz | Sürücü `zephyr/module.yml` ile modül olarak kaydedilir |
| Shell geçmişi | `CONFIG_SHELL_HISTORY` imaja `k_heap_alloc` çeker | `CONFIG_SHELL_HISTORY=n` |
| `testcase.yaml` içinde tab | `west build -T` YAML'ı ayrıştıramaz (twister ayrıştırır) | Yalnızca boşlukla hizalanır |
| ISM330DHCX `CTRL4_C` | Rezerv bitler 7, 4 ve 0'dır (ST `ism330dhcx_reg.h`); bit 3 `DRDY_MASK`'tir | `ism330_spi_only.c` yalnızca bit 2'yi set eder, rezervleri 0 yazar |

### 6.5 Kapsam dışı

Konum ve hız kestirimi, ivmeölçer bias durumu, çevrimiçi manyetometre kalibrasyonu, kalibrasyonun kalıcı bellekte saklanması, düşük güç modları, bootloader ve donanım watchdog'u bu sürümün kapsamında değildir. Bunlardan biri gerekirse önce bu belgeye girer.
