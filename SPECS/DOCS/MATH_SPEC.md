# MATH_SPEC.md — EKF ve Eksen Dönüşüm Şartnamesi

| Alan | Değer |
|---|---|
| Kapsam | Koordinat çerçeveleri, sensör→gövde dönüşümleri, kalibrasyon modelleri, 7 durumlu kuaterniyon EKF'nin tüm denklemleri, sabitler ve referans test vektörleri |
| Kod karşılığı | `src/sensor_hub/axis_map.h`, `src/sensor_hub/mag_cal.h`, `src/ekf/ekf_math.[ch]`, `src/ekf/ekf_core.[ch]`, `src/ekf/ekf_config.h` |
| Bağlı belgeler | `HARDWARE_MAP.md` (fiziksel yerleşim), `TEST_PLAN.md` (doğrulama) |

Bu dosyadaki her denklem, sabit ve işaret kodda birebir uygulanır. Bir sabit kodda değişirse önce §14'teki tablo güncellenir. §15'teki referans vektörler bu denklemlerin float64 uygulamasından üretilmiştir. Kalibrasyon ve kapılama sabitleri ise aynı denklemleri koşan bir prototiple simülasyonda doğrulanmıştır (§16).

---

## 1. Notasyon ve konvansiyonlar

| Konu | Karar |
|---|---|
| Kuaterniyon | Hamilton, skaler önde: $q = [q_0, q_1, q_2, q_3]^T$, $\lVert q \rVert = 1$ |
| Anlamı | $q \equiv q_{nb}$: gövde çerçevesindeki vektörü navigasyon çerçevesine döndürür, $v_n = R(q)\,v_b$ |
| Çarpım | $p \otimes r = [\,p_0 r_0 - \vec p\cdot\vec r,\;\; p_0\vec r + r_0\vec p + \vec p\times\vec r\,]$ |
| Euler | ZYX (yaw ψ → pitch θ → roll φ), $R = R_z(\psi)R_y(\theta)R_x(\phi)$ |
| Navigasyon çerçevesi | NED (Kuzey–Doğu–Aşağı) |
| Gövde çerçevesi | FRD (İleri–Sağ–Aşağı) |
| Birimler | rad, rad/s, m/s², Gauss (G), s. Derece yalnızca telemetri ve config arayüzünde |
| Sayı tipi | Yalnızca `float` (`float32_t`). Sabitler `f` son ekli, matematik `sqrtf/sinf/cosf/atan2f/asinf/fabsf`. `-Wdouble-promotion -Werror` |
| Açı sarma | $\mathrm{wrap}(\alpha) \in (-\pi, \pi]$ |

Dönüş matrisi:

$$
R(q)=\begin{bmatrix}
q_0^2+q_1^2-q_2^2-q_3^2 & 2(q_1q_2-q_0q_3) & 2(q_1q_3+q_0q_2)\\
2(q_1q_2+q_0q_3) & q_0^2-q_1^2+q_2^2-q_3^2 & 2(q_2q_3-q_0q_1)\\
2(q_1q_3-q_0q_2) & 2(q_2q_3+q_0q_1) & q_0^2-q_1^2-q_2^2+q_3^2
\end{bmatrix}
$$

Yardımcı matrisler. Her ikisi de $q\otimes[0,\vec w] = \Omega(\vec w)\,q = \Xi(q)\,\vec w$ özdeşliğinden gelir:

$$
\Omega(\vec w)=\begin{bmatrix}
0 & -w_x & -w_y & -w_z\\
w_x & 0 & w_z & -w_y\\
w_y & -w_z & 0 & w_x\\
w_z & w_y & -w_x & 0
\end{bmatrix},\qquad
\Xi(q)=\begin{bmatrix}
-q_1 & -q_2 & -q_3\\
q_0 & -q_3 & q_2\\
q_3 & q_0 & -q_1\\
-q_2 & q_1 & q_0
\end{bmatrix}
$$

$\Xi(q)^T q = 0$ özdeşliği §7 ve §10'daki projeksiyonun dayanağıdır.

---

## 2. Koordinat çerçeveleri

### 2.1 Kart çerçevesi B (PCB)

SparkFun Eagle dosyasının CAD çerçevesidir (`HARDWARE_MAP.md §2.1`). $+z_B$ bileşen yüzünden dışarı bakar. $+x_B$ Header A'da GND→POCI yönündedir. $+y_B$ Header A'dan Header B'ye (INT1…SCX) doğrudur.

### 2.2 ISM330DHCX çerçevesi A

Kanıtlar:

1. DS13012 Şekil 4: pin-1 köşesinden $X$ uzun kenar boyunca, $Y$ kısa kenar boyunca, $Z$ üst yüzden dışarı.
2. SparkFun LGA-14L footprint'inin tDocu katmanı: $X=+x$, $Y=+y$, $Z$ dışarı (⊙).
3. U1 kartta **180° döndürülmüş** yerleşmiştir.
4. Kartın arka serigrafisindeki X ve Y okları $-x_B$ ve $-y_B$ yönündedir.

Sonuç: $X_A=-x_B,\;\; Y_A=-y_B,\;\; Z_A=+z_B$. Bu çerçeve sağ-elli bir çerçevedir.

### 2.3 MMC5983MA çerçevesi M — dikkat: sol-elli

Kanıtlar:

1. MMC5983MA datasheet s.9 "Marking illustration": $X^+$ pin 1→4 kenarı boyunca, $Y^+$ pin 16→13 kenarı boyunca, $Z^+$ paketin içine (aşağı). Not: "Coordinates shown above indicate polarity after a SET operation."
2. SparkFun LGA-16 footprint tDocu: $X=-x$, $Y=-y$, $Z$ içeri (⊗).
3. U2 kartta döndürülmemiştir (0°).
4. SparkFun Hookup Guide uyarısı: iki çipin Z eksenleri zıt yönlüdür.

Sonuç: $X_M=-x_B,\;\; Y_M=-y_B,\;\; Z_M=-z_B$.

$X_M \times Y_M = +z_B = -Z_M$ olduğu için datasheet'te çizilen M çerçevesi **sol-ellidir**. Üç bağımsız kaynak (MEMSIC çizimi, SparkFun footprint'i, SparkFun'ın "Z ters" uyarısı) aynı sonucu veriyor. Bu yüzden M→gövde dönüşümünün determinantı **−1'dir** ve bu bir hata değildir. Kodda "det = +1 olmalı" diye düzeltme yapılması **yasaktır**. Kesin doğrulama `TEST_PLAN.md` T-RAW-10 işaret testidir.

### 2.4 Gövde çerçevesi b (FRD)

| Eksen | Tanım | Fiziksel karşılık |
|---|---|---|
| $x_b$ (ileri) | $X_A = -x_B$ | Kartın arkasındaki **X oku** |
| $y_b$ (sağ) | $-Y_A = +y_B$ | Header B (INT1…SCX) tarafı |
| $z_b$ (aşağı) | $-Z_A = -z_B$ | Bileşen yüzünün tersi |

Kart bileşen yüzü yukarıda, X oku öne bakarken tutulduğunda Header A solda, Header B sağda kalır. Kart yatay ve X oku kuzeye bakıyorsa $q = [1,0,0,0]$ olur.

### 2.5 Navigasyon çerçevesi n (NED)

- Yerçekimi: $g_n = [0, 0, +g_0]^T$, $g_0 = 9.80665$ m/s².
- Yerel manyetik alan: $m_n = B\,[\cos I\cos D,\; \cos I\sin D,\; \sin I]^T$. $D$ sapma (doğu pozitif), $I$ eğim (aşağı pozitif), $B$ toplam şiddet.
- $D$, $I$, $B$ değerleri konuma özeldir ve NOAA WMM hesaplayıcısından alınır. İstanbul için yaklaşık değerler: $D \approx +6°$, $I \approx +56°$, $B \approx 0.47$ G. **Kullanmadan önce WMM ile doğrulanmalıdır.**
- `CONFIG_APP_MAG_DECLINATION_MDEG = 0` ise yaw manyetik kuzeye göredir.

### 2.6 Dönüşüm matrisleri

$$
R_{bA}=\begin{bmatrix}1&0&0\\0&-1&0\\0&0&-1\end{bmatrix}\;(\det=+1),\qquad
R_{bM}=\begin{bmatrix}1&0&0\\0&-1&0\\0&0&+1\end{bmatrix}\;(\det=-1)
$$

$$
f_b = R_{bA}\,f_A,\qquad \omega_b = R_{bA}\,\omega_A,\qquad m_b = R_{bM}\,m_{M,cal}
$$

Uygulama kuralı: dönüşümler **işaretli permütasyon** tablosuyla uygulanır, çarpma yapılmaz.

```c
struct axis_map {
	uint8_t src[3];		/* body axis i takes sensor axis src[i] */
	int8_t sign[3];
};

static const struct axis_map ism_to_body = { .src = { 0, 1, 2 }, .sign = { 1, -1, -1 } };
static const struct axis_map mmc_to_body = { .src = { 0, 1, 2 }, .sign = { 1, -1,  1 } };
```

### 2.7 Montaj rotasyonu

Kart başka bir yönelimde bir gövdeye monte edilirse $R_{mount}$ yalnızca 90°'nin katları olarak tanımlanır, böylece sonuç yine işaretli permütasyon olur. Tablolar derleme zamanında $R_{mount}R_{bA}$ ve $R_{mount}R_{bM}$ olarak yeniden üretilir. Bu projede $R_{mount} = I_3$.

### 2.8 Beklenen işaretler (doğrulama tablosu)

Kart hareketsiz, kuzey yarımküre ($I>0$). İvmeölçer özgül kuvvet ölçer: $f = a - g$, durağan halde $f_b = -R^T g_n$.

| Duruş | $f_A$ | $f_b$ | $m_M$ işareti | $m_b$ işareti |
|---|---|---|---|---|
| Düz, bileşen yüzü yukarı, X oku kuzeye | $(0,0,+g)$ | $(0,0,-g)$ | $x>0,\; y\approx0,\; z>0$ | $x>0,\; y\approx0,\; z>0$ |
| Düz, X oku doğuya | $(0,0,+g)$ | $(0,0,-g)$ | $y>0$ | $y<0$ |
| Düz, X oku batıya | $(0,0,+g)$ | $(0,0,-g)$ | $y<0$ | $y>0$ |
| Ters (bileşen yüzü aşağı) | $(0,0,-g)$ | $(0,0,+g)$ | — | — |
| X oku yukarı (burun dik, pitch +90°) | $(+g,0,0)$ | $(+g,0,0)$ | — | — |
| Header B yukarı (sağ kanat yukarı, roll −90°) | $(0,-g,0)$ | $(0,+g,0)$ | — | — |

Kural: durağan halde **yukarı bakan eksen $+g$ okur**. Genel ifade: $f_b = g_0\,[\sin\theta,\; -\sin\phi\cos\theta,\; -\cos\phi\cos\theta]^T$.

---

## 3. Sensör modelleri ve kalibrasyon

### 3.1 İşleme sırası (sensor_hub)

1. Sürücüden SI birimlerinde ham değerler: $f_A$ [m/s²], $\omega_A$ [rad/s], $m_M$ [G]. MMC sürücüsü köprü ofsetini SET/RESET ile zaten çıkarmıştır (`MMC5983MA_SPEC.md §8`).
2. **Sensör çerçevesinde** kalibrasyon: $f_{A,cal}$ ve $m_{M,cal}$.
3. Eksen haritası (§2.6) ile gövde çerçevesine geçiş.
4. `struct imu_sample` olarak `k_msgq`'ya gönderim (§4).

Kalibrasyon katsayıları sensör çerçevesinde üretilir ve **aynı çerçevede** uygulanır. Eksen haritası değişirse kalibrasyon katsayıları geçersiz olmaz.

### 3.2 Jiroskop

$$\omega_m = \omega + b_g + n_g,\qquad \dot b_g = n_b$$

Ölçek ve eksen-arası hata modellenmez (ISM330DHCX: hassasiyet ±%2, cross-axis ±%1). Bias EKF durumudur (§5).

### 3.3 İvmeölçer

$$f_{A,cal} = S_a\,(f_A - o_a),\qquad S_a=\mathrm{diag}(s_x,s_y,s_z)$$

- Varsayılanlar: $S_a = I$, $o_a = 0$.
- 6 yüz kalibrasyonu (`TEST_PLAN.md` T-RAW-12) her eksen için $+g$ ve $-g$ okumalarından türetilir: $o_i = (f_i^+ + f_i^-)/2$, $s_i = 2g_0/(f_i^+ - f_i^-)$.
- Datasheet sıfır-g ofseti tipik ±10 mg (maks. ±65 mg) olduğundan bu kalibrasyon önerilir.

### 3.4 Manyetometre: hard-iron / soft-iron

$$m_{M,cal} = W\,(m_M - V)$$

| Sembol | Boyut | Birim | Anlamı |
|---|---|---|---|
| $V$ | 3×1 | G | Hard-iron ofseti (kart üzerindeki kalıcı mıknatıslanma) |
| $W$ | 3×3, simetrik pozitif tanımlı | — | Soft-iron düzeltmesi + ölçek. $\lVert m_{M,cal}\rVert \approx B_{ref}$ olacak şekilde ölçeklenir |
| $B_{ref}$ | skaler | G | Yerel toplam alan şiddeti (WMM). Kapılamada kullanılır |

Kurallar:

- $V$ yalnızca kartın kendi manyetik izini içerir. Köprü ofseti (sıcaklıkla kayan sensör iç ofseti) sürücüde SET/RESET ile çıkarılır. Bu yüzden kalibrasyon verisi **çalışma zamanındaki sürücüyle, aynı Kconfig ayarlarıyla** toplanmalıdır.
- Kart kutuya alınırsa, yanına pil, vida, hoparlör gibi ferromanyetik parçalar eklenirse veya montaj değişirse kalibrasyon yenilenir.
- Varsayılan derleme değerleri: $V = 0$, $W = I$, `MAG_CAL_VALID = 0`. `MAG_CAL_VALID = 0` iken EKF manyetometre güncellemesi **devre dışıdır**; yaw yalnızca jiroskoptan gelir ve telemetride `yaw_valid = 0` raporlanır.

**Elipsoid uydurma** (çevrimdışı araç: `tools/mag_calib.py`, numpy):

1. Kart her yönde yavaşça döndürülerek en az 600 adet $m_M$ örneği toplanır. Kapsama kriteri (örnek ortalaması çıkarılmış veriyle): $\pm X, \pm Y, \pm Z$ yönlerinin her biri için ilgili bileşenin $> 0.7\lVert m\rVert$ olduğu en az 30 örnek.
2. Genel ikinci derece yüzey: $m^T A m + 2\beta^T m + c = 0$. Tasarım matrisinin satırı $[x^2, y^2, z^2, 2yz, 2xz, 2xy, 2x, 2y, 2z, 1]$, parametre vektörü $u = [A_{11}, A_{22}, A_{33}, A_{23}, A_{13}, A_{12}, \beta_1, \beta_2, \beta_3, c]$. $u$, tasarım matrisinin en küçük tekil değerine ait sağ tekil vektördür. $A$ pozitif tanımlı değilse $u \leftarrow -u$.
3. $V = -A^{-1}\beta$, $\;k = V^T A V - c$.
4. $A/k = U\Lambda U^T$ öz-ayrışımı ile $W = B_{ref}\, U \Lambda^{1/2} U^T$. Böylece $\lVert W(m-V)\rVert = B_{ref}$ olur.
5. Kabul kriterleri: $\mathrm{std}(\lVert m_{cal}\rVert)/B_{ref} < 0.02$ ve $\mathrm{cond}(W) < 1.5$. İkincisi sağlanmazsa veri büyük olasılıkla bozuktur (yakında demir parça, eksik kapsama).
6. Çıktı `src/sensor_hub/mag_cal.h` içine C dizileri olarak yazılır: `MAG_CAL_V[3]`, `MAG_CAL_W[3][3]`, `MAG_CAL_FIELD_NORM_G`, `MAG_CAL_VALID 1`.

$W$'nin simetrik seçilmesi, soft-iron'un rotasyon bileşeninin gözlemlenemez olmasının sonucudur. Kalan eksen hizalama hatası MMC datasheet'inde tipik 1°, maks. 3°'dir ve heading doğruluğunun alt sınırını belirler.

---

## 4. sensor_hub → EKF arayüz sözleşmesi

```c
#define IMU_SAMPLE_MAG_VALID BIT(0)

struct imu_sample {
	uint32_t timestamp_cyc;	/* k_cycle_get_32() taken in the DRDY trigger handler */
	float dt_s;		/* timestamp delta to the previous accepted sample */
	float gyro[3];		/* rad/s, frame b */
	float accel[3];		/* m/s^2, frame b, calibrated */
	float mag[3];		/* G, frame b, hard/soft-iron corrected */
	uint32_t flags;
};
```

**Δt üretimi:**

$$\Delta t_k = \frac{(\texttt{uint32\_t})(t_k - t_{k-1})}{f_{cyc}},\qquad f_{cyc} = \texttt{sys\_clock\_hw\_cycles\_per\_sec()} = 170\,\text{MHz}$$

- İşaretsiz çıkarma taşmaya dayanıklıdır (taşma 25.3 s'de bir).
- 9.615 ms ≈ 1.63·10⁶ döngü, float mantisinde (2²⁴) tam temsil edilir.
- Geçerli aralık: $10^{-4} < \Delta t \le 0.05$ s. Dışında kalan örnek **tamamen atılır**, `dt_fault` sayacı artar, referans zaman $t_{k-1} \leftarrow t_k$ olur.
- Nominal $\Delta t = 1/104$ s.

**Manyetometre gecikmesi:** MMC sürücüsü boru hattı modunda çalışır. $k$. tikte okunan değer $(k-1)$. tikte başlatılan ölçümdür (≈ 1 tik, 9.6 ms gecikme). 90°/s'de bu ≈ 0.9° anlık heading gecikmesidir ve kabul edilir. Örnek, ait olduğu IMU örneğiyle birlikte işlenir.

---

## 5. Durum, kovaryans ve boyutlar

### 5.1 Matris boyutları

| Sembol | Boyut | CMSIS örneği | Anlamı |
|---|---|---|---|
| $x$ | 7×1 | `ekf->x` | $[q_0,q_1,q_2,q_3,b_{gx},b_{gy},b_{gz}]^T$ |
| $P$ | 7×7 | `ekf->P` | Durum kovaryansı |
| $F$ | 7×7 | `ws.F` | Durum geçiş Jacobian'ı |
| $Q$ | 7×7 | `ws.Q` | Süreç gürültüsü |
| $H_a$ | 3×7 | `ws.Ha` | İvme ölçüm Jacobian'ı |
| $R_a$ | 3×3 | `ws.Ra` | İvme ölçüm gürültüsü |
| $S_a$ | 3×3 | `ws.Sa` | İnovasyon kovaryansı |
| $K_a$ | 7×3 | `ws.Ka` | Kalman kazancı |
| $H_\psi$ | 1×7 | `ws.Hm` | Heading Jacobian'ı |
| $R_\psi$, $S_\psi$ | skaler | `float` | Heading gürültüsü, inovasyon varyansı |
| $K_\psi$ | 7×1 | `ws.Km` | Kalman kazancı |

Tüm diziler `static float32_t` ve satır-öncelikli (row-major) tutulur. Ara sonuçlar için tek bir statik çalışma alanı (`struct ekf_workspace`) kullanılır; yığında matris açılmaz.

### 5.2 Başlangıç kovaryansı $P_0$

$$
P_0 = \begin{bmatrix}\sigma_{q0}^2 I_4 & 0\\ 0 & \sigma_{b0}^2 I_3\end{bmatrix}
\quad\text{ardından §10'daki projeksiyon uygulanır.}
$$

| Parametre | Değer | Açıklama |
|---|---|---|
| $\sigma_{q0}$ | 0.05 | Kuaterniyon bileşeni. $\delta q \approx \tfrac12\delta\theta$ olduğundan ≈ 5.7° tutum belirsizliği |
| $\sigma_{b0}$ (hizalama durağan) | 1.745·10⁻³ rad/s (0.1°/s) | Bias, hizalama ortalamasından alınmıştır |
| $\sigma_{b0}$ (hizalama hareketli) | 5.236·10⁻² rad/s (3°/s) | Datasheet sıfır-hız seviyesi üst sınırı |

---

## 6. Hizalama (başlangıç durumu)

Füzyon thread'i `ALIGNING` durumunda $N_{align} = 100$ örnek (≈ 0.96 s) toplar ve ortalamaları hesaplar: $\bar f$, $\bar\omega$, $\bar m$ (yalnızca `MAG_VALID` örnekler), $\sigma_\omega$ (eksen başına standart sapma).

**Durağanlık testi:**

$$\max_i \sigma_{\omega,i} < 0.02\ \text{rad/s} \;\wedge\; \bigl\lvert \lVert\bar f\rVert - g_0\bigr\rvert  < 0.05\,g_0$$

- Durağan değilse pencere atılır ve yeniden toplanır.
- 10 pencere (≈ 10 s) boyunca durağanlık sağlanamazsa son pencereyle "hareketli" başlatma yapılır.

**Tilt:**

$$\phi_0 = \operatorname{atan2}(-\bar f_y, -\bar f_z),\qquad \theta_0 = \operatorname{atan2}\!\left(\bar f_x, \sqrt{\bar f_y^2+\bar f_z^2}\right)$$

**Yaw** (`MAG_CAL_VALID == 1`, en az 50 geçerli MMC örneği var ve $\bar m$ §9'daki norm kapısını geçiyorsa):

$$m_\ell = R_y(\theta_0)R_x(\phi_0)\,\bar m,\qquad \psi_0 = D + \operatorname{atan2}(-m_{\ell,y}, m_{\ell,x})$$

Aksi halde $\psi_0 = 0$ ve `yaw_valid = 0`. §9'daki yaw reset mekanizması manyetometre geçerli olduğunda heading'i 3 s içinde yakalar.

**Euler → kuaterniyon** (yarım açıların $c_\ast=\cos(\ast/2)$, $s_\ast=\sin(\ast/2)$):

$$
q = \begin{bmatrix}
c_\phi c_\theta c_\psi + s_\phi s_\theta s_\psi\\
s_\phi c_\theta c_\psi - c_\phi s_\theta s_\psi\\
c_\phi s_\theta c_\psi + s_\phi c_\theta s_\psi\\
c_\phi c_\theta s_\psi - s_\phi s_\theta c_\psi
\end{bmatrix}
$$

**Bias:** durağan ise $b_0 = \bar\omega$, değilse $b_0 = 0$. $P_0$ değeri §5.2'ye göre seçilir.

---

## 7. Tahmin adımı

Girdi: $\omega_m$ (gövde), $\Delta t$.

1. $\hat\omega = \omega_m - b$
2. $\vec\vartheta = \hat\omega\,\Delta t$, $\;\vartheta = \lVert\vec\vartheta\rVert$
3. Tam üstel artım:
   $$\delta q = \begin{cases}[\cos\frac{\vartheta}{2},\; \frac{\sin(\vartheta/2)}{\vartheta}\vec\vartheta] & \vartheta \ge 10^{-3}\\[2pt] [1-\frac{\vartheta^2}{8},\; (\tfrac12-\tfrac{\vartheta^2}{48})\vec\vartheta] & \vartheta < 10^{-3}\end{cases}$$
4. Jacobian'lar **adım 6'dan önceki** $q_k$ ile kurulur:
   $$F=\begin{bmatrix} I_4 + \tfrac{\Delta t}{2}\Omega(\hat\omega) & -\tfrac{\Delta t}{2}\,\Xi(q_k)\\ 0_{3\times4} & I_3\end{bmatrix}$$
5. Süreç gürültüsü (açısal rastgele yürüyüş + bias rastgele yürüyüşü, çapraz terimler ihmal):
   $$Q=\begin{bmatrix}\dfrac{N_g^2\,\Delta t}{4}\,\Xi(q_k)\,\Xi(q_k)^T & 0\\ 0 & N_b^2\,\Delta t\,I_3\end{bmatrix}$$
6. $q_{k+1} = q_k \otimes \delta q$, $\;b_{k+1} = b_k$
7. $P \leftarrow F P F^T + Q$
8. §10'daki normalizasyon + projeksiyon + simetri adımı.
9. **Jiroskop doyması:** $\omega_m$'nin herhangi bir ekseninde $\lvert\omega_{m,i}\rvert \ge$ `EKF_GYRO_SAT` (500 dps ölçeğinin %95'i = 8.29 rad/s) ise §10'daki **şişirme** uygulanır ve `gyro_sat++`. Doymuş jiroskop gerçek dönüşün bir kısmını kaybeder. Şişirme, ivmeölçer ve manyetometrenin kayıp açıyı hemen geri almasını sağlar. Simülasyonda 1028 dps'lik 0.35 s'lik bir hareket (±500 dps'te kırpılmış) sonrası hata < 1°.

$\Xi(q)$ sütunları $q$'ya diktir; bu yüzden $Q$ kovaryansı birim küre teğet uzayının dışına taşımaz.

---

## 8. İvmeölçer düzeltmesi

**Ölçüm modeli** (birim vektör, büyüklük ayrıştırılmış):

$$z_a = \frac{f_b}{\lVert f_b\rVert},\qquad h_a(q) = -R(q)^T e_3 = -\begin{bmatrix}2(q_1q_3-q_0q_2)\\ 2(q_2q_3+q_0q_1)\\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}$$

$$H_a = \Bigl[\; -\begin{bmatrix}-2q_2 & 2q_3 & -2q_0 & 2q_1\\ 2q_1 & 2q_0 & 2q_3 & 2q_2\\ 2q_0 & -2q_1 & -2q_2 & 2q_3\end{bmatrix}\;\Big|\; 0_{3\times3}\;\Bigr]$$

**Yerçekimi kapılama:**

$$e = \frac{\bigl\lvert \lVert f_b\rVert - g_0\bigr\rvert }{g_0}$$

| Koşul | Davranış | Sayaç |
|---|---|---|
| $e \le \epsilon_1 = 0.05$ | $R_a = \sigma_a^2 I_3$ | — |
| $\epsilon_1 < e < \epsilon_2 = 0.20$ | $R_a = s\,\sigma_a^2 I_3$, $\;s = 1 + (\lambda-1)\dfrac{e-\epsilon_1}{\epsilon_2-\epsilon_1}$, $\;\lambda = 100$ | `acc_soft` |
| $e \ge \epsilon_2$ | Güncelleme atlanır | `acc_gated` |

**İnovasyon kapısı:** $y = z_a - h_a$, $\;S_a = H_aPH_a^T + R_a$, $\;\mathrm{NIS} = y^TS_a^{-1}y$.

- $\mathrm{NIS} > 11.34$ ($\chi^2_3$, %99) → ret, `acc_rejected++`, `acc_reject_run++`.
- Kabul edilen her güncellemede `acc_reject_run = 0`.

Norm kapısı yalnızca büyüklüğü görür. Yönü değişen ama büyüklüğü ≈ g kalan ivmeleri (ör. 0.3 g karışık sallama) NIS kapısı yakalar. Simülasyonda NIS kapısı roll/pitch sapmasını 4.7°'den 1.0°'ye indirdi (§16).

**Tilt reset:** `acc_reject_run` ≥ 520 (5 s) ve son örnek norm kapısını geçiyorsa:

1. Roll ve pitch §6 formülleriyle anlık $f_b$'den yeniden hesaplanır, yaw korunur.
2. §10'daki şişirme uygulanır.
3. `tilt_resets++`, `acc_reject_run = 0`.

Ara bir "1 s sonra kovaryansı şişir" kuralı **yoktur**. Simülasyonda bu kural, 2 s'lik 0.5 g ivme sırasında bozuk ölçümlerin kabul edilmesine ve 27° pitch hatasına yol açtı. Kuralsız aynı senaryoda sapma 0.01°. Tilt hatasının asıl kaynağı jiroskop doymasıdır ve §7 adım 9'da ayrıca ele alınır.

Güncelleme: §10.

---

## 9. Manyetometre heading düzeltmesi

Manyetometre yalnızca **heading** düzeltir. Ölçülen alan mevcut tutumla navigasyon çerçevesine döndürülür, düşey bileşen atılır ve yatay izdüşümün yönü sapma açısıyla karşılaştırılır.

$$m_n = R(q)\,m_b,\qquad y_\psi = \mathrm{wrap}\bigl(D - \operatorname{atan2}(m_{n,y}, m_{n,x})\bigr)$$

Tahmin edilen ölçüm $h_\psi(q) = \psi(q) = \operatorname{atan2}(a, b)$ olup

$$a = 2(q_0q_3+q_1q_2),\qquad b = 1-2(q_2^2+q_3^2),\qquad a^2+b^2=\cos^2\theta$$

$$H_\psi = \Bigl[\;\frac{1}{a^2+b^2}\bigl[\,2bq_3,\;\; 2bq_2,\;\; 2bq_1+4aq_2,\;\; 2bq_0+4aq_3\,\bigr]\;\Big|\;0,0,0\;\Bigr]$$

**Kapılar** (sırayla; ilk başarısız olan güncellemeyi atlar ve `mag_gated` sayacını artırır):

| Kapı | Koşul | Neden |
|---|---|---|
| Örnek | `IMU_SAMPLE_MAG_VALID` yok | Sürücü `-EBUSY`/`-ERANGE` döndürdü. Bu kapı füzyon thread'indedir: `ekf_update_mag()` hiç çağrılmaz, `mag_gated` artmaz |
| Etkin | `use_mag == false` (`MAG_CAL_VALID = 0`, `CONFIG_APP_EKF_USE_MAG=n` veya shell `ekf mag off`) | §3.4 |
| Norm | $\bigl\lvert \lVert m_b\rVert - B_{ref}\bigr\rvert  / B_{ref} > 0.15$ | Yerel manyetik bozulma |
| Eğim | $\bigl\lvert \operatorname{atan2}(m_{n,z}, \sqrt{m_{n,x}^2+m_{n,y}^2}) - I\bigr\rvert  > 10°$ | Normu koruyan bozulma |
| Tekillik | $a^2+b^2 < 0.0302$ (yani $\lvert\theta\rvert > 80°$) | Yaw tanımsızlaşır |

**İnovasyon kapısı:** $S_\psi = H_\psi P H_\psi^T + \sigma_\psi^2$ (skaler, matris tersi yok), $\;\mathrm{NIS} = y_\psi^2/S_\psi$.

- $\mathrm{NIS} > 6.63$ ($\chi^2_1$, %99) → ret, `mag_rejected++`, `mag_reject_run++`, `mag_accept_run = 0`.
- Kabulde: `mag_reject_run = 0`, `mag_accept_run++`.

**Heading kurtarma durum makinesi.** Gerçek Dünya alanı sabittir. Bu yüzden yakınsamış bir filtrenin ısrarlı reddi büyük olasılıkla **yerel bozulmadır** ve benimsenmemelidir. Yakınsamamış bir filtrenin ısrarlı reddi ise büyük olasılıkla **yanlış heading**'dir ve düzeltilmelidir.

| Durum | Koşul | Eylem |
|---|---|---|
| `heading_converged = false` (hizalama ve her yeniden hizalama sonrası) | `mag_accept_run` ≥ 104 (1 s) | `heading_converged = true` |
| `heading_converged = false` | `mag_reject_run` ≥ 312 (3 s) | **Yaw reset:** $\psi \leftarrow \psi(q) + y_\psi$, roll/pitch korunarak $q$ yeniden kurulur; §10 şişirme; `yaw_resets++`; `mag_reject_run = 0` |
| `heading_converged = true` | `mag_reject_run` = 312 (3 s) | Yalnızca §10 şişirme (bir kez). Küçük gerçek heading hataları (≲ 16°) yeniden yakalanabilir; büyük bozulmalar reddedilmeye devam eder |
| `heading_converged = true` | `mag_reject_run` ≥ 3120 (30 s) | `heading_converged = false` (sonraki ret yaw reset yapar) |

Simülasyon sonuçları:

- Başlangıç heading hatası 120° → yaw reset, 3.5 s'de hata < 0.25°.
- 0.3 G bozulma 5 s → yaw sapması 0.24°.
- 0.3 G **kalıcı** bozulma → 30 s boyunca reddedilir, sonra benimsenir (kasıtlı: 30 s'lik tutarlı alan, ortamın kalıcı değiştiğini gösterir; yeniden kalibrasyon gerekir).

Güncelleme: §10, $K_\psi = PH_\psi^T/S_\psi$.

---

## 10. Ortak güncelleme ve normalizasyon adımı

Her kabul edilen ölçüm güncellemesi:

1. $K = PH^TS^{-1}$
2. $x \leftarrow x + K y$
3. Joseph formu: $P \leftarrow (I-KH)\,P\,(I-KH)^T + K R K^T$
4. Normalizasyon ve projeksiyon (aşağıda)

**Normalizasyon ve kovaryans projeksiyonu** (her tahmin ve her güncellemeden sonra):

$$n = \lVert q\rVert,\qquad \bar q = q/n,\qquad J = \frac{1}{n}\bigl(I_4 - \bar q\bar q^T\bigr),\qquad T = \begin{bmatrix}J&0\\0&I_3\end{bmatrix}$$

$$q \leftarrow \bar q,\qquad P \leftarrow T P T^T,\qquad P \leftarrow \tfrac12(P+P^T)$$

$J$, normalizasyon fonksiyonunun Jacobian'ıdır. Bu adım kovaryansı birim küre teğet uzayında tutar ve gözlemlenemez norm yönünde varyans birikmesini önler. Sonuçta $P_{qq}$ rank-3 olur. $P$'nin tersi hiçbir yerde alınmadığı için bu bir sorun değildir.

**Şişirme (inflate)** — §7, §8, §9'daki kurtarma kurallarının ortak işlemi:

$$P_{qq} \leftarrow \sigma_{q0}^2 I_4,\qquad P_{qb} = P_{bq}^T \leftarrow 0,\qquad \text{ardından normalizasyon/projeksiyon}$$

$P_{bb}$ korunur; bias bilgisi tutum belirsizliğinden bağımsızdır. `ekf_inflate_attitude()` adıyla tek bir fonksiyon olarak yazılır.

---

## 11. Sayısal kararlılık ve hata yönetimi

| Sağlanması gereken koşul | Ne zaman | Sağlanmazsa |
|---|---|---|
| Girdi `isfinite` (gyro, accel, mag, Δt) | Her API çağrısı başında | `-EINVAL`, durum değişmez |
| $\lVert f_b\rVert \ge 10^{-3}$, $\lVert m_b\rVert \ge 10^{-6}$ | `ekf_update_accel()`, `ekf_update_mag()` başında | `-EINVAL`, durum değişmez |
| Δt aralığı (§4) | `ekf_predict()` | `-EINVAL`, durum değişmez |
| $n = \lVert q\rVert > 10^{-6}$ | Normalizasyon | `-EDOM` → filtre `ALIGNING`'e döner |
| $S_a$ tersi alınabilir (`arm_mat_inverse_f32` = `ARM_MATH_SUCCESS`) | İvme güncellemesi | Güncelleme atlanır, `-EDOM` → `ALIGNING` |
| $S_\psi > 10^{-12}$ | Heading güncellemesi | Güncelleme atlanır, `-EDOM` → `ALIGNING` |
| $x$ ve $\mathrm{diag}(P)$ `isfinite`, $P_{ii} \ge 0$ | Her adım sonu | `-EDOM` → `ALIGNING` |

Kuaterniyon işaret kanonikleştirmesi ($q_0 \ge 0$) yalnızca **telemetri kopyasına** uygulanır. İç durumda $q \to -q$ yapılırsa $P_{qb}$ blokları da işaret değiştirmelidir; bu yüzden iç durumda yapılmaz.

**API dönüş değerleri:**

- `0` — uygulandı
- `-EAGAIN` — kapı nedeniyle atlandı/reddedildi (hata değil, sayaçlar artar)
- `-EINVAL` — geçersiz girdi
- `-EDOM` — sayısal hata

```c
struct ekf_params {
	float declination_rad;	/* D */
	float inclination_rad;	/* I */
	float field_norm_g;	/* B_ref */
	bool use_mag;		/* false also when MAG_CAL_VALID == 0 */
};

int ekf_init(struct ekf *ekf, const struct ekf_params *params,
	     const float q0[4], const float b0[3], bool stationary);
int ekf_predict(struct ekf *ekf, const float gyro_b[3], float dt_s);
int ekf_update_accel(struct ekf *ekf, const float accel_b[3]);
int ekf_update_mag(struct ekf *ekf, const float mag_b[3]);
int ekf_align(const float accel_mean[3], const float mag_mean[3], bool mag_ok,
	      float declination_rad, float q_out[4]);
void ekf_set_use_mag(struct ekf *ekf, bool use_mag);
void ekf_get_stats(const struct ekf *ekf, struct ekf_stats *out);
```

Tik başına çağrı sırası: `ekf_predict` → `ekf_update_accel` → (`MAG_VALID` ise) `ekf_update_mag`.

**Sağlık sayaçları** (`struct ekf_stats`, telemetri ve testler bunları okur; hepsi `uint32_t`, taşmada sarar):

| Sayaç | Artış koşulu |
|---|---|
| `acc_soft` | §8 yumuşak bölge ($\epsilon_1 < e < \epsilon_2$) |
| `acc_gated` | §8 sert kapı ($e \ge \epsilon_2$) |
| `acc_rejected` | §8 NIS reddi |
| `mag_gated` | §9 kapılarından biri |
| `mag_rejected` | §9 NIS reddi |
| `yaw_resets`, `tilt_resets` | §9, §8 |
| `gyro_sat` | §7 adım 9 |
| `numeric_faults` | §11 `-EDOM` |

Ek durum bayrakları: `heading_converged`, `yaw_valid`, `state` (`ALIGNING` / `RUNNING`).

---

## 12. Çıkış dönüşümleri

**Kuaterniyon → Euler:**

$$\phi = \operatorname{atan2}\bigl(2(q_0q_1+q_2q_3),\;1-2(q_1^2+q_2^2)\bigr)$$

$$\theta = \operatorname{asin}\bigl(\mathrm{clamp}(2(q_0q_2-q_1q_3), -1, 1)\bigr)$$

$$\psi = \operatorname{atan2}\bigl(2(q_0q_3+q_1q_2),\;1-2(q_2^2+q_3^2)\bigr)$$

- $\lvert\theta\rvert \to 90°$ civarında roll ve yaw ayrışamaz (gimbal lock). Bu yalnızca gösterim sorunudur; filtre kuaterniyonla çalışır.
- Yaw aralığı $(-180°, 180°]$, yön: yukarıdan bakıldığında saat yönü pozitif (NED).

**Telemetri:** float değerler `printk` ile basılmaz, çünkü değişken argüman `double`'a terfi eder ve `-Wdouble-promotion` hatası verir; M4F'de double yazılımla hesaplanır. Açılar milidereceye, kuaterniyon bileşenleri ×10⁶'ya ölçeklenmiş `int32_t` olarak basılır. Biçim: `TEST_PLAN.md §4.0`.

---

## 13. CMSIS-DSP uygulama kuralları

| Kural | Gerekçe |
|---|---|
| Her matris `arm_mat_init_f32()` ile statik bir diziye bağlanır, boyutlar §5.1 ile birebir | Determinizm, heap yasağı |
| `arm_mat_mult_f32(A, B, C)` çağrısında `C`, `A` veya `B` ile aynı bellek olamaz | CMSIS çıktı ile girdinin örtüşmesini desteklemez |
| `arm_mat_inverse_f32(src, dst)` **`src`'yi bozar** → her zaman bir kopya üzerinde çağrılır | CMSIS belgesi: "The source matrix is modified by the function" |
| Her CMSIS dönüş değeri kontrol edilir (`ARM_MATH_SUCCESS`, `ARM_MATH_SIZE_MISMATCH`, `ARM_MATH_SINGULAR`) | Sessiz boyut hatalarını yakalamak için |
| $3\times3$ $S_a$ için `arm_mat_inverse_f32`; skaler $S_\psi$ için doğrudan bölme | 1×1 matris tersi gereksiz yük |
| $F$ seyrek yapıdadır ama tam 7×7 çarpım kullanılır | 7×7×7 ≈ 343 MAC; okunabilirlik optimizasyondan önce gelir. Profil (`TEST_PLAN.md` T-DYN-12) aksini gösterirse blok çarpıma geçilir |
| Kuaterniyon, Euler ve $\Omega$, $\Xi$, $R$ yardımcıları `ekf_math.c` içinde düz C ile yazılır | Küçük sabit boyutlu işlemler için CMSIS çağrısı gereksiz |

Kconfig: `CONFIG_CMSIS_DSP=y`, `CONFIG_CMSIS_DSP_MATRIX=y`, `CONFIG_FPU=y`, `CONFIG_FPU_SHARING=y` (füzyon ve telemetri thread'leri FPU kullanır).

---

## 14. Sabitler — `src/ekf/ekf_config.h`

| Makro | Değer | Birim | Kaynak / gerekçe |
|---|---|---|---|
| `EKF_G0` | 9.80665f | m/s² | Standart yerçekimi |
| `EKF_DT_NOMINAL` | 9.615385e-3f | s | 1/104 Hz |
| `EKF_DT_MIN` | 1.0e-4f | s | §4 |
| `EKF_DT_MAX` | 0.05f | s | §4 |
| `EKF_GYRO_ARW` ($N_g$) | 1.0e-3f | rad/s/√Hz | Datasheet 5 mdps/√Hz = 8.7·10⁻⁵; titreşim, zamanlama ve model hatası için ≈11× büyütüldü |
| `EKF_GYRO_BIAS_RW` ($N_b$) | 2.0e-5f | rad/s/√s | Bias kararsızlığı 3°/h ve 0.005 dps/°C sıcaklık kayması izlenebilsin diye |
| `EKF_ACC_SIGMA` ($\sigma_a$) | 0.03f | birimsiz | ≈ 1.7° eşdeğer. Gürültü yoğunluğu (60 µg/√Hz) yerine titreşim ve doğrusal ivme baskın |
| `EKF_HEADING_SIGMA` ($\sigma_\psi$) | 0.05f | rad | ≈ 2.9°. Kalibrasyon artığı ve çip hizalama hatası (≤ 3°) baskın |
| `EKF_P0_QUAT_SIGMA` | 0.05f | — | §5.2 |
| `EKF_P0_BIAS_SIGMA_STATIC` | 1.745e-3f | rad/s | 0.1°/s |
| `EKF_P0_BIAS_SIGMA_MOVING` | 5.236e-2f | rad/s | 3°/s |
| `EKF_ACC_GATE_SOFT` ($\epsilon_1$) | 0.05f | — | §8 |
| `EKF_ACC_GATE_HARD` ($\epsilon_2$) | 0.20f | — | §8 |
| `EKF_ACC_R_INFLATE_MAX` ($\lambda$) | 100.0f | — | §8 |
| `EKF_ACC_NIS_MAX` | 11.34f | — | $\chi^2_3$, %99 |
| `EKF_MAG_NIS_MAX` | 6.63f | — | $\chi^2_1$, %99 |
| `EKF_MAG_NORM_GATE` | 0.15f | — | §9 |
| `EKF_MAG_DIP_GATE` | 0.17453f | rad | 10° |
| `EKF_MAG_MIN_COS2_PITCH` | 0.0302f | — | $\cos^2 80°$ |
| `EKF_YAW_RESET_REJECTS` | 312 | örnek | 3 s @ 104 Hz (§9) |
| `EKF_MAG_CONVERGE_ACCEPTS` | 104 | örnek | 1 s (§9) |
| `EKF_MAG_DISTRUST_REJECTS` | 3120 | örnek | 30 s (§9) |
| `EKF_TILT_RESET_REJECTS` | 520 | örnek | 5 s @ 104 Hz (§8) |
| `EKF_GYRO_SAT` | 8.29f | rad/s | 0.95 × 500 dps (§7). `gyro-range` değişirse güncellenir |
| `EKF_ALIGN_SAMPLES` | 100 | örnek | §6 |
| `EKF_ALIGN_MAG_MIN` | 50 | örnek | §6 |
| `EKF_ALIGN_GYRO_STD_MAX` | 0.02f | rad/s | §6 |
| `EKF_ALIGN_ACC_NORM_TOL` | 0.05f | — | §6 |
| `EKF_ALIGN_MAX_WINDOWS` | 10 | pencere | §6 |
| `EKF_QEXP_SMALL_ANGLE` | 1.0e-3f | rad | §7 |

Konuma ve çalışma moduna bağlı değerler `Kconfig.app`'ten gelir ve `struct ekf_params`'a (§11) aktarılır:

| Sembol | Tip | Varsayılan | Anlamı |
|---|---|---|---|
| `CONFIG_APP_MAG_DECLINATION_MDEG` | int | 0 | $D$, milidereceler, doğu pozitif. 0 → yaw manyetik kuzeye göre |
| `CONFIG_APP_MAG_INCLINATION_MDEG` | int | 56000 | $I$, milidereceler, aşağı pozitif. Varsayılan İstanbul için yaklaşık değerdir; WMM'den konuma göre ayarlanır |
| `CONFIG_APP_EKF_USE_MAG` | bool | y | `n` ise heading yalnızca jiroskoptan gelir |
| `CONFIG_APP_MAG_OFFSET_REFRESH_S` | int | 60 | MMC köprü ofseti tazeleme aralığı (`MMC5983MA_SPEC.md §9.9`) |

$B_{ref}$, `mag_cal.h` içindeki `MAG_CAL_FIELD_NORM_G` değeridir.

Q/R değerleri **başlangıç ayarıdır**. Değiştirmek için `TEST_PLAN.md` Faz 3 simülasyon testlerinin ve Faz 4 ölçümlerinin yeniden geçmesi gerekir; değişiklik bu tabloya gerekçesiyle yazılır.

---

## 15. Referans test vektörleri

Float64 uygulamadan üretilmiştir. Float32 uygulama için tolerans: kuaterniyon ve birim vektörlerde 1·10⁻⁵, Jacobian elemanlarında 1·10⁻⁴.

**V1 — Euler → q.** $\phi = 30°,\; \theta = -20°,\; \psi = 45°$:

$$q = [\,0.8616424,\;\; 0.2996729,\;\; -0.0574224,\;\; 0.4055504\,]$$

**V2 — R(q)** (V1 için):

$$R=\begin{bmatrix}0.6644630 & -0.7332948 & 0.1441097\\ 0.6644630 & 0.4914501 & -0.5629971\\ 0.3420201 & 0.4698463 & 0.8137977\end{bmatrix}$$

**V3 — Durağan ivme** (V1 tutumunda): $f_b = [-3.3540718,\; -4.6076183,\; -7.9806290]$ m/s², $\;h_a(q) = [-0.3420201,\; -0.4698463,\; -0.8137977]$.

**V4 — Manyetik alan** (V1 tutumunda, $m_n = [0.25, 0, 0.40]$ G, $D = 0$): $m_b = [0.3029238,\; 0.0046148,\; 0.3615465]$ G, $\;y_\psi = 0$.

**V5 — Hizalama:** V3 ve V4'teki $f_b, m_b$ ile §6 → $(\phi,\theta,\psi) = (30°, -20°, 45°)$.

**V6 — Jacobian'lar** (V1'de; merkezi sonlu farkla doğrulandı, hata < 2·10⁻⁹):

$$H_a^{(q)}=\begin{bmatrix}-0.1148449 & -0.8111009 & 1.7232849 & -0.5993457\\ -0.5993457 & -1.7232849 & -0.8111009 & 0.1148449\\ -1.7232849 & 0.5993457 & -0.1148449 & -0.8111009\end{bmatrix}$$

$$H_\psi^{(q)} = [\,0.6103431,\;\; -0.0864193,\;\; 0.2781614,\;\; 2.5174362\,]$$

**V7 — Üstel artım:** $\hat\omega = [0.1, -0.2, 0.3]$ rad/s, $\Delta t = 0.01$ s → $\delta q = [0.9999983,\; 0.0005000,\; -0.0010000,\; 0.0015000]$. V1 ile $q \otimes \delta q = [0.8608253,\; 0.3004226,\; -0.0585307,\; 0.4065712]$.

**V8 — Sabit yaw hızı:** $q = [1,0,0,0]$, $\omega = [0,0,0.5]$ rad/s, $\Delta t = 1/104$, 208 adım (güncelleme yok) → $\psi = 1.000000$ rad.

**V9 — Eksen haritası:** $f_A = (0,0,+9.80665) \to f_b = (0,0,-9.80665)$. $\;m_M = (1,2,3) \to m_b = (1,-2,3)$. $\;\det(\texttt{ism\_to\_body}) = +1$, $\;\det(\texttt{mmc\_to\_body}) = -1$.

---

## 16. Simülasyon doğrulaması ve bilinen sınırlamalar

Bu şartname, §7–§10'u birebir uygulayan bir float64 prototipte koşturuldu. Simülasyon koşulları: ISM330DHCX datasheet gürültüleri, 2 mG manyetometre gürültüsü, gerçek bias (0.01, −0.02, 0.015) rad/s, 104 Hz.

| Senaryo | Sonuç | TEST_PLAN eşiği |
|---|---|---|
| Durağan, $b_0 = 0$, hareketli $P_0$ | 30 s'de bias hatası < 2·10⁻⁵ rad/s, tutum hatası < 0.02° | T-EKF-06 |
| Dinamik (30°/s yaw, ±20° roll, ±15° pitch) | RMS hata < 0.09°, maks. < 0.17° | T-EKF-07 |
| 0.3 g karışık sallama, 5 s | NIS kapısı ile maks. 1.0° (kapısız 4.7°) | T-EKF-08 |
| 0.5 g sabit ivme, 2 s | NIS ile 208 ret, sapma < 0.07°, tilt reset yok | T-EKF-08 |
| 0.3 G manyetik bozulma, 5 s | Yaw sapması 0.24° | T-EKF-09 |
| 0.06 G yatay bozulma, 5 s | Yaw sapması ≈ 6° (norm/eğim kapılarından ayırt edilemez) | T-DYN-10 |
| 0.3 G kalıcı bozulma | 30 s reddedilir, sonra benimsenir (tasarım gereği) | T-EKF-09 |
| Başlangıç yaw hatası 120° | Yaw reset, 3.5 s'de hata < 0.25° | T-EKF-10 |
| 1028 dps roll hareketi, 0.35 s, ±500 dps'te kırpılmış | Doyma şişirmesiyle hareket biter bitmez hata < 1° | T-EKF-12 |

**Bilinen sınırlamalar:**

1. **Heading → roll sızıntısı.** $H_\psi$ kuaterniyon uzayında tanımlı olduğundan, eğimli durumda heading güncellemesi roll/pitch'i de hafifçe iter. 20°/−15° eğimde 1° inovasyon ≈ 0.14° roll değişimi yaratır (5° inovasyonda 0.70°). İvmeölçer bunu sonraki tiklerde düzeltir. Büyük inovasyonlar NIS kapısıyla reddedilir; yaw reset yolu (§9) yalnızca ψ'yi değiştirir, roll/pitch'e dokunmaz. Testi: T-EKF-11.
2. Manyetometre örneği ≈ 1 tik gecikmelidir (§4).
3. Uzun süreli merkezcil ivme (araç dönüşü) ≈ g normunu korursa ilk 5 s NIS kapısıyla, sonrasında tilt reset ile yanlış tilte kilitlenebilir. El tipi kullanım için kabul edilmiştir.
4. Soft-iron rotasyonu ve çip hizalama hatası (≤ 3°) heading doğruluğunun alt sınırıdır.
5. Hizalama, normu ve eğimi koruyan bir manyetik bozulmanın içinde yapılırsa filtre yanlış heading'e yakınsar. Bozulma kalkınca ölçümler 30 s boyunca reddedilir, sonra yaw reset ile düzelir. Daha hızlı kurtarma için shell komutu `ekf align` füzyonu `ALIGNING` durumuna döndürür.
