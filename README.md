## Geliştirme Ortamı Kurulumu

Bu proje, güvenlik araçlarının (Python kriptografi kütüphaneleri, SSL araçları) en verimli çalıştığı **Linux ortamında** geliştirilmiştir.
Windows kullanıcılarının **VirtualBox üzerine Kali Linux** kurması önerilir.

---

### A. Sanal Makine ve İşletim Sistemi (VirtualBox & Kali Linux)

1. **Oracle VM VirtualBox** uygulamasını indirip kurun.
2. **Kali Linux** (veya Ubuntu) ISO dosyasını indirin ve sanal makineye kurulumunu yapın.
3. Sanal makinenin **Ağ Ayarlarını** aşağıdaki şekilde yapılandırın:

   * Ağ Türü: **Köprü Bağdaştırıcısı (Bridged Adapter)**

Bu ayar sayesinde sanal makine, ev ağınızdan (modem/router) **gerçek bir IP adresi** alır ve **ESP32 ile aynı ağda** haberleşebilir.

---

### B. Gerekli Kütüphanelerin Kurulumu

Kali Linux açıldıktan sonra terminali açın ve aşağıdaki komutları çalıştırın:

```bash
sudo apt update
sudo apt install python3-pip
pip3 install pycryptodome
```

Bu kütüphane, firmware paketleme (şifreleme) işlemleri için gereklidir.

---

### C. Proje Klasörünün Oluşturulması (Komut Satırı ile)

Tüm proje dosyalarını tek bir yerde toplamak için terminal üzerinden proje klasörünü oluşturun:

```bash
# Masaüstüne git
cd Desktop

# Klasörü oluştur
mkdir SECURE_OTA

# Klasörün içine gir
cd SECURE_OTA
```

**Not:**

* STM32’den derlediğiniz `app.bin` dosyasını
* Size verilen `firmware_sealer.py` scriptini

bu klasörün içine kopyalayın veya sürükleyip bırakın.

---

## 3. Sistemin Çalıştırılması (Adım Adım)

### Adım 1: IP Kontrolü ve Sunucu Başlatma

ESP32’nin sunucuya bağlanabilmesi için **Kali Linux IP adresi**, ESP32 kodundaki IP ile **birebir aynı** olmalıdır.

Terminalde IP adresinizi öğrenin:

```bash
ip a
```

Çıktıda `eth0` veya `wlan0` arayüzü altında görünen:

```
inet 192.168.1.XX
```

adresini not edin.

**Kritik Kontrol:**

* Bu IP adresi, ESP32 kodundaki aşağıdaki satır ile **aynı mı?**

```c
#define BASE_URL "http://192.168.1.XX:8000"
```

Değilse ESP32 kodunu güncelleyin ve tekrar yükleyin.

IP doğruysa HTTP sunucusunu başlatın:

```bash
python3 -m http.server 8000
```

Bu noktadan sonra terminal ekranında **GET isteklerini** ve sunucu loglarını göreceksiniz.

---

### Adım 2: Cihazlara Kod Yükleme Sıralaması

Sistemin kararlı çalışması için **yükleme sırası önemlidir**.

#### 1️⃣ Önce STM32 (Target) Kodunu Yükleyin

* Bootloader yazılımını **STM32CubeIDE** veya **ST-Link Utility** ile STM32 kartına flaşlayın.

**Neden?**
ESP32 açıldığında, karşısında OTA komutlarını dinleyen **hazır bir STM32 Bootloader** bulunmalıdır.

---

#### 2️⃣ Sonra ESP32 (Bridge) Kodunu Yükleyin

* ESP-IDF veya kullandığınız geliştirme ortamı üzerinden ESP32 yazılımını yükleyin.

---

### Adım 3: Log Takibi ve Güncelleme Başlatma

ESP32’nin çalışma durumunu izlemek için **Serial Monitor** açık olmalıdır.

ESP-IDF kullanıyorsanız:

```bash
idf.py monitor
```

#### Süreç Akışı

1. ESP32 açılır ve Wi-Fi ağına bağlanır.
2. Sunucudaki `version.txt` dosyası kontrol edilir.
3. Sunucudaki sürüm, cihazdaki sürümden yüksekse terminalde:

```
NEW FIRMWARE AVAILABLE
```

mesajı görülür.

**Dikkat:**
Bu aşamada ESP32 **hemen güncellemeye başlamaz**.

* STM32’nin **Bootloader modunda** olması gerekir.
* STM32’ye **Reset atılmasını** bekler veya
* Bootloader’ın **Listening Mode** süresi içinde olmasını kontrol eder.

Koşullar sağlandığında OTA güncelleme otomatik olarak başlatılır.
