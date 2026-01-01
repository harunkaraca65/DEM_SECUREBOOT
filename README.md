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


Secure OTA Bootloader – User Guide (English)
2. Development Environment Setup

This project is developed in a Linux environment where security tools (Python cryptography libraries, SSL utilities) operate most efficiently. Windows users are strongly recommended to install Kali Linux on VirtualBox.

A. Virtual Machine and Operating System (VirtualBox & Kali Linux)

Download and install Oracle VM VirtualBox.

Download the Kali Linux (or Ubuntu) ISO file and complete the installation inside a virtual machine.

Configure the virtual machine network adapter as Bridged Adapter.

This configuration allows the virtual machine to obtain an IP address from your local network (modem/router) and communicate directly with the ESP32 on the same network.

B. Required Library Installation

After booting Kali Linux, open a terminal and install the required Python cryptography libraries:

sudo apt update
sudo apt install python3-pip
pip3 install pycryptodome
C. Project Directory Setup (Command Line)

To keep all project files in a single location, create a directory named SECURE_OTA using the terminal:

# Navigate to Desktop
cd Desktop


# Create project directory
mkdir SECURE_OTA


# Enter the directory
cd SECURE_OTA

Note: Copy the compiled app.bin file from STM32 and the provided firmware_sealer.py script into this directory.

3. System Execution (Step-by-Step)
Step 1: IP Address Verification and Server Startup

For ESP32 to connect to the server, the IP address of Kali Linux must match the IP defined in the ESP32 firmware.

Check the IP address in the terminal:

ip a

Identify the address under eth0 or wlan0 (e.g., 192.168.1.XX).

Critical Check: Ensure this IP address matches the value defined in the ESP32 source code:

#define BASE_URL "http://192.168.1.XX:8000"

If not, update the ESP32 code and re-flash it.

Once verified, start the HTTP server:

python3 -m http.server 8000

You should now see incoming GET requests in the terminal.

Step 2: Firmware Flashing Order

Correct flashing order is essential for stable operation.

1. Flash STM32 (Target)

Upload the Bootloader firmware to the STM32 using STM32CubeIDE or ST-Link Utility.

Reason: When ESP32 boots, it expects a ready and listening STM32 bootloader.

2. Flash ESP32 (Bridge)

Upload the ESP32 firmware using ESP-IDF or your preferred development environment.

Step 3: Log Monitoring and Update Trigger

Keep the ESP32 serial monitor open to observe runtime logs:

# If using ESP-IDF
idf.py monitor

Process Flow:

ESP32 boots and connects to Wi-Fi.

It fetches and checks version.txt from the server.

If the server version is higher than the device version, the terminal prints: NEW FIRMWARE AVAILABLE

⚠️ Important: At this stage, ESP32 does not immediately start the update. It waits for the STM32 to enter Bootloader Listening Mode.

This can be achieved by:

Resetting the STM32 manually, or

Allowing the bootloader listening timeout to remain active

4. Future Work

Although the project is functionally operational, there are still planned improvements to enhance stability and security validation. The upcoming development steps are outlined below:

Jump to Slot A Stability
In the current implementation, the application jump process (jump to Slot A) is not fully stable due to the occasional EMPTY APP error.
This issue will be permanently resolved by improving vector table relocation, stack pointer validation, and flash integrity checks between the bootloader and the application.

UART Security Attack Simulation
Passive monitoring and attack simulations will be conducted on the UART line using a Kali Linux environment.
These tests will practically demonstrate and document that:

Encrypted OTA data cannot be read in plain text

Captured UART traffic appears meaningless due to encryption

No firmware content or sensitive information is leaked

With these improvements, the project will reach a higher level of runtime stability and provable communication security.
