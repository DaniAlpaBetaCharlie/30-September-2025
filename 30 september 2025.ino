// ============================================================
// ESP8266 I2C Scanner + Identifikasi TSL2561 & SHT31
// - Scan semua alamat I2C
// - Tandai mana yang kemungkinan TSL2561 / SHT31
// - Coba inisialisasi sensor pada alamat terdeteksi
// - Cetak alamat & status ke Serial Monitor
// ============================================================

#include <Arduino.h>                    // // Fitur dasar Arduino
#include <Wire.h>                       // // I2C
#include <Adafruit_Sensor.h>            // // Abstraksi sensor Adafruit (untuk TSL2561)
#include <Adafruit_TSL2561_U.h>         // // Library TSL2561
#include <Adafruit_SHT31.h>             // // Library SHT31

// --------------------------
// Pin I2C (ESP8266)
// --------------------------
#define I2C_SDA D2                      // // SDA di D2 (GPIO4)
#define I2C_SCL D1                      // // SCL di D1 (GPIO5)

// --------------------------
// Alamat-alamat umum sensor
// --------------------------
const uint8_t SHT31_ADDR_A = 0x44;      // // SHT31 default A (ADD pin low)
const uint8_t SHT31_ADDR_B = 0x45;      // // SHT31 alternative (ADD pin high)

const uint8_t TSL_ADDR_FLOAT = 0x39;    // // TSL2561 default (ADDR floating)
const uint8_t TSL_ADDR_LOW   = 0x29;    // // TSL2561 jika ADDR ke GND
const uint8_t TSL_ADDR_HIGH  = 0x49;    // // TSL2561 jika ADDR ke VCC

// --------------------------
// Objek sensor (dibuat saat tahu alamat)
// --------------------------
Adafruit_SHT31 sht31;                   // // Objek SHT31 (alamat dipilih saat begin)
Adafruit_TSL2561_Unified *tsl = nullptr;// // Pointer ke objek TSL2561 (karena konstruktor butuh alamat saat pembuatan)

// --------------------------
// Variabel deteksi
// --------------------------
bool foundSHT = false;                  // // Apakah ada SHT31?
uint8_t shtAddr = 0;                    // // Alamat SHT31 yang terdeteksi

bool foundTSL = false;                  // // Apakah ada TSL2561?
uint8_t tslAddr = 0;                    // // Alamat TSL2561 yang terdeteksi

// --------------------------
// Helper: nama heksadesimal 0x..
// --------------------------
String toHex(uint8_t addr) {
  char buf[6];                          // // Buffer "0xHH\0"
  snprintf(buf, sizeof(buf), "0x%02X", addr); // // Format heksadesimal 2 digit
  return String(buf);                   // // Kembalikan sebagai String
}

// --------------------------
// Scan I2C: cari semua device
// --------------------------
void scanI2C() {
  Serial.println(F("\nMemulai scan I2C..."));
  uint8_t count = 0;                    // // Counter device

  for (uint8_t address = 1; address < 127; address++) { // // Cek alamat 1..126
    Wire.beginTransmission(address);    // // Mulai transmisi kosong
    uint8_t error = Wire.endTransmission(); // // 0 artinya ACK (ada perangkat)
    if (error == 0) {
      Serial.print(F("Ditemukan device pada alamat "));
      Serial.println(toHex(address));   // // Cetak alamat heksadesimal
      count++;

      // // Tandai kemungkinan SHT31
      if (address == SHT31_ADDR_A || address == SHT31_ADDR_B) {
        foundSHT = true;
        shtAddr = address;
      }

      // // Tandai kemungkinan TSL2561
      if (address == TSL_ADDR_FLOAT || address == TSL_ADDR_LOW || address == TSL_ADDR_HIGH) {
        foundTSL = true;
        tslAddr = address;
      }
    } else if (error == 4) {
      Serial.print(F("Kesalahan tak diketahui pada alamat "));
      Serial.println(toHex(address));
    }
  }

  if (count == 0) {
    Serial.println(F("Tidak ada perangkat I2C terdeteksi."));
  } else {
    Serial.printf("Total perangkat terdeteksi: %u\n", count);
  }
}

// --------------------------
// Inisialisasi SHT31 jika ada
// --------------------------
void tryInitSHT31() {
  if (!foundSHT) {
    Serial.println(F("SHT31 tidak terdeteksi saat scan."));
    return;
  }

  Serial.print(F("Mencoba inisialisasi SHT31 pada "));
  Serial.println(toHex(shtAddr));

  if (sht31.begin(shtAddr)) {           // // begin() menerima alamat 0x44/0x45
    Serial.println(F("SHT31 berhasil diinisialisasi."));
    // // Tes baca cepat
    float t = sht31.readTemperature();  // // Baca suhu
    float h = sht31.readHumidity();     // // Baca RH
    if (!isnan(t) && !isnan(h)) {
      Serial.printf("SHT31 OK -> T=%.2f°C, RH=%.1f%%\n", t, h);
    } else {
      Serial.println(F("SHT31 terinisialisasi, namun pembacaan awal gagal."));
    }
  } else {
    Serial.println(F("Gagal inisialisasi SHT31 (cek kabel/alamat)."));
  }
}

// --------------------------
// Inisialisasi TSL2561 jika ada
// --------------------------
void tryInitTSL2561() {
  if (!foundTSL) {
    Serial.println(F("TSL2561 tidak terdeteksi saat scan."));
    return;
  }

  // // TSL2561 butuh pemilihan alamat via konstanta di konstruktor
  // // Buat objek sesuai alamat terdeteksi
  sensors_event_t dummy;                // // Untuk memanggil API unified nanti (opsional)

  if (tslAddr == TSL_ADDR_LOW) {
    tsl = new Adafruit_TSL2561_Unified(TSL2561_ADDR_LOW, 12345);  // // ADDR ke GND
  } else if (tslAddr == TSL_ADDR_HIGH) {
    tsl = new Adafruit_TSL2561_Unified(TSL2561_ADDR_HIGH, 12345); // // ADDR ke VCC
  } else { // default/floating
    tsl = new Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);// // ADDR floating
  }

  Serial.print(F("Mencoba inisialisasi TSL2561 pada "));
  Serial.println(toHex(tslAddr));

  if (tsl && tsl->begin()) {            // // Coba mulai TSL2561
    Serial.println(F("TSL2561 berhasil diinisialisasi."));

    // // Set beberapa opsi wajar (gain & timing)
    tsl->setGain(TSL2561_GAIN_AUTO);    // // Gain otomatis (mudah & aman)
    tsl->setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS); // // Integrasi sedang

    // // Tes baca cepat
    sensors_event_t event;
    tsl->getEvent(&event);              // // Ambil 1 sampel lux
    if (event.light) {
      Serial.printf("TSL2561 OK -> Lux=%.2f\n", event.light);
    } else {
      Serial.println(F("TSL2561 terinisialisasi, namun pembacaan awal tidak valid (mungkin terlalu gelap?)."));
    }
  } else {
    Serial.println(F("Gagal inisialisasi TSL2561 (cek kabel/alamat)."));
  }
}

// --------------------------
// Setup
// --------------------------
void setup() {
  Serial.begin(115200);                 // // Buka Serial 115200 bps
  delay(200);                           // // Stabilkan sedikit
  Serial.println(F("\n=== I2C Scanner + SHT31 & TSL2561 Detector ==="));

  Wire.begin(I2C_SDA, I2C_SCL);         // // Mulai I2C dengan pin custom ESP8266
  delay(50);                            // // Delay kecil

  scanI2C();                            // // Pindai semua alamat I2C

  // // Informasi ringkas yang dicetak ke Serial (untuk laporan ke mentor)
  if (foundSHT) {
    Serial.print(F("Alamat SHT31 terdeteksi: "));
    Serial.println(toHex(shtAddr));
  }
  if (foundTSL) {
    Serial.print(F("Alamat TSL2561 terdeteksi: "));
    Serial.println(toHex(tslAddr));
  }

  // // Coba inisialisasi masing-masing sensor (opsional, tapi bagus untuk verifikasi)
  tryInitSHT31();                       // // Uji SHT31
  tryInitTSL2561();                     // // Uji TSL2561
}

// --------------------------
// Loop: baca periodik (opsional)
// --------------------------
unsigned long lastPrint = 0;            // // Penanda waktu
const unsigned long PRINT_MS = 3000;    // // Interval cetak 3 detik

void loop() {
  // // Contoh pembacaan periodik setelah inisialisasi (jika berhasil)
  unsigned long now = millis();
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;

    // // Baca SHT31 jika ada
    if (foundSHT && sht31.isHeaterEnabled() == 0) { // // (cek objek aktif; heaterEnabled hanya sebagai akses “dummy”)
      float t = sht31.readTemperature();
      float h = sht31.readHumidity();
      Serial.printf("[SHT31 %s] T=%.2f°C, RH=%.1f%%\n", toHex(shtAddr).c_str(), t, h);
    }

    // // Baca TSL2561 jika ada
    if (foundTSL && tsl) {
      sensors_event_t event;
      tsl->getEvent(&event);
      Serial.printf("[TSL2561 %s] Lux=%.2f\n", toHex(tslAddr).c_str(), event.light);
    }
  }
}
