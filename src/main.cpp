/*
  Booky.
  A BlueTooth recording audiobook. RFID activated.
  
  Copyright (C) 2024 gera

  MIT license
*/

// ==> Example A2DP Receiver which uses the A2DP I2S output to an AudioKit board
#include <Wire.h>

#define I2C_SDA_IO    5
#define I2C_SCL_IO    18
#define SD_DETECT_IO  GPIO_NUM_34

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);

#include <WiFi.h>
#include <SD_MMC.h>

#ifdef USE_MFRC522
#include <MFRC522.h>
#define RC522_UART_MISO     23
#define RC522_UART_MOSI     19
#define RC522_RST           22
#endif

#include <Adafruit_PN532.h>

#include <ArduinoOTA.h>
#include "wifi_settings.h"
#include "kit.h"

void WiFi_OTA_setup() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.setHostname(myname); 
  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.begin();
}

void I2C_setup() {
  Wire.begin (I2C_SDA_IO, I2C_SCL_IO);
}

void display_setup() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
    Serial.println(F("SSD1306 allocation failed"));

  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
}

void I2C_scan() {
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void s_consume() {
  int anything = 0;
  while (Serial1.available()) {
    anything = 1;
    Serial.printf("%02x ", Serial1.read());
  }

  if (anything) Serial.printf("\n");
}

void SD_setup() {

  gpio_set_direction(SD_DETECT_IO, GPIO_MODE_INPUT);

  if(!SD_MMC.begin("/sdcard", false)){    // 1-bit mode
      Serial.println("Card Mount Failed");
  } else {
      Serial.printf("Card Mount Success.\n"
        "\tSize: %llu\n"
        "\tTotal: %llu\n"
        "\tUsed: %llu\n",SD_MMC.cardSize(), SD_MMC.totalBytes(), SD_MMC.usedBytes());

        // See file:///home/gera/.platformio/packages/framework-arduinoespressif32/libraries/SD_MMC/examples/SDMMC_Test/SDMMC_Test.ino
  }
}

void SD_loop() {
  static int prevSD = -1;
  int sd = gpio_get_level(SD_DETECT_IO);

  if (prevSD != sd) Serial.printf("SD Detect changed to: %d\n", sd);
  prevSD = sd;
}

#ifdef USE_MFRC522
MFRC522_UART mfrc522(Serial1, RC522_RST);
MFRC522 rfid(&mfrc522);

void RC522_setup() {
  Serial1.begin(9600, SERIAL_8N1, RC522_UART_MISO, RC522_UART_MOSI);

  pinMode(RC522_RST, OUTPUT);
  digitalWrite(RC522_RST, LOW);
  delayMicroseconds(2);
  digitalWrite(RC522_RST, HIGH);
  delay(50);

  Serial1.write(0x80);
  delay(2);
  Serial1.write(0x01);
  Serial1.write(0x0f);
  delay(10);

  s_consume();
  // rfid.PCD_Init();
  // rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
}

void RC522_loop() {
  const unsigned long RFID_POLLING_PERIOD_MS = 100;
  static unsigned long lastTimestamp = 0;
  unsigned long currentTimestamp = millis();

  Serial.printf("%02x %02x %02x %d\n",
    rfid.PICC_IsNewCardPresent(),
    mfrc522.PCD_ReadRegister(MFRC522::Status1Reg),
    mfrc522.PCD_ReadRegister(MFRC522::Status2Reg),
    currentTimestamp); //
  ;
/*   if ((currentTimestamp - lastTimestamp) > RFID_POLLING_PERIOD_MS) {
    lastTimestamp = currentTimestamp;

    if (rfid.PICC_IsNewCardPresent()) {
      if (rfid.PICC_ReadCardSerial()) {
        Serial.printf("Card ID: %02x %02x %02x %02x\n",
          rfid.uid.uidByte[0],
          rfid.uid.uidByte[1],
          rfid.uid.uidByte[2],
          rfid.uid.uidByte[3]);
      }
    }
  }
 */
}
#endif  // USE_MFRC522

Adafruit_PN532 nfc(-1, -1);

void PN532_printVersion() {
  uint32_t versiondata;

  versiondata = nfc.getFirmwareVersion();

  if (!versiondata) {
    Serial.println("PN53x no encontrado");
    return;
  }

  // Mostrar datos del sensor
  Serial.print("Found chip PN5");
  Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. ");
  Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.');
  Serial.println((versiondata>>8) & 0xFF, DEC);
}

void PN532_setup() {  
  nfc.begin();

  PN532_printVersion();  
  // Configurar para leer etiquetas RFID
  // nfc.setPassiveActivationRetris(1);
  nfc.SAMConfig();
}

void printArray(byte *buffer, byte bufferSize) {
   for (byte i = 0; i < bufferSize; i++) {
      Serial.print(buffer[i] < 0x10 ? " 0" : " ");
      Serial.print(buffer[i], HEX);
   }
}

void PN532_loop(uint16_t timeout) {
  boolean success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
  char uid_s[10];
  uint8_t uidLength;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength, timeout);
  
  if (success) {
    snprintf(uid_s, sizeof(uid_s), "%02X%02X-%02X%02X", uid[0], uid[1], uid[2], uid[3]);
    kit_startPlaying(uid_s);
    Serial.printf("UID (%d): %s\n", uidLength, uid_s);
    // printArray(uid, uidLength);
    display.printf(uid_s);
    display.display();
  }
}

void display_loop() {
  display.clearDisplay();
  display.setCursor(0, 0);     // Start at top-left corner
  display.printf("Le\xa2n\n%d\n\n", millis()/1000);
  display.display();
}

void setup() {
  Serial.begin(921600);

  kit_setup();

  // WiFi_OTA_setup();
  I2C_setup();
  display_setup();
  SD_setup();
  // RC522_setup();
  PN532_setup();
}

void loop() {
  int timeOnEntry = millis();

  kit_loop();
  Serial.printf("%d\n", timeOnEntry);
  if (kit_isPlaying()) {
    PN532_loop(20);
  } else {
    // ArduinoOTA.handle();

    // I2C_scan();
    SD_loop();

  #ifdef USE_MFRC522
    // RC522_loop();

    Serial1.write(MFRC522::VersionReg | 0x80);
    s_consume();
  #endif // USE_MFRC522

    display_loop();
    PN532_loop(500);

    timeOnEntry += 1000 - millis();
    if (timeOnEntry < 0) timeOnEntry = 0;
    delay(timeOnEntry);
  }
}