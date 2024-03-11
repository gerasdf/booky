/*
  Streaming Music from Bluetooth
  
  Copyright (C) 2020 Phil Schatzmann
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
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

const char* myname = "Booky";

#include <AudioKitHAL.h> // https://github.com/pschatzmann/arduino-audiokit
#include <BluetoothA2DPSink.h>

#define FILE_WRITING

AudioKit kit;
BluetoothA2DPSink a2dp_sink;

File writingFile;
const File nofile;

String title = (const char *)NULL;
String artist = (const char *)NULL;
String album = (const char *)NULL;

// Write Bluetooth data to file
void writeWAVHdr() {
#define WAV_BITRATE           44100
#define WAV_CHANNELS          2
#define WAV_BITS_PER_CHANNEL  16
#define WAV_BYTES_PER_SAMPLE  (WAV_BITS_PER_CHANNEL/8*WAV_CHANNELS)

  // Put together with my own ideas and info from
  // http://www.topherlee.com/software/pcm-tut-wavformat.html
  // https://forum.arduino.cc/t/creating-a-wav-file-header/314260


  struct soundhdr {
    char  riff[4];        /* "RIFF"                                  */
    long  flength;        /* file length in bytes                    */
    char  wave[4];        /* "WAVE"                                  */
    char  fmt_[4];         /* "fmt "                                  */
    long  chunk_size;     /* size of FMT chunk in bytes (usually 16) */
    struct {
      short format_tag;     /* 1=PCM, 257=Mu-Law, 258=A-Law, 259=ADPCM */
      short num_chans;      /* 1=mono, 2=stereo                        */
      long  srate;          /* Sampling rate in samples per second     */
      long  bytes_per_sec;  /* bytes per second = srate*bytes_per_samp */
      short bytes_per_samp; /* 2=16-bit mono, 4=16-bit stereo          */
      short bits_per_samp;  /* Number of bits per sample               */
    } fmt;
    char  data[4];        /* "data"                                  */
    long  dlength;        /* data length in bytes (filelength - 44)  */
  } wavh;
  memcpy(wavh.riff,"RIFF",4);
  memcpy(wavh.wave,"WAVE",4);
  memcpy(wavh.fmt_,"fmt ",4);
  memcpy(wavh.data,"data",4);
  
  wavh.chunk_size = sizeof(wavh.fmt);
  wavh.fmt.format_tag = 1; // PCM
  wavh.fmt.num_chans = WAV_CHANNELS;

  wavh.fmt.srate = WAV_BITRATE;
  wavh.fmt.bytes_per_samp = WAV_BYTES_PER_SAMPLE;
  wavh.fmt.bits_per_samp = WAV_BITS_PER_CHANNEL;
  wavh.fmt.bytes_per_sec = WAV_BYTES_PER_SAMPLE*WAV_BITRATE;
  
  wavh.flength = -1;    // Hack that apparently works, so we don't have to go back and write the header
  wavh.dlength = -1;    // Hack that apparently works

  if (writingFile)
    writingFile.write((uint8_t*)&wavh, sizeof wavh);
}

void receiveBTRawData(const uint8_t *data, uint32_t length) {
  if (writingFile)
    writingFile.write(data, length);
}

void recieveBTMetadata(uint8_t id, const uint8_t*text) {
  Serial.printf("Got Metadata id:%d text:%s\n", id, text);
  switch (id) {
    case ESP_AVRC_MD_ATTR_TITLE: {
      String newData = (const char*)text;
      title = newData;
      break;
    }
    case ESP_AVRC_MD_ATTR_ARTIST: {
      artist = (const char*)text;
      break;
    }
    case ESP_AVRC_MD_ATTR_ALBUM:
      String newData = (const char*)text;
      album =  newData;
      break;
    break;
  }
}

void receiveBTStatusChange(esp_avrc_playback_stat_t playback) {
  switch (playback) {
    case ESP_AVRC_PLAYBACK_PLAYING:
      Serial.println("It's Now PLAYING");
      if (writingFile) {
        writingFile.close();
        writingFile = nofile;
      }
#ifdef FILE_WRITING
      if (title) {
        writingFile = SD_MMC.open("/" + title + ".wav", FILE_WRITE);
        writeWAVHdr();
      }
#endif
      break;
    case ESP_AVRC_PLAYBACK_PAUSED:
    case ESP_AVRC_PLAYBACK_STOPPED:
      Serial.println("It's Now STOPPPED");
      if (writingFile) {
        writingFile.close();
        writingFile = nofile;
      }
      break;
    default:
      Serial.printf("Status has changerd to %d\n", playback);
  }
}

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

// void kit_setup() {
//   kit.addDefaultActions();
//   kit.addAction(kit.getKey(4), mode);
// }

void kit_setup() {

  // setup codec chip
  auto cfg = kit.defaultConfig(KitOutput);
  cfg.i2s_active = true;
  cfg.sd_active = false;

  kit.begin(cfg);
  kit.setVolume(100);

  Serial.println("Starting A2DP...");
  // define custom pins pins
  i2s_pin_config_t my_pin_config = {
      .mck_io_num = PIN_I2S_AUDIO_KIT_MCLK,
      .bck_io_num = PIN_I2S_AUDIO_KIT_BCK,
      .ws_io_num = PIN_I2S_AUDIO_KIT_WS,
      .data_out_num = PIN_I2S_AUDIO_KIT_DATA_OUT,
      .data_in_num = I2S_PIN_NO_CHANGE
  };
  a2dp_sink.set_pin_config(my_pin_config);

  if(!SD_MMC.begin("/sdcard", true)){    // 1-bit mode
      Serial.println("Card Mount Failed");
  } else {
      Serial.printf("Card Mount Success.\n"
        "\tSize: %llu\n"
        "\tTotal: %llu\n"
        "\tUsed: %llu\n",SD_MMC.cardSize(), SD_MMC.totalBytes(), SD_MMC.usedBytes());

        // See file:///home/gera/.platformio/packages/framework-arduinoespressif32/libraries/SD_MMC/examples/SDMMC_Test/SDMMC_Test.ino
  }

  a2dp_sink.set_raw_stream_reader(receiveBTRawData);
  a2dp_sink.set_avrc_metadata_callback(recieveBTMetadata);
  a2dp_sink.set_avrc_rn_playstatus_callback(receiveBTStatusChange);
  a2dp_sink.start(myname);  
}

void kit_startPlayingFolder(char *folderName) {

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

void PN532_loop() {
  boolean success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
  char uid_s[10];
  uint8_t uidLength;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength, 250);
  
  if (success) {
    snprintf(uid_s, sizeof(uid_s), "%02X%02X-%02X%02X", uid[0], uid[1], uid[2], uid[3]);
    kit_startPlayingFolder(uid_s);
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

  Serial.printf("%d\n", timeOnEntry);

  // ArduinoOTA.handle();

  // I2C_scan();
  SD_loop();
  display_loop();

#ifdef USE_MFRC522
  // RC522_loop();

  Serial1.write(MFRC522::VersionReg | 0x80);
  s_consume();
#endif // USE_MFRC522
  PN532_loop();

  timeOnEntry += 1000 - millis();
  if (timeOnEntry < 0) timeOnEntry = 0;

  delay(timeOnEntry);
}