const char* myname = "Booky";

#include <SD_MMC.h>
#include <AudioKitHAL.h>
#include <BluetoothA2DPSink.h>

#define FILE_WRITING

AudioKit kit;
BluetoothA2DPSink a2dp_sink;

const File nofile;

File writingFile=nofile;
File playingFile=nofile;

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
  a2dp_sink.set_sample_rate_callback(nullptr);
  a2dp_sink.start(myname);  
}

int kit_isPlayingFile() {
    return !!playingFile;
}

void kit_stopPlayingFile() {
    playingFile.close();
    playingFile = nofile;
}

void kit_playChunkFromFile() {
    static char buff[1024];
    size_t read;

    read = playingFile.readBytes(buff, sizeof(buff));
    
    if (read) kit.write(buff, read);

    if (!playingFile.available()) {
        kit_stopPlayingFile();
    }
}

void kit_startPlayingFile(char *folderName) {
    String path("/");
    path += folderName;
    path += ".wav";

    if (!SD_MMC.exists(path))
        return;

    playingFile = SD_MMC.open(path, FILE_READ);

    Serial.printf("Starting to play %s. Size: %d\n", path, playingFile.size());
}

void kit_loop() {
    kit_playChunkFromFile();
}