
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <string.h>

// ============================================================
// CONFIGURATION
// ============================================================

// Replace with the OTHER ESP32's station MAC address.
// Example only -- do not use this example MAC as your real peer.
// C8:F0:9E:5D:59:D8
uint8_t PEER_MAC[6] = {0xC8, 0xF0, 0x9E, 0x5D, 0x59, 0xD8};

constexpr uint8_t WIFI_CHANNEL = 1;

// Microphone: ICS-43434, I2S0
constexpr i2s_port_t MIC_I2S = I2S_NUM_0;
constexpr int MIC_BCLK = 26;
constexpr int MIC_WS   = 25;
constexpr int MIC_DOUT = 33;

// Speaker: MAX98357A, I2S1
constexpr i2s_port_t SPK_I2S = I2S_NUM_1;
constexpr int SPK_BCLK = 14;
constexpr int SPK_WS   = 27;
constexpr int SPK_DIN  = 22;

// LEDs -- verify these against your board.
constexpr int RED_LED_PIN  = 4;
constexpr int BLUE_LED_PIN = 2;

// Set true for an LED that turns on when its GPIO is LOW.
constexpr bool LED_ACTIVE_LOW = false;

// Audio configuration
constexpr int SAMPLE_RATE = 16000;
constexpr int FRAME_SAMPLES = 320;  // 20 ms at 16 kHz
constexpr float MIC_GAIN = 2.0f;

// ESP-NOW payload: 165 bytes, below the v1 250-byte limit.
struct __attribute__((packed)) AudioPacket {
  uint16_t sequence;
  int16_t predictor;
  uint8_t index;
  uint8_t data[160];
};

static_assert(sizeof(AudioPacket) == 165,
              "Unexpected audio packet size");

QueueHandle_t audioQueue = nullptr;

volatile uint32_t lastTxMs = 0;
volatile uint32_t lastRxMs = 0;
volatile bool errorActive = false;

uint16_t txSequence = 0;

// ============================================================
// LED STATUS
// ============================================================

void setLed(int pin, bool on) {
  digitalWrite(pin,
      (on != LED_ACTIVE_LOW) ? HIGH : LOW);
}

void initStatusLeds() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  setLed(RED_LED_PIN, false);
  setLed(BLUE_LED_PIN, false);
}

// Errors remain latched until reset, so a persistent problem
// is still visible after the original error has passed.
void signalError() {
  errorActive = true;
}

void signalTx() {
  lastTxMs = millis();
}

void signalRx() {
  lastRxMs = millis();
}

void updateStatusLeds() {
  uint32_t now = millis();

  // RED: three rapid flashes per second while an error is latched.
  if (errorActive) {
    uint32_t phase = now % 1000;

    bool redOn =
        (phase < 100) ||
        (phase >= 200 && phase < 300) ||
        (phase >= 400 && phase < 500);

    setLed(RED_LED_PIN, redOn);
  } else {
    setLed(RED_LED_PIN, false);
  }

  // BLUE: recent TX and RX activity.
  bool transmitting = (uint32_t)(now - lastTxMs) < 100;
  bool receiving    = (uint32_t)(now - lastRxMs) < 180;

  if (transmitting && receiving) {
    // Both directions active: rapid alternating indication.
    setLed(BLUE_LED_PIN, (now % 300) < 100);
  } else if (transmitting) {
    // Sending: two short flashes.
    uint32_t phase = now % 500;
    setLed(BLUE_LED_PIN,
           phase < 60 || (phase >= 120 && phase < 180));
  } else if (receiving) {
    // Receiving: one longer flash.
    setLed(BLUE_LED_PIN, (now % 400) < 180);
  } else {
    setLed(BLUE_LED_PIN, false);
  }
}

// ============================================================
// IMA ADPCM COMPRESSION / DECOMPRESSION
// ============================================================

const int imaStepTable[89] = {
  7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
  19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
  50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
  130, 143, 157, 173, 190, 209, 230, 253, 279,
  307, 337, 371, 408, 449, 494, 544, 598, 658,
  724, 796, 876, 963, 1060, 1166, 1282, 1411,
  1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024,
  3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484,
  7132, 7845, 8630, 9493, 10442, 11487, 12635,
  13899, 15289, 16818, 18500, 20350, 22385, 24623,
  27086, 29794, 32767
};

const int imaIndexTable[16] = {
  -1, -1, -1, -1, 2, 4, 6, 8,
  -1, -1, -1, -1, 2, 4, 6, 8
};

int encodeNibble(int sample, int &predictor, int &index) {
  int step = imaStepTable[index];
  int diff = sample - predictor;
  int code = 0;

  if (diff < 0) {
    code = 8;
    diff = -diff;
  }

  int delta = step >> 3;

  if (diff >= step) {
    code |= 4;
    diff -= step;
    delta += step;
  }

  if (diff >= (step >> 1)) {
    code |= 2;
    diff -= step >> 1;
    delta += step >> 1;
  }

  if (diff >= (step >> 2)) {
    code |= 1;
    delta += step >> 2;
  }

  if (code & 8) predictor -= delta;
  else          predictor += delta;

  if (predictor > 32767)  predictor = 32767;
  if (predictor < -32768) predictor = -32768;

  index += imaIndexTable[code];
  if (index < 0)  index = 0;
  if (index > 88) index = 88;

  return code;
}

int decodeNibble(int code, int &predictor, int &index) {
  int step = imaStepTable[index];
  int delta = step >> 3;

  if (code & 4) delta += step;
  if (code & 2) delta += step >> 1;
  if (code & 1) delta += step >> 2;

  if (code & 8) predictor -= delta;
  else          predictor += delta;

  if (predictor > 32767)  predictor = 32767;
  if (predictor < -32768) predictor = -32768;

  index += imaIndexTable[code];
  if (index < 0)  index = 0;
  if (index > 88) index = 88;

  return predictor;
}

void encodeAudio(const int16_t *samples, AudioPacket &packet) {
  packet.predictor = samples[0];
  packet.index = 0;
  memset(packet.data, 0, sizeof(packet.data));

  int predictor = samples[0];
  int index = 0;

  // Sample 0 is stored in predictor; encode samples 1..319.
  for (int i = 1; i < FRAME_SAMPLES; i++) {
    int code = encodeNibble(samples[i], predictor, index);
    int nibblePosition = i - 1;
    int bytePosition = nibblePosition / 2;

    if ((nibblePosition & 1) == 0) {
      packet.data[bytePosition] = code & 0x0F;
    } else {
      packet.data[bytePosition] |= (code & 0x0F) << 4;
    }
  }
}

void decodeAudio(const AudioPacket &packet, int16_t *samples) {
  int predictor = packet.predictor;
  int index = packet.index;

  samples[0] = (int16_t)predictor;

  for (int i = 1; i < FRAME_SAMPLES; i++) {
    int nibblePosition = i - 1;
    uint8_t packed = packet.data[nibblePosition / 2];

    int code;
    if ((nibblePosition & 1) == 0) {
      code = packed & 0x0F;
    } else {
      code = (packed >> 4) & 0x0F;
    }

    samples[i] = (int16_t)decodeNibble(
        code, predictor, index);
  }
}

// ============================================================
// I2S SETUP
// ============================================================

bool installMicI2S() {
  i2s_config_t config = {};

  config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  config.sample_rate = SAMPLE_RATE;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 8;
  config.dma_buf_len = 256;
  config.use_apll = false;
  config.tx_desc_auto_clear = false;
  config.fixed_mclk = 0;

  if (i2s_driver_install(MIC_I2S, &config, 0, nullptr) != ESP_OK) {
    return false;
  }

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = MIC_BCLK;
  pins.ws_io_num = MIC_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = MIC_DOUT;

  if (i2s_set_pin(MIC_I2S, &pins) != ESP_OK) {
    i2s_driver_uninstall(MIC_I2S);
    return false;
  }

  return true;
}

bool installSpeakerI2S() {
  i2s_config_t config = {};

  config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  config.sample_rate = SAMPLE_RATE;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 8;
  config.dma_buf_len = 256;
  config.use_apll = false;
  config.tx_desc_auto_clear = true;
  config.fixed_mclk = 0;

  if (i2s_driver_install(SPK_I2S, &config, 0, nullptr) != ESP_OK) {
    return false;
  }

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = SPK_BCLK;
  pins.ws_io_num = SPK_WS;
  pins.data_out_num = SPK_DIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  if (i2s_set_pin(SPK_I2S, &pins) != ESP_OK) {
    i2s_driver_uninstall(SPK_I2S);
    return false;
  }

  return true;
}

// Read the requested amount even if I2S returns partial blocks.
bool readI2SExact(void *buffer, size_t length) {
  uint8_t *dest = (uint8_t *)buffer;
  size_t received = 0;

  while (received < length) {
    size_t bytesRead = 0;
    esp_err_t result = i2s_read(
        MIC_I2S,
        dest + received,
        length - received,
        &bytesRead,
        portMAX_DELAY);

    if (result != ESP_OK || bytesRead == 0) {
      return false;
    }

    received += bytesRead;
  }

  return true;
}

bool writeI2SExact(const void *buffer, size_t length) {
  const uint8_t *src = (const uint8_t *)buffer;
  size_t sent = 0;

  while (sent < length) {
    size_t bytesWritten = 0;
    esp_err_t result = i2s_write(
        SPK_I2S,
        src + sent,
        length - sent,
        &bytesWritten,
        portMAX_DELAY);

    if (result != ESP_OK || bytesWritten == 0) {
      return false;
    }

    sent += bytesWritten;
  }

  return true;
}

// ============================================================
// ESP-NOW CALLBACKS
// ============================================================

void OnDataRecv(const uint8_t *mac,
                const uint8_t *data,
                int len) {
  // Accept packets only from the configured peer.
  if (memcmp(mac, PEER_MAC, 6) != 0) {
    return;
  }

  if (len != sizeof(AudioPacket)) {
    errorActive = true;
    return;
  }

  AudioPacket packet;
  memcpy(&packet, data, sizeof(packet));

  if (packet.index > 88) {
    errorActive = true;
    return;
  }

  // Mark that a packet was received.
  signalRx();

  // Add received audio to the playback queue, if this
  // sender sketch uses a receive/playback queue.
  if (audioQueue != nullptr) {
    if (xQueueSend(audioQueue, &packet, 0) != pdTRUE) {
      AudioPacket discarded;
      xQueueReceive(audioQueue, &discarded, 0);
      xQueueSend(audioQueue, &packet, 0);
    }
  }
}

// ============================================================
// AUDIO TASKS
// ============================================================

void transmitTask(void *parameter) {
  // Stereo 32-bit microphone slots.
  static int32_t raw[FRAME_SAMPLES * 2];
  static int16_t samples[FRAME_SAMPLES];

  AudioPacket packet = {};

  for (;;) {
    if (!readI2SExact(raw, sizeof(raw))) {
      signalError();
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    // ICS-43434 is wired for the LEFT slot.
    // Its 24-bit sample is typically left-aligned in 32 bits.
    for (int i = 0; i < FRAME_SAMPLES; i++) {
      int32_t sample = raw[i * 2] >> 16;
      sample = (int32_t)(sample * MIC_GAIN);

      if (sample > 32767)  sample = 32767;
      if (sample < -32768) sample = -32768;

      samples[i] = (int16_t)sample;
    }

    packet.sequence = txSequence++;
    encodeAudio(samples, packet);

    esp_err_t result = esp_now_send(
        PEER_MAC,
        (const uint8_t *)&packet,
        sizeof(packet));

    if (result == ESP_OK) {
      signalTx();
    } else {
      signalError();
    }
  }
}

void receiveTask(void *parameter) {
  static int16_t samples[FRAME_SAMPLES];
  static int32_t output[FRAME_SAMPLES * 2];

  AudioPacket packet;

  for (;;) {
    if (xQueueReceive(audioQueue, &packet, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    decodeAudio(packet, samples);

    // Duplicate mono audio into left and right slots.
    for (int i = 0; i < FRAME_SAMPLES; i++) {
      uint32_t word = ((uint32_t)(uint16_t)samples[i]) << 16;
      output[i * 2]     = (int32_t)word;
      output[i * 2 + 1] = (int32_t)word;
    }

    if (!writeI2SExact(output, sizeof(output))) {
      signalError();
    }
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  initStatusLeds();

  Serial.println();
  Serial.println("ESP-NOW full-duplex audio starting");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("This ESP32 MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
    Serial.println("ERROR: Could not disable Wi-Fi power save");
    signalError();
  }

  if (esp_wifi_set_channel(
          WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("ERROR: Could not set Wi-Fi channel");
    signalError();
    while (true) {
      updateStatusLeds();
      delay(5);
    }
  }

  if (!installMicI2S()) {
    Serial.println("ERROR: Microphone I2S initialization failed");
    signalError();
    while (true) {
      updateStatusLeds();
      delay(5);
    }
  }

  if (!installSpeakerI2S()) {
    Serial.println("ERROR: Speaker I2S initialization failed");
    signalError();
    while (true) {
      updateStatusLeds();
      delay(5);
    }
  }

  audioQueue = xQueueCreate(8, sizeof(AudioPacket));

  if (audioQueue == nullptr) {
    Serial.println("ERROR: Could not create audio queue");
    signalError();
    while (true) {
      updateStatusLeds();
      delay(5);
    }
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW initialization failed");
    signalError();
    while (true) {
      updateStatusLeds();
      delay(5);
    }
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, PEER_MAC, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Could not add ESP-NOW peer");
    signalError();
    while (true) {
      updateStatusLeds();
      delay(5);
    }
  }

  // Keep audio tasks independent from the LED update loop.
  BaseType_t txOk = xTaskCreate(
      transmitTask, "AudioTX", 4096, nullptr, 3, nullptr);

  BaseType_t rxOk = xTaskCreate(
      receiveTask, "AudioRX", 4096, nullptr, 3, nullptr);

  if (txOk != pdPASS || rxOk != pdPASS) {
    Serial.println("ERROR: Could not create audio tasks");
    signalError();
    while (true) {
      updateStatusLeds();
      delay(5);
    }
  }

  Serial.println("Ready: audio TX/RX tasks started");
}

void loop() {
  updateStatusLeds();
  delay(5);
}
