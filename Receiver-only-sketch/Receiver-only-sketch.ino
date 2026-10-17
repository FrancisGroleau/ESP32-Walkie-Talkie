
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

// Replace with the SENDER ESP32's station MAC address.
// 14:08:08:A6:65:B4

uint8_t SENDER_MAC[6] = {
  0x14, 0x08, 0x08, 0xA6, 0x65, 0xB4
};

constexpr uint8_t WIFI_CHANNEL = 1;

// MAX98357A -- preserve your existing receiver wiring.
constexpr i2s_port_t SPK_I2S = I2S_NUM_0;
constexpr int SPK_BCLK = 14;
constexpr int SPK_WS   = 15;
constexpr int SPK_DIN  = 22;

// Onboard LED pins -- verify against your board.
constexpr int RED_LED_PIN  = 4;
constexpr int BLUE_LED_PIN = 2;
constexpr bool LED_ACTIVE_LOW = false;

// Must match the sender.
constexpr int SAMPLE_RATE = 16000;
constexpr int FRAME_SAMPLES = 320;

// Sender packet format: 320 mono PCM samples compressed to ADPCM.
struct __attribute__((packed)) AudioPacket {
  uint16_t sequence;
  int16_t predictor;
  uint8_t index;
  uint8_t data[160];
};

static_assert(sizeof(AudioPacket) == 165,
              "Audio packet size mismatch");

QueueHandle_t audioQueue = nullptr;

volatile uint32_t lastRxMs = 0;
volatile bool errorActive = false;

uint16_t lastSequence = 0;
bool haveSequence = false;
uint32_t lostPackets = 0;

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

void signalError() {
  errorActive = true;
}

void signalRx() {
  lastRxMs = millis();
}

void updateStatusLeds() {
  uint32_t now = millis();

  // Red: three rapid flashes repeating while an error is latched.
  if (errorActive) {
    uint32_t phase = now % 1000;

    bool redOn =
        phase < 100 ||
        (phase >= 200 && phase < 300) ||
        (phase >= 400 && phase < 500);

    setLed(RED_LED_PIN, redOn);
  } else {
    setLed(RED_LED_PIN, false);
  }

  // Blue: one longer flash when a valid audio packet was received.
  bool receiving = (uint32_t)(now - lastRxMs) < 180;

  if (receiving) {
    setLed(BLUE_LED_PIN, (now % 400) < 180);
  } else {
    setLed(BLUE_LED_PIN, false);
  }
}

// ============================================================
// IMA ADPCM DECODER -- MUST MATCH SENDER
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

void decodeAudio(const AudioPacket &packet, int16_t *samples) {
  int predictor = packet.predictor;
  int index = packet.index;

  if (index > 88) {
    index = 88;
  }

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
// MAX98357A I2S OUTPUT
// ============================================================

bool setupSpeakerI2S() {
  i2s_config_t config = {};

  config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  config.sample_rate = SAMPLE_RATE;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 8;
  config.dma_buf_len = 256;
  config.use_apll = false;
  config.tx_desc_auto_clear = true;
  config.fixed_mclk = 0;

  esp_err_t result = i2s_driver_install(
      SPK_I2S, &config, 0, nullptr);

  if (result != ESP_OK) {
    Serial.printf("ERROR: I2S driver install failed: %d\n",
                  (int)result);
    return false;
  }

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = SPK_BCLK;
  pins.ws_io_num = SPK_WS;
  pins.data_out_num = SPK_DIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  result = i2s_set_pin(SPK_I2S, &pins);

  if (result != ESP_OK) {
    Serial.printf("ERROR: I2S pin setup failed: %d\n",
                  (int)result);
    i2s_driver_uninstall(SPK_I2S);
    return false;
  }

  return true;
}

// ============================================================
// ESP-NOW RECEIVE CALLBACK
// ============================================================

void OnDataRecv(const uint8_t *mac,
                const uint8_t *data,
                int len) {
  // Accept packets only from the configured sender.
  if (memcmp(mac, SENDER_MAC, 6) != 0) {
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

  // Track missing packets.
  static bool haveSequence = false;
  static uint16_t lastSequence = 0;

  if (haveSequence) {
    uint16_t expected = lastSequence + 1;
    if (packet.sequence != expected) {
      uint16_t missing = packet.sequence - expected;
      lostPackets += missing;
    }
  }

  lastSequence = packet.sequence;
  haveSequence = true;

  // Signal reception and clear the error indicator.
  errorActive = false;
  signalRx();

  // Queue the packet for playback.
  if (xQueueSend(audioQueue, &packet, 0) != pdTRUE) {
    AudioPacket discarded;
    xQueueReceive(audioQueue, &discarded, 0);
    xQueueSend(audioQueue, &packet, 0);
  }
}

// ============================================================
// AUDIO PLAYBACK TASK
// ============================================================

void playbackTask(void *parameter) {
  static int16_t samples[FRAME_SAMPLES];
  static int16_t output[FRAME_SAMPLES * 2];

  AudioPacket packet;

  for (;;) {
    if (xQueueReceive(audioQueue, &packet,
                      portMAX_DELAY) != pdTRUE) {
      continue;
    }

    decodeAudio(packet, samples);

    // Duplicate mono audio to left and right speaker slots.
    for (int i = 0; i < FRAME_SAMPLES; i++) {
      output[i * 2]     = samples[i];
      output[i * 2 + 1] = samples[i];
    }

    size_t bytesWritten = 0;
    size_t totalBytes = sizeof(output);
    size_t offset = 0;

    while (offset < totalBytes) {
      esp_err_t result = i2s_write(
          SPK_I2S,
          ((uint8_t *)output) + offset,
          totalBytes - offset,
          &bytesWritten,
          portMAX_DELAY);

      if (result != ESP_OK || bytesWritten == 0) {
        signalError();
        break;
      }

      offset += bytesWritten;
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
  Serial.println("ESP-NOW audio receiver starting");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("Receiver MAC: ");
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

  if (!setupSpeakerI2S()) {
    signalError();

    while (true) {
      updateStatusLeds();
      delay(5);
    }
  }

  audioQueue = xQueueCreate(8, sizeof(AudioPacket));

  if (audioQueue == nullptr) {
    Serial.println("ERROR: Audio queue creation failed");
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

  if (esp_now_register_recv_cb(OnDataRecv) != ESP_OK) {
    Serial.println("ERROR: Receive callback registration failed");
    signalError();

    while (true) {
      updateStatusLeds();
      delay(5);
    }
  }

  BaseType_t taskResult = xTaskCreate(
      playbackTask,
      "AudioPlayback",
      4096,
      nullptr,
      3,
      nullptr);

  if (taskResult != pdPASS) {
    Serial.println("ERROR: Could not create playback task");
    signalError();

    while (true) {
      updateStatusLeds();
      delay(5);
    }
  }

  Serial.println("Receiver ready");
  Serial.println("Waiting for audio packets...");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  updateStatusLeds();
  delay(5);
}
