#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>

// Forward declaration for serial debugging
void printTargets();

// ==================================================
// HEAD UNIT: ESP32-S3 + HLK-LD2450
// Sends filtered target coordinates to the wrist ESP32
// over ESP-NOW.
// ==================================================

#define RADAR_RX 16
#define RADAR_TX 17
#define FRAME_SIZE 30

const uint8_t CHANNEL = 1;
const uint8_t wristMAC[6] = {
  0x44, 0xB1, 0x76, 0xCE, 0xDE, 0x8C
};

HardwareSerial RadarSerial(1);

uint8_t frame[FRAME_SIZE];
int frameIndex = 0;

const uint8_t HEADER[] = {
  0xAA, 0xFF, 0x03, 0x00
};

const uint8_t FOOTER[] = {
  0x55, 0xCC
};

// ==================================================
// ESP-NOW PACKET
// Must match the receiver's RadarPacket structure.
// ==================================================

struct RadarPacket {
  uint32_t sequence;
  int16_t x[3];
  int16_t y[3];
  int16_t speed[3];
  uint16_t resolution[3];
  uint8_t displayedMask;
};

static_assert(
  sizeof(RadarPacket) <= 250,
  "ESP-NOW packet too large"
);

uint32_t packetSequence = 0;

// ==================================================
// TARGET DATA
// ==================================================

struct Target {
  int16_t x;
  int16_t y;
  int16_t speed;
  uint16_t resolution;
  bool active;
};

Target targets[3] = {};

// ==================================================
// TARGET FILTER SETTINGS
// ==================================================

const int REQUIRED_HITS = 3;
const float SMOOTHING = 0.35f;

const float FILTER_MAX_X = 2500.0f;
const float FILTER_MAX_Y = 4000.0f;

struct TargetFilter {
  float smoothX = 0;
  float smoothY = 0;
  int consecutiveHits = 0;
  bool displayed = false;
};

TargetFilter filters[3];

// ==================================================
// RADAR DECODING
// ==================================================

int16_t decodeSignedMagnitude(
  uint8_t low,
  uint8_t high
) {
  uint16_t raw = ((uint16_t)high << 8) | low;
  int16_t magnitude = raw & 0x7FFF;

  if (raw & 0x8000) {
    return -magnitude;
  }

  return magnitude;
}

uint16_t decodeUnsigned(
  uint8_t low,
  uint8_t high
) {
  return ((uint16_t)high << 8) | low;
}

bool validFrame() {
  for (int i = 0; i < 4; i++) {
    if (frame[i] != HEADER[i]) {
      return false;
    }
  }

  return (
    frame[28] == FOOTER[0] &&
    frame[29] == FOOTER[1]
  );
}

void decodeTargets() {
  for (int i = 0; i < 3; i++) {
    int offset = 4 + i * 8;

    targets[i].x = decodeSignedMagnitude(
      frame[offset],
      frame[offset + 1]
    );

    targets[i].y = decodeSignedMagnitude(
      frame[offset + 2],
      frame[offset + 3]
    );

    targets[i].speed = decodeSignedMagnitude(
      frame[offset + 4],
      frame[offset + 5]
    );

    targets[i].resolution = decodeUnsigned(
      frame[offset + 6],
      frame[offset + 7]
    );

    targets[i].active = !(
      targets[i].x == 0 &&
      targets[i].y == 0
    );
  }
}

// ==================================================
// TARGET FILTERING AND SMOOTHING
// ==================================================

void filterTargets() {
  for (int i = 0; i < 3; i++) {

    bool inRange =
      targets[i].active &&
      fabsf((float)targets[i].x) <= FILTER_MAX_X &&
      fabsf((float)targets[i].y) <= FILTER_MAX_Y;
      Serial.printf(
  "T%d: X=%d Y=%d Active=%d InRange=%d Hits=%d\n",
  i + 1,
  targets[i].x,
  targets[i].y,
  targets[i].active,
  inRange,
  filters[i].consecutiveHits
);
    
    if (!inRange) {
      filters[i].consecutiveHits = 0;
      filters[i].displayed = false;
      continue;
    }

    if (filters[i].consecutiveHits == 0) {
      filters[i].smoothX = targets[i].x;
      filters[i].smoothY = targets[i].y;
    } else {
      filters[i].smoothX += SMOOTHING *
        ((float)targets[i].x - filters[i].smoothX);

      filters[i].smoothY += SMOOTHING *
        ((float)targets[i].y - filters[i].smoothY);
    }

    filters[i].consecutiveHits++;

    if (
      filters[i].consecutiveHits >= REQUIRED_HITS
    ) {
      filters[i].displayed = true;
    }
  }
}

// ==================================================
// SEND TARGET DATA TO WRIST ESP32
// ==================================================

bool sendRadarPacket() {
  RadarPacket packet = {};

  packet.sequence = packetSequence++;

  for (int i = 0; i < 3; i++) {

    packet.x[i] = filters[i].displayed
      ? (int16_t)roundf(filters[i].smoothX)
      : 0;

    packet.y[i] = filters[i].displayed
      ? (int16_t)roundf(filters[i].smoothY)
      : 0;

    packet.speed[i] = targets[i].speed;
    packet.resolution[i] = targets[i].resolution;

    if (filters[i].displayed) {
      packet.displayedMask |= (1u << i);
    }
  }

  esp_err_t result = esp_now_send(
    wristMAC,
    reinterpret_cast<const uint8_t*>(&packet),
    sizeof(packet)
  );

  if (result != ESP_OK) {
    Serial.printf(
      "ESP-NOW send error: %d\n",
      (int)result
    );

    return false;
  }

  return true;
}

// ==================================================
// SETUP
// ==================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // Configure Wi-Fi for ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(CHANNEL);
  delay(200);

  Serial.print("Radar MAC: ");
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println(
      "ESP-NOW initialization failed"
    );

    while (true) {
      delay(1000);
    }
  }

  // Register the wrist receiver as a peer
  esp_now_peer_info_t peer = {};

  memcpy(peer.peer_addr, wristMAC, 6);

  peer.channel = CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println(
      "Failed to add wrist peer"
    );

    while (true) {
      delay(1000);
    }
  }

  // Initialize HLK-LD2450 UART
  RadarSerial.begin(
    256000,
    SERIAL_8N1,
    RADAR_RX,
    RADAR_TX
  );

  Serial.println(
    "Radar transmitter ready; ESP-NOW channel 1"
  );

  Serial.println(
    "Waiting for valid LD2450 frames..."
  );
}

// ==================================================
// MAIN LOOP
// ==================================================

void loop() {
  while (RadarSerial.available()) {

    uint8_t incoming = RadarSerial.read();

    if (frameIndex < FRAME_SIZE) {
      frame[frameIndex++] = incoming;
    }

    if (frameIndex == FRAME_SIZE) {

      if (validFrame()) {

        decodeTargets();
        filterTargets();
        sendRadarPacket();
        printTargets();

        frameIndex = 0;

      } else {

        // Slide the buffer by one byte to search
        // for the next valid radar frame.
        memmove(
          frame,
          frame + 1,
          FRAME_SIZE - 1
        );

        frameIndex = FRAME_SIZE - 1;
      }
    }
  }
}

// ==================================================
// SERIAL DEBUG OUTPUT
// Defined after targets[] and filters[] so their
// declarations are available to this function.
// ==================================================

void printTargets() {
  Serial.println("--- Radar Targets ---");

  for (int i = 0; i < 3; i++) {

    Serial.print("Target ");
    Serial.print(i + 1);

    Serial.print(": Raw X=");
    Serial.print(targets[i].x);

    Serial.print(", Raw Y=");
    Serial.print(targets[i].y);

    Serial.print(", Active=");
    Serial.print(
      targets[i].active ? "Yes" : "No"
    );

    Serial.print(", Filtered=");
    Serial.print(
      filters[i].displayed ? "Yes" : "No"
    );

    if (filters[i].displayed) {

      Serial.print(", Sent X=");
      Serial.print(
        (int16_t)roundf(filters[i].smoothX)
      );

      Serial.print(", Sent Y=");
      Serial.print(
        (int16_t)roundf(filters[i].smoothY)
      );
    }

    Serial.println();
  }

  Serial.println();
}