#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define the ESP-NOW packet before any functions.
struct RadarPacket {
  uint32_t sequence;
  int16_t x[3];
  int16_t y[3];
  int16_t speed[3];
  uint16_t resolution[3];
  uint8_t displayedMask;
};

static_assert(sizeof(RadarPacket) <= 250,
              "ESP-NOW packet too large");

struct SoftwareTrack {
  bool initialized = false;
  bool currentlyDetected = false;
  int16_t x = 0;
  int16_t y = 0;
  float vx = 0;
  float vy = 0;
  uint8_t missedFrames = 0;
  uint8_t consecutiveHits = 0;
};

SoftwareTrack softwareTracks[3];

// Continue with your existing OLED settings and code.

// WRIST UNIT: ESP32 + OLED + mode button.
// Receives filtered LD2450 targets from the head unit via ESP-NOW.

// OLED SETTINGS
// ==================================================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C
#define OLED_SDA      8
#define OLED_SCL      9

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==================================================

// BUTTON / DISPLAY MODE
// Button goes between GPIO 4 and GND.
// ==================================================
const int BUTTON_PIN = 4;
const unsigned long DEBOUNCE_MS = 40;

enum DisplayMode {
  DIRECT_TRACKING,
  CONFIDENCE_MAP
};

DisplayMode displayMode = DIRECT_TRACKING;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;

// Target selection: -1 = AUTO, 0 = T1, 1 = T2, 2 = T3
int lockedTarget = -1;

const unsigned long LONG_PRESS_MS = 700;
unsigned long buttonPressStart = 0;
bool longPressHandled = false;

void handleModeButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
    lastButtonReading = reading;
  }

  // Wait for the button reading to stabilize.
  if (millis() - lastDebounceTime < DEBOUNCE_MS) {
    return;
  }

  // Handle a stable button-state change.
  if (reading != stableButtonState) {
    stableButtonState = reading;

    // Button pressed.
    if (stableButtonState == LOW) {
      buttonPressStart = millis();
      longPressHandled = false;
    }

    // Button released.
    else {
      // A short press switches display modes.
      if (!longPressHandled) {
        displayMode =
          (displayMode == DIRECT_TRACKING)
          ? CONFIDENCE_MAP
          : DIRECT_TRACKING;

        Serial.print("Display mode: ");
        Serial.println(
          displayMode == DIRECT_TRACKING
          ? "TRACK"
          : "CONF"
        );
      }
    }
  }

  // A long press cycles the selected target.
  if (
    stableButtonState == LOW &&
    !longPressHandled &&
    millis() - buttonPressStart >= LONG_PRESS_MS
  ) {
    longPressHandled = true;

    if (lockedTarget < 2) {
      lockedTarget++;
    } else {
      lockedTarget = -1;
    }

    Serial.print("Target selection: ");

    if (lockedTarget == -1) {
      Serial.println("AUTO");
    } else {
      Serial.print("T");
      Serial.println(lockedTarget + 1);
    }
  }
}

// ==================================================

// MAP SETTINGS
// ==================================================
const int MAP_LEFT      = 2;
const int MAP_RIGHT     = 78;
const int MAP_TOP       = 15;
const int MAP_BOTTOM    = 61;
const int MAP_CENTER_X  = 40;

const float MAX_X_MM = 2000.0f;  // +/- 2.0 m
const float MAX_Y_MM = 4000.0f;  // 0 to 4.0 m forward

// ==================================================

struct Target {
  int16_t x;
  int16_t y;
  int16_t speed;
  uint16_t resolution;
  bool active;
};
Target targets[3] = {};

struct TargetFilter {
  float smoothX = 0;
  float smoothY = 0;
  int consecutiveHits = 0;
  bool displayed = false;
};
TargetFilter filters[3];

static_assert(sizeof(RadarPacket) <= 250,
              "ESP-NOW packet too large");

// The rest of your sketch follows...

const uint8_t CHANNEL = 1;
const uint8_t radarMAC[6] = {0x44, 0xB1, 0x76, 0xCE, 0xCD, 0x98};
volatile bool packetPending = false;
portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;
RadarPacket incomingPacket;
uint32_t lastPacketMillis = 0;
uint32_t lastSequence = 0;
bool haveSequence = false;
bool signalLost = true;
unsigned long lastScreenUpdate = 0;

// POSITION HISTORY FOR CONFIDENCE-STYLE DISPLAY
// This estimates recent position variability, not a
// calibrated probability of detection.
// ==================================================
const int HISTORY_LENGTH = 10;

struct PositionSample {
  float x;
  float y;
};

PositionSample positionHistory[3][HISTORY_LENGTH];
int historyCount[3] = {0, 0, 0};
int historyNext[3] = {0, 0, 0};

void clearPositionHistory(int targetIndex) {
  historyCount[targetIndex] = 0;
  historyNext[targetIndex] = 0;
}

void addPositionSample(int targetIndex, float x, float y) {
  positionHistory[targetIndex][historyNext[targetIndex]].x = x;
  positionHistory[targetIndex][historyNext[targetIndex]].y = y;

  historyNext[targetIndex] =
    (historyNext[targetIndex] + 1) % HISTORY_LENGTH;

  if (historyCount[targetIndex] < HISTORY_LENGTH) {
    historyCount[targetIndex]++;
  }
}

// Approximate radial spread of recent detections around their mean.
float positionVariabilityMM(int targetIndex) {
  int count = historyCount[targetIndex];
  if (count < 2) {
    return 0.0f;
  }

  float meanX = 0.0f;
  float meanY = 0.0f;

  for (int i = 0; i < count; i++) {
    meanX += positionHistory[targetIndex][i].x;
    meanY += positionHistory[targetIndex][i].y;
  }

  meanX /= count;
  meanY /= count;

  float sumSquared = 0.0f;
  for (int i = 0; i < count; i++) {
    float dx = positionHistory[targetIndex][i].x - meanX;
    float dy = positionHistory[targetIndex][i].y - meanY;
    sumSquared += dx * dx + dy * dy;
  }

  return sqrtf(sumSquared / count);
}

// ==================================================

// COORDINATE MAPPING
// ==================================================
int mapX(int16_t x) {
  int px;

  if (x >= 0) {
    px = MAP_CENTER_X +
      (int)((float)x *
      (MAP_RIGHT - 1 - MAP_CENTER_X) / MAX_X_MM);
  } else {
    px = MAP_CENTER_X -
      (int)((float)(-x) *
      (MAP_CENTER_X - (MAP_LEFT + 1)) / MAX_X_MM);
  }

  return constrain(px, MAP_LEFT + 1, MAP_RIGHT - 1);
}

int mapY(int16_t y) {
  // The observed radar convention reports forward Y as negative.
  float forwardMM = fabsf((float)y);

  int py = MAP_BOTTOM -
    (int)(forwardMM *
    (MAP_BOTTOM - MAP_TOP) / MAX_Y_MM);

  return constrain(py, MAP_TOP + 1, MAP_BOTTOM - 1);
}

float targetDistance(int16_t x, int16_t y) {
  return sqrtf(
    (float)x * x + (float)y * y
  ) / 1000.0f;
}

// ==================================================

// DRAW RADAR GRID
// ==================================================
void drawRadarGrid() {
  display.drawRect(
    MAP_LEFT, MAP_TOP,
    MAP_RIGHT - MAP_LEFT + 1,
    MAP_BOTTOM - MAP_TOP + 1,
    SSD1306_WHITE
  );

  display.drawLine(
    MAP_CENTER_X, MAP_TOP,
    MAP_CENTER_X, MAP_BOTTOM,
    SSD1306_WHITE
  );

  int leftGuide = mapX(-1000);
  int rightGuide = mapX(1000);

  for (int y = MAP_TOP + 2; y < MAP_BOTTOM; y += 4) {
    display.drawPixel(leftGuide, y, SSD1306_WHITE);
    display.drawPixel(rightGuide, y, SSD1306_WHITE);
  }

  for (int meters = 1; meters <= 2; meters++) {
    int guideY = mapY(-meters * 1000);

    for (int x = MAP_LEFT + 2; x < MAP_RIGHT; x += 4) {
      display.drawPixel(x, guideY, SSD1306_WHITE);
    }
  }

  display.fillTriangle(
    MAP_CENTER_X, MAP_BOTTOM - 1,
    MAP_CENTER_X - 3, MAP_BOTTOM - 5,
    MAP_CENTER_X + 3, MAP_BOTTOM - 5,
    SSD1306_WHITE
  );
}

// ==================================================

// DRAW TARGETS — DIRECT TRACKING MODE
// ==================================================
void drawTargetLockMarker(int targetIndex, int px, int py) {
  if (targetIndex != lockedTarget) {
    return;
  }

  // Ring around the selected target.
  display.drawCircle(px, py, 5, SSD1306_WHITE);

  // Small horizontal and vertical targeting marks.
  display.drawLine(px - 7, py, px - 5, py, SSD1306_WHITE);
  display.drawLine(px + 5, py, px + 7, py, SSD1306_WHITE);
  display.drawLine(px, py - 7, px, py - 5, SSD1306_WHITE);
  display.drawLine(px, py + 5, px, py + 7, SSD1306_WHITE);
}

void drawTargets() {
  for (int i = 0; i < 3; i++) {
    if (!filters[i].displayed) {
      continue;
    }

    int16_t x = (int16_t)roundf(filters[i].smoothX);
    int16_t y = (int16_t)roundf(filters[i].smoothY);

    int px = mapX(x);
    int py = mapY(y);

    display.fillCircle(px, py, 2, SSD1306_WHITE);
    display.drawPixel(px - 3, py, SSD1306_WHITE);
    display.drawPixel(px + 3, py, SSD1306_WHITE);
    drawTargetLockMarker(i, px, py);
  }
}

// ==================================================

// DRAW TARGETS — CONFIDENCE-STYLE MODE
// The halo grows with recent position variability.
// It is a visual uncertainty estimate, not probability.
// ==================================================
void drawConfidenceMap() {
  for (int i = 0; i < 3; i++) {
    if (!filters[i].displayed) {
      continue;
    }

    int16_t x = (int16_t)roundf(filters[i].smoothX);
    int16_t y = (int16_t)roundf(filters[i].smoothY);

    int px = mapX(x);
    int py = mapY(y);

    float variability = positionVariabilityMM(i);

    // Convert millimetres of variation to display pixels.
    // Keep a small minimum halo so the mode is visibly distinct.
    int radius = 3 + (int)(variability * 0.012f);
    radius = constrain(radius, 3, 10);

    // Sparse concentric rings create a monochrome "gradient".
    display.drawCircle(px, py, radius, SSD1306_WHITE);
    if (radius >= 5) {
      display.drawCircle(px, py, radius - 2, SSD1306_WHITE);
    }

    // Dither the area inside the outer ring.
    for (int dy = -radius + 1; dy < radius; dy++) {
      for (int dx = -radius + 1; dx < radius; dx++) {
        int distanceSquared = dx * dx + dy * dy;
        if (distanceSquared < radius * radius &&
            ((dx + dy + i) & 1) == 0) {
          int pixelX = px + dx;
          int pixelY = py + dy;

          // Keep the confidence pattern inside the map boundary.
          if (pixelX > MAP_LEFT && pixelX < MAP_RIGHT &&
              pixelY > MAP_TOP && pixelY < MAP_BOTTOM) {
            display.drawPixel(pixelX, pixelY, SSD1306_WHITE);
          }
        }
      }
    }

    // Bright center marks the filtered target position.
    display.fillCircle(px, py, 1, SSD1306_WHITE);
    drawTargetLockMarker(i, px, py);
  }
}

// ==================================================

// DRAW DISTANCE PANEL
// ==================================================
void drawDistancePanel() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(82, 16);
  display.print("TARGETS");

  display.drawLine(82, 25, 126, 25, SSD1306_WHITE);

  int rowY = 29;

for (int i = 0; i < 3; i++) {

    // Draw a small marker beside the selected target's row.
    if (i == lockedTarget) {
      display.fillRect(80, rowY, 2, 7, SSD1306_WHITE);
    }

    display.setCursor(82, rowY);
    display.print(i + 1);
    display.print(":");

    if (filters[i].displayed) {
      float distance = targetDistance(
        (int16_t)roundf(filters[i].smoothX),
        (int16_t)roundf(filters[i].smoothY)
      );

      display.print(distance, 2);
      display.print("m");
    } else {
      display.print("--");
    }

    rowY += 10;
  }
}

// ==================================================

// UPDATE OLED
// ==================================================
void updateOLED() {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(2, 3);
  display.print("LD2450 ");

  if (displayMode == DIRECT_TRACKING) {
    display.print("TRACK");
  } else {
    display.print("CONF");
  }

  if (signalLost) {
  display.setCursor(76, 3);
  display.print("LOST");
} else if (lockedTarget >= 0) {
  display.setCursor(76, 3);
  display.print("LOCK T");
  display.print(lockedTarget + 1);
} else {
  display.setCursor(76, 3);
  display.print("AUTO");
}

  display.drawLine(0, 12, 127, 12, SSD1306_WHITE);

  drawRadarGrid();

  if (displayMode == DIRECT_TRACKING) {
    drawTargets();
  } else {
    drawConfidenceMap();
  }

  drawDistancePanel();
  display.display();
}

// ==================================================

void onDataReceived(const esp_now_recv_info_t *info,
                    const uint8_t *data, int len) {
  if (len != sizeof(RadarPacket)) return;
  RadarPacket packet;
  memcpy(&packet, data, sizeof(packet));

  portENTER_CRITICAL(&packetMux);
  incomingPacket = packet;
  packetPending = true;
  portEXIT_CRITICAL(&packetMux);
}

void applyIncomingPacket(const RadarPacket &packet) {
  lastPacketMillis = millis();
  signalLost = false;
  lastSequence = packet.sequence;
  haveSequence = true;

  for (int i = 0; i < 3; i++) {
    bool active = (packet.displayedMask & (1u << i)) != 0;
    targets[i].x = packet.x[i];
    targets[i].y = packet.y[i];
    targets[i].speed = packet.speed[i];
    targets[i].resolution = packet.resolution[i];
    targets[i].active = active;
    filters[i].smoothX = packet.x[i];
    filters[i].smoothY = packet.y[i];
    filters[i].displayed = active;

    if (active) {
      addPositionSample(i, packet.x[i], packet.y[i]);
    } else {
      clearPositionHistory(i);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED initialization failed!");
    while (true) delay(1000);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Starting wrist...");
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(CHANNEL);
  delay(200);
  Serial.print("Wrist MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    while (true) delay(1000);
  }
  if (esp_now_register_recv_cb(onDataReceived) != ESP_OK) {
    Serial.println("Receiver callback registration failed");
    while (true) delay(1000);
  }

  Serial.println("Wrist receiver ready; ESP-NOW channel 1");
  Serial.println("Button: GPIO 4 to GND; switches TRACK / CONF");
  updateOLED();
}

void loop() {
  handleModeButton();

  RadarPacket packet;
  bool gotPacket = false;
  portENTER_CRITICAL(&packetMux);
  if (packetPending) {
    packet = incomingPacket;
    packetPending = false;
    gotPacket = true;
  }
  portEXIT_CRITICAL(&packetMux);

  if (gotPacket) {
    applyIncomingPacket(packet);
    Serial.printf("Packet %lu received\n", (unsigned long)packet.sequence);
  }

  if (!signalLost && millis() - lastPacketMillis > 1000) {
    signalLost = true;
    for (int i = 0; i < 3; i++) {
      filters[i].displayed = false;
      targets[i].active = false;
      clearPositionHistory(i);
    }
  }

  // Refresh periodically so button changes and SIGNAL LOST appear promptly.
  if (millis() - lastScreenUpdate >= 100) {
    updateOLED();
    lastScreenUpdate = millis();
  }
  delay(5);
}
