/* ╔══════════════════════════════════════════════════════════════════════════╗
 * ║         DuskBuddy — Single-Chip Combined Firmware  v1.2                ║
 * ║         ESP32-S3                                                       ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  SYSTEMS ON THIS CHIP:                                                 ║
 * ║    • WebSocket client  → receives gesture commands + TTS audio         ║
 * ║    • I2S audio output  → PCM stream to external DAC/amplifier          ║
 * ║    • L298N motor driver → FORWARD / BACKWARD / LEFT / RIGHT / SPIN     ║
 * ║    • IR obstacle sensor → non-blocking avoidance turn                  ║
 * ║    • Dual SSD1306 OLEDs → RoboEyes, 10 random emotions                 ║
 * ║    • TTP223 touch sensor → tap = CUTE, 3s hold = CRYING CUTE           ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  WIRING SUMMARY                                                        ║
 * ║  ─────────────────────────────────────────────────────────────         ║
 * ║  I2S      BCLK → 40   LRC → 41   DOUT → 42                            ║
 * ║  Motors   IN1→1  IN2→2  IN3→3  IN4→4   (L298N)                        *
 * ║           ENA→11 (PWM left speed)    ENB→12 (PWM right speed)         ║
 * ║           ENA → 11 (PWM speed left)   ENB → 12 (PWM speed right)      ║
 * ║  IR       OUT  → 48   (LOW = obstacle)                                 ║
 * ║  OLED-L   SDA → 8    SCL → 9    (I2C bus 0, addr 0x3C)                ║
 * ║  OLED-R   SDA → 35   SCL → 36   (I2C bus 1, addr 0x3C)                ║
 * ║  Touch    OUT → 17   (TTP223, HIGH = touched)                          ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  REQUIRED LIBRARIES                                                    ║
 * ║    WebSocketsClient  (links2004/arduinoWebSockets)                     ║
 * ║    Adafruit_SSD1306  + Adafruit_GFX                                    ║
 * ║    FluxGarage_RoboEyes                                                 ║
 * ║    ESP32 Arduino core (includes WiFi, driver/i2s)                      ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  INTEGRATION NOTES                                                     ║
 * ║    • Eyes run fully independently of motors/audio — no cross-coupling  ║
 * ║    • Audio playing does NOT change the current eye emotion             ║
 * ║    • IR obstacle detection does NOT change the current eye emotion     ║
 * ║    • Touch sensor has highest priority over automatic emotion cycling  ║
 * ║    • All timing is millis()-based; no blocking delay() in the loop     ║
 * ║  BACKWARD SOFT-RAMP (new in v1.2):                                     ║
 * ║    • BACKWARD accelerates gradually from 0 → MOTOR_SPEED               ║
 * ║    • STOP from BACKWARD decelerates gradually to 0 before cutting pins ║
 * ║    • Any other direction command cancels the ramp instantly            ║
 * ║    • BWD_STEP_MS / BWD_STEP_AMT tune ramp aggressiveness               ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

// ───────────────────────────────────────────────────────────────────────────
//  INCLUDES
// ───────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <driver/i2s.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 1 — CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

// ─── WiFi ──────────────────────────────────────────────────────────────────
const char* WIFI_SSID = "Asus H";
const char* WIFI_PASS = "12345678";

// ─── WebSocket server ───────────────────────────────────────────────────────
const char*    WS_HOST = "192.168.153.111";
const uint16_t WS_PORT = 5000;
const char*    WS_PATH = "/";

// ─── I2S audio pins ─────────────────────────────────────────────────────────
#define I2S_BCLK        40
#define I2S_LRC         41
#define I2S_DOUT        42
#define I2S_PORT_NUM    I2S_NUM_0
#define SAMPLE_RATE     22050
#define DMA_BUF_COUNT   8
#define DMA_BUF_LEN     1024
#define I2S_DRAIN_MS    120   // wait for DMA to empty after stream ends

// ─── Motor driver pins (L298N) ──────────────────────────────────────────────
#define MOTOR_L_FWD  11   // IN1
#define MOTOR_L_BWD  2    // IN2
#define MOTOR_R_FWD  3    // IN3
#define MOTOR_R_BWD  4    // IN4

// ─── L298N enable pins — PWM speed control ──────────────────────────────────
#define MOTOR_ENA    15  // ENA → GPIO 15  (left  side enable)
#define MOTOR_ENB    12  // ENB → GPIO 12  (right side enable)

// PWM config — ESP32 Arduino core v3.x API (ledcAttach / ledcWrite by pin)
#define PWM_FREQ      1000   // 1 kHz carrier — smooth, no audible whine
#define PWM_RES       8      // 8-bit resolution → duty 0–255

// ─── Speed settings (0 = stopped, 255 = full throttle) ──────────────────────
#define MOTOR_SPEED       200   // ← normal drive speed  (~63% of max)
#define MOTOR_TURN_SPEED  180   // ← turn / IR-avoidance (~47% of max)

// ─── IR obstacle sensor ─────────────────────────────────────────────────────
#define IR_SENSOR_PIN   48
#define IR_ACTIVE_LOW   true   // LOW = obstacle; set false if active-HIGH
#define IR_TURN_MS      400    // how long to turn left on detection
#define IR_COOLDOWN_MS  600    // min gap between consecutive reactions

// ─── OLED displays ──────────────────────────────────────────────────────────
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
// Left  eye → I2C bus 0 : SDA=8,  SCL=9
// Right eye → I2C bus 1 : SDA=35, SCL=36

// ─── Touch sensor ───────────────────────────────────────────────────────────
#define TOUCH_PIN          17
#define TOUCH_DEBOUNCE_MS  50
#define LONG_HOLD_MS       3000   // hold ≥ 3s → CRYING CUTE
#define CUTE_DURATION_MS   5000
#define CRY_DURATION_MS    7000

// ─── Backward soft-ramp tuning ───────────────────────────────────────────────
// Increase BWD_STEP_MS to make the ramp slower; decrease to make it snappier.
// Increase BWD_STEP_AMT to jump speed in bigger increments per tick.
// At these values (5 ms × 25 units): 0→200 takes ~40 ms; 200→0 takes ~40 ms.
#define BWD_STEP_MS    5   // milliseconds between each PWM step
#define BWD_STEP_AMT  25   // PWM units changed per step (0–255 scale)


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 2 — MOTOR / AUDIO / WEBSOCKET STATE
// ═══════════════════════════════════════════════════════════════════════════

WebSocketsClient ws;
volatile bool audioPlaying     = false;
volatile bool firstBinaryChunk = true;
String        lastCommand      = "STOP";

// IR non-blocking turn state
unsigned long irLastReactTime = 0;
unsigned long irTurnStartTime = 0;
bool          irTurning       = false;

// ─── Backward soft-ramp state machine ────────────────────────────────────────
// BWD_IDLE      : backward is not active — nothing to do
// BWD_RAMP_UP   : gradually increasing speed after BACKWARD command
// BWD_HOLD      : running at full MOTOR_SPEED in reverse
// BWD_RAMP_DOWN : gradually decreasing speed after STOP while reversing
enum BwdRampState { BWD_IDLE, BWD_RAMP_UP, BWD_HOLD, BWD_RAMP_DOWN };
BwdRampState  bwdRampState = BWD_IDLE;
uint8_t       bwdCurSpeed  = 0;       // current PWM level during ramp
unsigned long lastRampTick = 0;       // timestamp of last ramp step


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 3 — ROBOEYES STATE & DATA
// ═══════════════════════════════════════════════════════════════════════════

TwoWire I2C_Left  = TwoWire(0);
TwoWire I2C_Right = TwoWire(1);

Adafruit_SSD1306 displayLeft (SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_Left,  OLED_RESET);
Adafruit_SSD1306 displayRight(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_Right, OLED_RESET);

RoboEyes<Adafruit_SSD1306> eyeLeft (displayLeft);
RoboEyes<Adafruit_SSD1306> eyeRight(displayRight);

// ─── Emotion struct ─────────────────────────────────────────────────────────
struct Emotion {
    const char*   name;
    byte          mood;
    bool          curiosity;
    int           blinkMin, blinkMax;
    int           blinkCloseDur;
    int           moveMin,  moveMax;
    byte          prefPos;
    unsigned long duration;
};

// ─── 10 normal emotions ─────────────────────────────────────────────────────
#define NUM_EMOTIONS 10
const Emotion emotions[NUM_EMOTIONS] = {
  //  Name        Mood     Curi   bMin  bMax  bCls  mMin   mMax   prefPos   dur
  { "HAPPY",    HAPPY,   true,  2000, 4000,  120,   600,  1800, DEFAULT,  8000 },
  { "ANGRY",    ANGRY,   false,  800, 2000,   90,   150,   700, DEFAULT,  7000 },
  { "TIRED",    TIRED,   false, 5000, 8000,  280,  3000,  6000, S,        9000 },
  { "SLEEPING", TIRED,   false,  600, 1200, 1100,  5000, 10000, S,       12000 },
  { "CURIOUS",  DEFAULT, true,  2000, 4000,  120,   300,  1000, DEFAULT,  8000 },
  { "CONFUSED", DEFAULT, false, 1200, 2800,  100,   100,   500, DEFAULT,  6000 },
  { "EXCITED",  HAPPY,   true,  1000, 2500,   90,   250,   800, DEFAULT,  7000 },
  { "SAD",      TIRED,   false, 4000, 7000,  220,  3500,  7000, S,       10000 },
  { "LAUGHING", HAPPY,   false,  700, 1800,   75,   400,  1000, DEFAULT,  6000 },
  { "DREAMY",   HAPPY,   false, 3000, 5500,  170,  2000,  4500, N,        9000 },
};

// ─── Special emotions ───────────────────────────────────────────────────────
const Emotion cuteEmotion = {
    "CUTE <3", HAPPY, true, 2500, 4000, 220, 1500, 3500, N, CUTE_DURATION_MS
};
const Emotion cryingEmotion = {
    "CRYING CUTE T_T", TIRED, true, 3500, 5500, 260, 2500, 5000, DEFAULT, CRY_DURATION_MS
};

const byte CUTE_POSITIONS[] = { N, NE, N, NW, N, DEFAULT, N };
#define NUM_CUTE_POS 7

const byte CRY_POSITIONS[]  = { DEFAULT, DEFAULT, N, DEFAULT, DEFAULT, NE, DEFAULT, NW };
#define NUM_CRY_POS 8

const byte ALL_POSITIONS[]  = { N, NE, E, SE, S, SW, W, NW, DEFAULT };
#define NUM_POSITIONS 9

// ─── Tear drop system ───────────────────────────────────────────────────────
#define MAX_TEARS       5
#define TEAR_UPDATE_MS  55
#define EYE_BOTTOM_Y    47   // just below cute eye bottom edge (center=32, h=30)

struct TearDrop { float y; float speed; int x; };
TearDrop tears[MAX_TEARS];
unsigned long lastTearUpdate = 0;

// ─── Eye runtime state ──────────────────────────────────────────────────────
int           currentIdx       = -1;
unsigned long emotionStartTime = 0;
unsigned long emotionDuration  = 0;

bool          cuteActive       = false;
bool          cryingActive     = false;
int           specialPosIdx    = 0;

unsigned long lastBlinkTime    = 0;
unsigned long blinkInterval    = 3000;
unsigned long blinkCloseTime   = 0;
unsigned long blinkCloseDur    = 130;
bool          eyesClosed       = false;

unsigned long lastMoveTime     = 0;
unsigned long moveInterval     = 2000;

// ─── Touch state ────────────────────────────────────────────────────────────
bool          touchHeld          = false;
unsigned long touchPressedAt     = 0;
bool          longTouchFired     = false;
bool          lastTouchRaw       = false;
unsigned long touchDebounceStamp = 0;
bool          touchStable        = false;


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 4 — MOTOR FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// Helper — set both enable channels to the same duty (0=stop, 255=full)
void setMotorSpeed(uint8_t speed) {
    ledcWrite(MOTOR_ENA, speed);
    ledcWrite(MOTOR_ENB, speed);
}

void motorsStop() {
    setMotorSpeed(0);
    digitalWrite(MOTOR_L_FWD, LOW); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, LOW); digitalWrite(MOTOR_R_BWD, LOW);
}

void motorsForward() {
    setMotorSpeed(MOTOR_SPEED);
    digitalWrite(MOTOR_L_FWD, LOW);  digitalWrite(MOTOR_L_BWD, HIGH);
    digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, HIGH);
    Serial.println("[MOTOR] Forward"); Serial.flush();
}

void motorsBackward() {
    setMotorSpeed(MOTOR_SPEED);
    digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, HIGH); digitalWrite(MOTOR_R_BWD, LOW);
    Serial.println("[MOTOR] Backward"); Serial.flush();
}

void motorsLeft() {
    setMotorSpeed(MOTOR_TURN_SPEED);
    digitalWrite(MOTOR_L_FWD, LOW);  digitalWrite(MOTOR_L_BWD, HIGH);
    digitalWrite(MOTOR_R_FWD, HIGH); digitalWrite(MOTOR_R_BWD, LOW);
    Serial.println("[MOTOR] Left"); Serial.flush();
}
void motorsRight() {
    setMotorSpeed(MOTOR_TURN_SPEED);
    digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, HIGH);
    Serial.println("[MOTOR] Right"); Serial.flush();
}
void motorsSpin() {
    setMotorSpeed(MOTOR_TURN_SPEED);
    digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, HIGH);
    Serial.println("[MOTOR] Spin CW"); Serial.flush();
}

// ─── Backward soft-ramp: arm the direction pins and begin ramping up ─────────
// Called once when the BACKWARD command arrives.
// Speed starts at 0 (or wherever it currently is if already reversing) and
// climbs toward MOTOR_SPEED in the handleBackwardRamp() tick each loop.
void startBackwardRamp() {
    // Set L298N direction pins for reverse — speed comes from the ramp
    digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, HIGH); digitalWrite(MOTOR_R_BWD, LOW);

    if (bwdRampState == BWD_RAMP_DOWN) {
        // Already slowing from a previous reverse — reverse the ramp seamlessly
        // bwdCurSpeed stays where it is; we just flip direction
        Serial.println("[MOTOR] Backward: reversing ramp — continuing from current speed");
    } else {
        bwdCurSpeed = 0;
        setMotorSpeed(0);
        Serial.println("[MOTOR] Backward: ramp UP start  0 → MOTOR_SPEED");
    }
    bwdRampState = BWD_RAMP_UP;
    lastRampTick = millis();
}

// ─── Backward soft-ramp: begin gradual deceleration to full stop ─────────────
// Called when STOP is issued while the backward ramp is active.
void startBackwardStop() {
    bwdRampState = BWD_RAMP_DOWN;
    lastRampTick = millis();
    Serial.println("[MOTOR] Backward: ramp DOWN start  MOTOR_SPEED → 0");
}

// ─── Backward soft-ramp: tick — called every loop iteration ─────────────────
// Steps the PWM duty one notch every BWD_STEP_MS milliseconds.
// Does nothing when bwdRampState == BWD_IDLE.
void handleBackwardRamp() {
    if (bwdRampState == BWD_IDLE || bwdRampState == BWD_HOLD) return;

    unsigned long now = millis();
    if (now - lastRampTick < BWD_STEP_MS) return;
    lastRampTick = now;

    if (bwdRampState == BWD_RAMP_UP) {
        // ── Accelerating ────────────────────────────────────────────────────
        int next = (int)bwdCurSpeed + BWD_STEP_AMT;
        if (next >= MOTOR_SPEED) {
            bwdCurSpeed  = MOTOR_SPEED;
            bwdRampState = BWD_HOLD;
            Serial.println("[MOTOR] Backward: ramp UP complete — full speed");
        } else {
            bwdCurSpeed = (uint8_t)next;
        }
        setMotorSpeed(bwdCurSpeed);

    } else if (bwdRampState == BWD_RAMP_DOWN) {
        // ── Decelerating ────────────────────────────────────────────────────
        if (bwdCurSpeed <= BWD_STEP_AMT) {
            // Speed has reached (or is about to cross) zero — full stop
            bwdCurSpeed  = 0;
            bwdRampState = BWD_IDLE;
            motorsStop();   // cut PWM and direction pins cleanly
            Serial.println("[MOTOR] Backward: ramp DOWN complete — stopped");
        } else {
            bwdCurSpeed -= BWD_STEP_AMT;
            setMotorSpeed(bwdCurSpeed);
        }
    }
}

// ─── Route a command to the correct motor function ───────────────────────────
void applyMotors(const String& cmd) {
    if (cmd == "BACKWARD") {
        // ── Special path: soft-start ramp ───────────────────────────────────
        // If already at full reverse, ignore duplicates.
        if (bwdRampState == BWD_HOLD) return;
        startBackwardRamp();

    } else {
        // ── Any other command: cancel any active backward ramp first ─────────
        if (bwdRampState != BWD_IDLE) {
            if (cmd == "STOP" || cmd.length() == 0) {
                // Soft-stop — only if the motors are actually moving
                if (bwdRampState == BWD_RAMP_UP || bwdRampState == BWD_HOLD) {
                    startBackwardStop();
                    return;   // let the ramp tick finish the job
                }
                // Already ramping down — nothing to do, let it finish
                return;
            } else {
                // Hard switch to a new direction — abort ramp immediately
                bwdRampState = BWD_IDLE;
                bwdCurSpeed  = 0;
                Serial.println("[MOTOR] Backward ramp cancelled — new direction");
            }
        }

        // ── Normal (instant) direction commands ──────────────────────────────
        if      (cmd == "FORWARD")  motorsForward();
        else if (cmd == "LEFT")     motorsLeft();
        else if (cmd == "RIGHT")    motorsRight();
        else if (cmd == "SPIN_CW")  motorsSpin();
        else { motorsStop(); Serial.println("[MOTOR] Stop"); Serial.flush(); }
    }
}

void executeCommand(const String& cmd) {
    Serial.printf("[CMD RECV] '%s'  audioPlaying=%s\n",
                  cmd.c_str(), audioPlaying ? "YES" : "NO");
    Serial.flush();
    if (audioPlaying) {
        Serial.println("[CMD] Blocked — audio still playing"); Serial.flush();
        return;
    }
    if (cmd.length() == 0) return;
    lastCommand = cmd;
    applyMotors(cmd);
}


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 5 — I2S AUDIO
// ═══════════════════════════════════════════════════════════════════════════

void setupI2S() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = true,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = 0,
    };
    i2s_pin_config_t pins = {
        .mck_io_num   = I2S_PIN_NO_CHANGE,
        .bck_io_num   = I2S_BCLK,
        .ws_io_num    = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num  = I2S_PIN_NO_CHANGE,
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT_NUM, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT_NUM, &pins));
    i2s_zero_dma_buffer(I2S_PORT_NUM);
    Serial.printf("[I2S] Ready — %d Hz, 16-bit, mono\n", SAMPLE_RATE);
    Serial.flush();
}


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 6 — WEBSOCKET
// ═══════════════════════════════════════════════════════════════════════════

void wsEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {

    case WStype_CONNECTED:
        Serial.printf("[WS] Connected → ws://%s:%d%s\n", WS_HOST, WS_PORT, WS_PATH);
        Serial.flush();
        firstBinaryChunk = true;
        audioPlaying     = false;
        break;

    case WStype_DISCONNECTED:
        Serial.println("[WS] Disconnected — retrying...");
        Serial.flush();
        audioPlaying     = false;
        firstBinaryChunk = true;
        lastCommand      = "STOP";
        bwdRampState     = BWD_IDLE;   // cancel any ramp on disconnect
        bwdCurSpeed      = 0;
        motorsStop();
        i2s_zero_dma_buffer(I2S_PORT_NUM);
        break;

    case WStype_BIN:
        if (firstBinaryChunk) {
            audioPlaying     = true;
            firstBinaryChunk = false;
            Serial.println("[AUDIO] Stream started — motors locked");
            Serial.flush();
        }
        {
            size_t written = 0;
            i2s_write(I2S_PORT_NUM, payload, length, &written, portMAX_DELAY);
        }
        break;

    case WStype_TEXT: {
        char buf[length + 1];
        memcpy(buf, payload, length);
        buf[length] = '\0';
        String msg = String(buf);
        msg.trim();

        Serial.printf("[WS TEXT RECV] '%s'\n", msg.c_str());
        Serial.flush();

        if (msg == "END") {
            delay(I2S_DRAIN_MS);
            i2s_zero_dma_buffer(I2S_PORT_NUM);
            audioPlaying     = false;
            firstBinaryChunk = true;
            Serial.println("[AUDIO] Playback complete — motors unlocked");
            Serial.flush();

        } else if (msg == "ERROR") {
            Serial.println("[WS] TTS error from server");
            Serial.flush();
            audioPlaying     = false;
            firstBinaryChunk = true;
            i2s_zero_dma_buffer(I2S_PORT_NUM);

        } else {
            msg.toUpperCase();
            executeCommand(msg);
        }
        break;
    }

    default: break;
    }
}


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 7 — IR OBSTACLE AVOIDANCE  (non-blocking)
// ═══════════════════════════════════════════════════════════════════════════

void handleIR() {
    if (audioPlaying) return;

    unsigned long now = millis();

    if (irTurning) {
        if (now - irTurnStartTime >= IR_TURN_MS) {
            irTurning = false;
            Serial.printf("[IR] Turn done — resuming: %s\n", lastCommand.c_str());
            Serial.flush();
            applyMotors(lastCommand);
        }
        return;
    }

    if (now - irLastReactTime < IR_COOLDOWN_MS) return;

    bool rawLow   = (digitalRead(IR_SENSOR_PIN) == LOW);
    bool obstacle = IR_ACTIVE_LOW ? rawLow : !rawLow;
    if (!obstacle) return;

    irLastReactTime = now;
    irTurnStartTime = now;
    irTurning       = true;

    Serial.println("[IR] Obstacle detected → turning left (non-blocking)");
    Serial.flush();
    motorsLeft();
}


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 8 — ROBOEYES HELPERS
// ═══════════════════════════════════════════════════════════════════════════

void bothMood (byte m) { eyeLeft.setMood(m);               eyeRight.setMood(m);              }
void bothCuri (bool c) { eyeLeft.setCuriosity(c ? ON:OFF); eyeRight.setCuriosity(c ? ON:OFF);}
void bothPos  (byte p) { eyeLeft.setPosition(p);           eyeRight.setPosition(p);          }
void bothClose()       { eyeLeft.close();                  eyeRight.close();                 }
void bothOpen ()       { eyeLeft.open();                   eyeRight.open();                  }

void killFlickers() {
    eyeLeft.setHFlicker(OFF, 0);  eyeRight.setHFlicker(OFF, 0);
    eyeLeft.setVFlicker(OFF, 0);  eyeRight.setVFlicker(OFF, 0);
}

void setEyeShape(byte w, byte h, byte r) {
    eyeLeft.setWidth(w, w);          eyeRight.setWidth(w, w);
    eyeLeft.setHeight(h, h);         eyeRight.setHeight(h, h);
    eyeLeft.setBorderradius(r, r);   eyeRight.setBorderradius(r, r);
}

const Emotion& activeEmotion() {
    if (cuteActive)   return cuteEmotion;
    if (cryingActive) return cryingEmotion;
    return emotions[currentIdx];
}

void printEmotion(const char* name, unsigned long dur) {
    Serial.println(F("╔══════════════════════════════╗"));
    Serial.print  (F("║  EMOTION : ")); Serial.print(name);
    int pad = 18 - strlen(name);
    for (int i = 0; i < pad; i++) Serial.print(' ');
    Serial.println(F("║"));
    Serial.print  (F("║  Duration: ")); Serial.print(dur / 1000);
    Serial.println(F("s                 ║"));
    Serial.println(F("╚══════════════════════════════╝"));
}

void seedTiming(const Emotion& e, unsigned long dur) {
    blinkInterval    = random(e.blinkMin, e.blinkMax);
    blinkCloseDur    = e.blinkCloseDur;
    moveInterval     = random(e.moveMin,  e.moveMax);
    emotionDuration  = dur;
    emotionStartTime = millis();
    lastBlinkTime    = millis();
    lastMoveTime     = millis();
    if (eyesClosed) { bothOpen(); eyesClosed = false; }
}

byte nextPosition() {
    if (cuteActive) {
        specialPosIdx = (specialPosIdx + 1) % NUM_CUTE_POS;
        return CUTE_POSITIONS[specialPosIdx];
    }
    if (cryingActive) {
        specialPosIdx = (specialPosIdx + 1) % NUM_CRY_POS;
        return CRY_POSITIONS[specialPosIdx];
    }
    const Emotion& e = emotions[currentIdx];
    if (e.prefPos != DEFAULT && random(100) < 70) return e.prefPos;
    return ALL_POSITIONS[random(NUM_POSITIONS)];
}


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 9 — EMOTION APPLIERS
// ═══════════════════════════════════════════════════════════════════════════

void applyEmotion(int idx) {
    setEyeShape(36, 36, 8);
    bothMood(emotions[idx].mood);
    bothCuri(emotions[idx].curiosity);
    killFlickers();
    currentIdx   = idx;
    cuteActive   = false;
    cryingActive = false;
    seedTiming(emotions[idx], emotions[idx].duration + random(-800, 800));
    printEmotion(emotions[idx].name, emotionDuration);
}

void applyCuteEmotion() {
    setEyeShape(30, 30, 15);
    bothMood(HAPPY);
    bothCuri(true);
    killFlickers();
    cuteActive    = true;
    cryingActive  = false;
    specialPosIdx = 0;
    bothPos(CUTE_POSITIONS[0]);
    seedTiming(cuteEmotion, CUTE_DURATION_MS);
    Serial.println(F("╔══════════════════════════════╗"));
    Serial.println(F("║  ** TAP **  EMOTION: CUTE <3 ║"));
    Serial.println(F("╚══════════════════════════════╝"));
}

void applyCryingCuteEmotion() {
    setEyeShape(30, 30, 15);
    bothMood(TIRED);
    bothCuri(true);
    killFlickers();
    cryingActive  = true;
    cuteActive    = false;
    specialPosIdx = 0;
    bothPos(DEFAULT);
    seedTiming(cryingEmotion, CRY_DURATION_MS);

    float spacing = (float)(SCREEN_HEIGHT + 10 - EYE_BOTTOM_Y) / MAX_TEARS;
    for (int i = 0; i < MAX_TEARS; i++) {
        tears[i].y     = EYE_BOTTOM_Y + i * spacing;
        tears[i].x     = 62 + random(-3, 4);
        tears[i].speed = 1.2f + i * 0.25f;
    }
    lastTearUpdate = millis();

    Serial.println(F("╔══════════════════════════════╗"));
    Serial.println(F("║  ** 3s HOLD ** CRYING CUTE   ║"));
    Serial.println(F("╚══════════════════════════════╝"));
}


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 10 — TEAR DROP RENDERING
// ═══════════════════════════════════════════════════════════════════════════

void drawOneTear(Adafruit_SSD1306& disp, int x, int y) {
    if (y < EYE_BOTTOM_Y - 5 || y > SCREEN_HEIGHT + 5) return;
    disp.fillCircle(x, y + 2, 2, WHITE);
    disp.fillTriangle(x - 2, y + 1, x + 2, y + 1, x, y - 3, WHITE);
}

void stepAndRenderTears() {
    for (int i = 0; i < MAX_TEARS; i++) {
        tears[i].y += tears[i].speed;
        if (tears[i].y > SCREEN_HEIGHT + 6) {
            tears[i].y = EYE_BOTTOM_Y;
            tears[i].x = 62 + random(-3, 4);
        }
    }
    for (int i = 0; i < MAX_TEARS; i++) {
        drawOneTear(displayLeft,  tears[i].x, (int)tears[i].y);
        drawOneTear(displayRight, tears[i].x, (int)tears[i].y);
    }
    displayLeft .display();
    displayRight.display();
}


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 11 — TOUCH HANDLER
// ═══════════════════════════════════════════════════════════════════════════

void handleTouch() {
    unsigned long now = millis();
    bool raw = (digitalRead(TOUCH_PIN) == HIGH);

    if (raw != lastTouchRaw) touchDebounceStamp = now;
    lastTouchRaw = raw;
    if (now - touchDebounceStamp < TOUCH_DEBOUNCE_MS) return;
    touchStable = raw;

    if (touchStable && !touchHeld) {
        touchHeld      = true;
        touchPressedAt = now;
        longTouchFired = false;
    }

    if (touchHeld && touchStable && !longTouchFired) {
        if (now - touchPressedAt >= LONG_HOLD_MS) {
            longTouchFired = true;
            applyCryingCuteEmotion();
        }
    }

    if (!touchStable && touchHeld) {
        touchHeld = false;
        if (!longTouchFired) applyCuteEmotion();
    }
}


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 12 — SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("\n╔══════════════════════════════════════════╗"));
    Serial.println(F(  "║   DuskBuddy Combined Firmware  v1.2      ║"));
    Serial.println(F(  "║   Motors + Audio + Eyes + Touch + IR     ║"));
    Serial.println(F(  "╚══════════════════════════════════════════╝"));

    // ── Motor direction pins ───────────────────────────────────────────
    pinMode(MOTOR_L_FWD, OUTPUT); pinMode(MOTOR_L_BWD, OUTPUT);
    pinMode(MOTOR_R_FWD, OUTPUT); pinMode(MOTOR_R_BWD, OUTPUT);

    // ── Motor speed — PWM on ENA and ENB ──────────────────────────────
    ledcAttach(MOTOR_ENA, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR_ENB, PWM_FREQ, PWM_RES);
    setMotorSpeed(0);

    motorsStop();
    Serial.printf("[MOTOR] Ready  speed=%d/255  turn=%d/255\n",
                  MOTOR_SPEED, MOTOR_TURN_SPEED);

    // ── IR sensor ─────────────────────────────────────────────────────
    pinMode(IR_SENSOR_PIN, INPUT);
    Serial.printf("[IR] Sensor ready on GPIO %d (active-%s)\n",
                  IR_SENSOR_PIN, IR_ACTIVE_LOW ? "LOW" : "HIGH");

    // ── I2S audio ─────────────────────────────────────────────────────
    setupI2S();

    // ── I2C buses & OLEDs ─────────────────────────────────────────────
    I2C_Left .begin(8,  9);
    I2C_Right.begin(35, 36);

    if (!displayLeft.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("[OLED] ERROR: Left display not found!")); for (;;);
    }
    if (!displayRight.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("[OLED] ERROR: Right display not found!")); for (;;);
    }
    Serial.println(F("[OLED] Both displays ready"));

    // ── RoboEyes init ─────────────────────────────────────────────────
    eyeLeft .begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100);
    eyeRight.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100);

    eyeLeft.setCyclops(ON);           eyeRight.setCyclops(ON);
    eyeLeft.setAutoblinker(OFF,0,0);  eyeRight.setAutoblinker(OFF,0,0);
    eyeLeft.setIdleMode(OFF,0,0);     eyeRight.setIdleMode(OFF,0,0);
    Serial.println(F("[EYES] RoboEyes ready"));

    // ── Touch sensor ──────────────────────────────────────────────────
    pinMode(TOUCH_PIN, INPUT);
    Serial.printf("[TOUCH] Sensor ready on GPIO %d  (tap=CUTE, 3s=CRY)\n", TOUCH_PIN);

    // ── WiFi ──────────────────────────────────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
    Serial.printf("\n[WiFi] Connected — IP: %s\n", WiFi.localIP().toString().c_str());

    // ── WebSocket ─────────────────────────────────────────────────────
    ws.begin(WS_HOST, WS_PORT, WS_PATH);
    ws.onEvent(wsEvent);
    ws.setReconnectInterval(4000);
    ws.enableHeartbeat(15000, 3000, 2);
    Serial.printf("[WS] Client configured → ws://%s:%d%s\n", WS_HOST, WS_PORT, WS_PATH);

    // ── First emotion ─────────────────────────────────────────────────
    randomSeed(analogRead(A0));
    applyEmotion(0);

    Serial.println(F("\n[DuskBuddy] All systems GO — entering main loop"));
    Serial.flush();
}


// ═══════════════════════════════════════════════════════════════════════════
//  ██████╗  SECTION 13 — MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    unsigned long now = millis();

    // ── 1. WebSocket (highest priority — keep TCP alive) ──────────────
    ws.loop();

    // ── 2. IR obstacle avoidance (non-blocking) ───────────────────────
    handleIR();

    // ── 3. Backward soft-ramp tick ────────────────────────────────────
    //    Steps the PWM level one notch toward target every BWD_STEP_MS ms.
    //    No-ops when bwdRampState == BWD_IDLE or BWD_HOLD.
    handleBackwardRamp();

    // ── 4. Touch sensor ───────────────────────────────────────────────
    handleTouch();

    // ── 5. Emotion auto-cycle ─────────────────────────────────────────
    if (!cuteActive && !cryingActive) {
        if (now - emotionStartTime >= emotionDuration) {
            int next;
            do { next = random(NUM_EMOTIONS); } while (next == currentIdx);
            applyEmotion(next);
        }
    } else {
        if (now - emotionStartTime >= emotionDuration) {
            cuteActive   = false;
            cryingActive = false;
            int next;
            do { next = random(NUM_EMOTIONS); } while (next == currentIdx);
            applyEmotion(next);
        }
    }

    // ── 6. Blink state machine ────────────────────────────────────────
    if (!eyesClosed) {
        if (now - lastBlinkTime >= blinkInterval) {
            bothClose();
            eyesClosed     = true;
            blinkCloseTime = now;
        }
    } else {
        if (now - blinkCloseTime >= blinkCloseDur) {
            bothOpen();
            eyesClosed    = false;
            lastBlinkTime = now;
            blinkInterval = random(activeEmotion().blinkMin, activeEmotion().blinkMax);
        }
    }

    // ── 7. Eye movement state machine ─────────────────────────────────
    if (now - lastMoveTime >= moveInterval) {
        bothPos(nextPosition());
        lastMoveTime = now;
        moveInterval = random(activeEmotion().moveMin, activeEmotion().moveMax);
    }

    // ── 8. Render eyes ────────────────────────────────────────────────
    eyeLeft .update();
    eyeRight.update();

    // ── 9. Tear drop overlay (CRYING CUTE only) ───────────────────────
    if (cryingActive && now - lastTearUpdate >= TEAR_UPDATE_MS) {
        stepAndRenderTears();
        lastTearUpdate = now;
    }
}
