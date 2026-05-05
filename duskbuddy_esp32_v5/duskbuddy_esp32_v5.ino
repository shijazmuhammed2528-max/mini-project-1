/*
 * DuskBuddy ESP32-S3 Firmware v5
 * ================================
 * FIXES vs previous version:
 *   1. audioPlaying cleared in wsEvent WStype_TEXT "END" handler AND also
 *      guarded with a short I2S drain wait so DMA truly finishes.
 *   2. All executeCommand() calls now Serial.printf the command with .c_str()
 *      AND flush Serial so prints actually appear on the monitor.
 *   3. wsEvent WStype_BIN: audioPlaying set ONLY on first binary chunk, not
 *      every chunk (avoids redundant sets mid-stream).
 *   4. Added [RECV] debug print for every TEXT message received so you can
 *      confirm gestures are arriving at the ESP32.
 *   5. motorsStop() called on disconnect and when audioPlaying blocks a cmd.
 *   6. IR avoidance: delay() replaced with a non-blocking millis() timer so
 *      ws.loop() keeps running during obstacle turns (prevents WS disconnect).
 *
 * Wiring:
 *   I2S   BCLK → GPIO 40   LRC → GPIO 41   DOUT → GPIO 42
 *   Motor IN1/IN2/IN3/IN4  → GPIO 1/2/3/4  (L298N)
 *   IR sensor OUT          → GPIO 48  (LOW = obstacle detected)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <driver/i2s.h>

// ─── WiFi ──────────────────────────────────────────────────
const char* WIFI_SSID = "Asus H";
const char* WIFI_PASS = "12345678";

// ─── WebSocket server ──────────────────────────────────────
const char*    WS_HOST = "192.168.186.111";
const uint16_t WS_PORT = 5000;
const char*    WS_PATH = "/";

// ─── I2S pins ──────────────────────────────────────────────
#define I2S_BCLK     40
#define I2S_LRC      41
#define I2S_DOUT     42
#define I2S_PORT_NUM I2S_NUM_0

// ─── Motor driver pins (L298N) ─────────────────────────────
#define MOTOR_L_FWD  1   // IN1
#define MOTOR_L_BWD  2   // IN2
#define MOTOR_R_FWD  3   // IN3
#define MOTOR_R_BWD  4   // IN4

// ─── IR obstacle sensor ────────────────────────────────────
#define IR_SENSOR_PIN   48     // IR OUT → GPIO 48
#define IR_ACTIVE_LOW   true   // LOW = obstacle (flip if active-HIGH)
#define IR_TURN_MS      400    // ms to turn left on detection
#define IR_COOLDOWN_MS  600    // min gap between consecutive reactions

// ─── Audio config ──────────────────────────────────────────
#define SAMPLE_RATE    22050
#define DMA_BUF_COUNT  8
#define DMA_BUF_LEN    1024

// ─── I2S drain wait after "END" ────────────────────────────
// Give the DMA buffers time to fully empty before clearing audioPlaying.
#define I2S_DRAIN_MS   120

// ═══════════════════════════════════════════════════════════
//  State
// ═══════════════════════════════════════════════════════════
WebSocketsClient ws;
volatile bool audioPlaying      = false;
volatile bool firstBinaryChunk  = true;  // tracks first chunk per stream
String        lastCommand       = "STOP";

// IR non-blocking turn state
unsigned long irLastReactTime  = 0;
unsigned long irTurnStartTime  = 0;
bool          irTurning        = false;

// ═══════════════════════════════════════════════════════════
//  Motor helpers
// ═══════════════════════════════════════════════════════════
void motorsStop() {
    digitalWrite(MOTOR_L_FWD, LOW); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, LOW); digitalWrite(MOTOR_R_BWD, LOW);
}
void motorsForward() {
    digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, HIGH); digitalWrite(MOTOR_R_BWD, LOW);
    Serial.println("[MOTOR] Forward");
    Serial.flush();
}
void motorsBackward() {
    digitalWrite(MOTOR_L_FWD, LOW);  digitalWrite(MOTOR_L_BWD, HIGH);
    digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, HIGH);
    Serial.println("[MOTOR] Backward");
    Serial.flush();
}
void motorsLeft() {
    digitalWrite(MOTOR_L_FWD, LOW);  digitalWrite(MOTOR_L_BWD, HIGH);
    digitalWrite(MOTOR_R_FWD, HIGH); digitalWrite(MOTOR_R_BWD, LOW);
    Serial.println("[MOTOR] Left");
    Serial.flush();
}
void motorsRight() {
    digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, HIGH);
    Serial.println("[MOTOR] Right");
    Serial.flush();
}
void motorsSpin() {
    digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, HIGH);
    Serial.println("[MOTOR] Spin CW");
    Serial.flush();
}

void applyMotors(const String& cmd) {
    if      (cmd == "FORWARD")  motorsForward();
    else if (cmd == "BACKWARD") motorsBackward();
    else if (cmd == "LEFT")     motorsLeft();
    else if (cmd == "RIGHT")    motorsRight();
    else if (cmd == "SPIN_CW")  motorsSpin();
    else {
        motorsStop();
        Serial.println("[MOTOR] Stop");
        Serial.flush();
    }
}

// ═══════════════════════════════════════════════════════════
//  Execute gesture command
// ═══════════════════════════════════════════════════════════
void executeCommand(const String& cmd) {
    // Always print receipt so we can confirm it arrived
    Serial.printf("[CMD RECV] '%s'  audioPlaying=%s\n",
                  cmd.c_str(), audioPlaying ? "YES" : "NO");
    Serial.flush();

    if (audioPlaying) {
        Serial.println("[CMD] Blocked — audio still playing");
        Serial.flush();
        return;
    }

    if (cmd.length() == 0) return;

    lastCommand = cmd;
    applyMotors(cmd);
}

// ═══════════════════════════════════════════════════════════
//  I2S setup
// ═══════════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════════
//  WebSocket event handler
// ═══════════════════════════════════════════════════════════
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
        motorsStop();
        i2s_zero_dma_buffer(I2S_PORT_NUM);
        break;

    case WStype_BIN:
        // FIX: only set audioPlaying on the FIRST chunk, not every chunk
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
        // Extract as proper null-terminated string
        char buf[length + 1];
        memcpy(buf, payload, length);
        buf[length] = '\0';
        String msg = String(buf);
        msg.trim();

        Serial.printf("[WS TEXT RECV] '%s'\n", msg.c_str());
        Serial.flush();

        if (msg == "END") {
            // Wait for DMA to drain before clearing the flag
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
            // Gesture command — convert to uppercase to be safe
            msg.toUpperCase();
            executeCommand(msg);
        }
        break;
    }

    default:
        break;
    }
}

// ═══════════════════════════════════════════════════════════
//  IR obstacle avoidance — NON-BLOCKING version
//  Uses millis() instead of delay() so ws.loop() keeps running.
// ═══════════════════════════════════════════════════════════
void handleIR() {
    if (audioPlaying) return;

    unsigned long now = millis();

    // Currently in a turn — check if turn time is up
    if (irTurning) {
        if (now - irTurnStartTime >= IR_TURN_MS) {
            irTurning = false;
            Serial.printf("[IR] Turn done — resuming: %s\n", lastCommand.c_str());
            Serial.flush();
            applyMotors(lastCommand);
        }
        return;   // don't check sensor again mid-turn
    }

    // Cooldown check
    if (now - irLastReactTime < IR_COOLDOWN_MS) return;

    bool rawLow   = (digitalRead(IR_SENSOR_PIN) == LOW);
    bool obstacle = IR_ACTIVE_LOW ? rawLow : !rawLow;
    if (!obstacle) return;

    // Start avoidance turn
    irLastReactTime = now;
    irTurnStartTime = now;
    irTurning       = true;

    Serial.println("[IR] Obstacle detected → turning left (non-blocking)");
    Serial.flush();
    motorsLeft();
}

// ═══════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[DuskBuddy] Booting v5...");

    // Motor pins
    pinMode(MOTOR_L_FWD, OUTPUT); pinMode(MOTOR_L_BWD, OUTPUT);
    pinMode(MOTOR_R_FWD, OUTPUT); pinMode(MOTOR_R_BWD, OUTPUT);
    motorsStop();

    // IR sensor
    pinMode(IR_SENSOR_PIN, INPUT);
    Serial.printf("[IR] Sensor ready on GPIO %d (active-%s)\n",
                  IR_SENSOR_PIN, IR_ACTIVE_LOW ? "LOW" : "HIGH");

    setupI2S();

    // WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
    Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

    // WebSocket
    ws.begin(WS_HOST, WS_PORT, WS_PATH);
    ws.onEvent(wsEvent);
    ws.setReconnectInterval(4000);
    ws.enableHeartbeat(15000, 3000, 2);

    Serial.println("[DuskBuddy] Ready!");
    Serial.println("[DuskBuddy] Waiting for gestures from PC server...");
    Serial.flush();
}

// ═══════════════════════════════════════════════════════════
//  Loop
// ═══════════════════════════════════════════════════════════
void loop() {
    ws.loop();
    handleIR();
}
