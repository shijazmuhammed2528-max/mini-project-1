/*
 * DuskBuddy ESP32-S3 Firmware
 * ============================
 * Connects to WiFi + WebSocket server (port 5000)
 * Receives binary PCM chunks → plays via I2S speaker
 * Receives "END" / "ERROR" control messages from server
 * Receives gesture commands via Serial from server:
 *   FORWARD / BACKWARD / LEFT / RIGHT / STOP / SPIN_CW
 * IR obstacle sensor on GPIO 48:
 *   When obstacle detected → turn left for IR_TURN_MS ms,
 *   then resume the last active movement command.
 * While audioPlaying == true → all motor commands are ignored
 *
 * Wiring:
 *   I2S   BCLK → GPIO 40   LRC → GPIO 41   DOUT → GPIO 42
 *   Motor IN1/IN2/IN3/IN4 → GPIO 1/2/3/4  (L298N)
 *   IR sensor OUT → GPIO 48  (LOW = obstacle detected)
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
#define MOTOR_L_FWD  1  //IN1
#define MOTOR_L_BWD  2  //IN2
#define MOTOR_R_FWD  3  //IN3
#define MOTOR_R_BWD  4  //IN4

// ─── IR obstacle sensor ────────────────────────────────────
#define IR_SENSOR_PIN  48      // IR OUT → GPIO 48
#define IR_ACTIVE_LOW  true    // LOW = obstacle (flip if active-HIGH)
#define IR_TURN_MS     400     // ms to turn left on detection
#define IR_COOLDOWN_MS 600     // min gap between consecutive reactions

// ─── Audio config ──────────────────────────────────────────
#define SAMPLE_RATE   22050
#define DMA_BUF_COUNT 8
#define DMA_BUF_LEN   1024

// ─── State ─────────────────────────────────────────────────
WebSocketsClient ws;
volatile bool audioPlaying = false;
String lastCommand = "STOP";
unsigned long irLastReactTime = 0;

// ═══════════════════════════════════════════════════════════
//  Motor helpers
// ═══════════════════════════════════════════════════════════
void motorsStop() {
    digitalWrite(MOTOR_L_FWD, LOW);  digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, LOW);
}
void motorsForward() {
    digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, HIGH); digitalWrite(MOTOR_R_BWD, LOW);
    Serial.println("motorsForward");
}
void motorsBackward() {
    digitalWrite(MOTOR_L_FWD, LOW);  digitalWrite(MOTOR_L_BWD, HIGH);
    digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, HIGH);
    Serial.println("motorsBackward");
}
void motorsLeft() {
    digitalWrite(MOTOR_L_FWD, LOW);  digitalWrite(MOTOR_L_BWD, HIGH);
    digitalWrite(MOTOR_R_FWD, HIGH); digitalWrite(MOTOR_R_BWD, LOW);
    Serial.println("motorsLeft");
}
void motorsRight() {
    digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, HIGH);
    Serial.println("motorsRight");
}
void motorsSpin() {
    digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
    digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, HIGH);
    Serial.println("motorsSpin");
}

// Apply motors from a command string (no state side-effects)
void applyMotors(const String& cmd) {
    if      (cmd == "FORWARD")  motorsForward();
    else if (cmd == "BACKWARD") motorsBackward();
    else if (cmd == "LEFT")     motorsLeft();
    else if (cmd == "RIGHT")    motorsRight();
    else if (cmd == "SPIN_CW")  motorsSpin();
    else                        motorsStop();
}

// ═══════════════════════════════════════════════════════════
//  Execute gesture command
// ═══════════════════════════════════════════════════════════
void executeCommand(const String& cmd) {
    if (audioPlaying) {
        Serial.printf("[CMD] Blocked (audio playing): %s\n", cmd.c_str());
        return;
    }
    Serial.printf("[CMD] %s\n", cmd.c_str());
    lastCommand = cmd;
    applyMotors(cmd);
}

// ═══════════════════════════════════════════════════════════
//  IR obstacle avoidance (GPIO 48)
//  On obstacle: turn left IR_TURN_MS ms → resume lastCommand
// ═══════════════════════════════════════════════════════════
void handleIR() {
    if (audioPlaying) return;

    unsigned long now = millis();
    if (now - irLastReactTime < IR_COOLDOWN_MS) return;

    bool rawLow   = (digitalRead(IR_SENSOR_PIN) == LOW);
    bool obstacle = IR_ACTIVE_LOW ? rawLow : !rawLow;
    if (!obstacle) return;

    irLastReactTime = now;
    Serial.println("[IR] Obstacle detected → turning left");

    motorsLeft();
    delay(IR_TURN_MS);

    Serial.printf("[IR] Resuming: %s\n", lastCommand.c_str());
    applyMotors(lastCommand);
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
}

// ═══════════════════════════════════════════════════════════
//  WebSocket event handler
// ═══════════════════════════════════════════════════════════
void wsEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
    case WStype_CONNECTED:
        Serial.printf("[WS] Connected → ws://%s:%d%s\n", WS_HOST, WS_PORT, WS_PATH);
        break;
    case WStype_DISCONNECTED:
        Serial.println("[WS] Disconnected — retrying...");
        audioPlaying = false;
        motorsStop();
        lastCommand = "STOP";
        i2s_zero_dma_buffer(I2S_PORT_NUM);
        break;
    case WStype_BIN:
        audioPlaying = true;
        {
            size_t written = 0;
            i2s_write(I2S_PORT_NUM, payload, length, &written, portMAX_DELAY);
        }
        break;
    case WStype_TEXT: {
        String msg = String((char*)payload).substring(0, length);
        if (msg == "END") {
            i2s_zero_dma_buffer(I2S_PORT_NUM);
            audioPlaying = false;
            Serial.println("[WS] Audio playback complete");
        } else if (msg == "ERROR") {
            Serial.println("[WS] TTS error from server");
            audioPlaying = false;
        } else {
            msg.trim(); msg.toUpperCase();
            executeCommand(msg);
        }
        break;
    }
    default: break;
    }
}

// ═══════════════════════════════════════════════════════════
//  Serial handler
// ═══════════════════════════════════════════════════════════
void handleSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim(); line.toUpperCase();
    if (line.length() == 0) return;
    executeCommand(line);
}

// ═══════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(MOTOR_L_FWD, OUTPUT); pinMode(MOTOR_L_BWD, OUTPUT);
    pinMode(MOTOR_R_FWD, OUTPUT); pinMode(MOTOR_R_BWD, OUTPUT);
    motorsStop();

    pinMode(IR_SENSOR_PIN, INPUT);
    Serial.printf("[IR] Sensor ready on GPIO %d (active-%s)\n",
                  IR_SENSOR_PIN, IR_ACTIVE_LOW ? "LOW" : "HIGH");

    setupI2S();

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
    Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

    ws.begin(WS_HOST, WS_PORT, WS_PATH);
    ws.onEvent(wsEvent);
    ws.setReconnectInterval(4000);
    ws.enableHeartbeat(15000, 3000, 2);

    Serial.println("[DuskBuddy] Ready!");
    Serial.println("[DuskBuddy] IR avoidance: obstacle → left turn → resume");
}

// ═══════════════════════════════════════════════════════════
//  Loop
// ═══════════════════════════════════════════════════════════
void loop() {
    ws.loop();
    handleSerial();
    handleIR();
}