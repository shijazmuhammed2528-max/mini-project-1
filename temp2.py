"""
DuskBuddy Server v4
====================
FIXES vs v3:
  1. audio_playing now waits for ACTUAL playback duration (pcm_bytes / bytes_per_sec)
     before clearing — not a flat 0.3 s. Motors unblock immediately after speech ends.
  2. Boot greeting changed to single word "Ready" (~0.5 s) so motors aren't
     blocked for 4+ seconds right after connect.
  3. Gesture confirmation: gesture must be stable for GESTURE_CONFIRM consecutive
     frames before sending — removes false triggers from brief hand blips.
  4. Gesture resend: same command re-sends every GESTURE_RESEND seconds while held.

Architecture:
  PC ──[Binary PCM]──► ESP32 → I2S speaker
  PC ──[Text "END"]──► ESP32 → clear audioPlaying flag
  PC ──[Text "FORWARD"/"BACKWARD"/"LEFT"/"RIGHT"/"STOP"/"SPIN_CW"]
                     ──► ESP32 → motor driver (L298N)

Install:
    pip install websockets edge-tts pydub opencv-python mediapipe
    pip install SpeechRecognition pyserial groq
    ffmpeg must be on PATH
"""

import asyncio
import threading
import logging
import io
import time
import cv2
import mediapipe as mp
import speech_recognition as sr
import serial
import serial.tools.list_ports
import websockets
import edge_tts
from pydub import AudioSegment
from groq import Groq

# ═══════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════════════════
WS_HOST      = "0.0.0.0"
WS_PORT      = 5000
VOICE        = "en-IN-PrabhatNeural"
SAMPLE_RATE  = 22050
CHANNELS     = 1
SAMPLE_WIDTH = 2          # bytes per sample (16-bit PCM)
CHUNK_BYTES  = 4096

GROQ_API_KEY = ""
GROQ_MODEL   = "llama-3.3-70b-versatile"

SERIAL_PORT  = None       # None = auto-detect; or "COM3" / "/dev/ttyUSB0"
SERIAL_BAUD  = 115200

GESTURE_CONFIRM = 3       # frames gesture must be stable before sending (~0.1 s at 30 fps)
GESTURE_RESEND  = 1.0     # re-send same command every N seconds while held
AUDIO_TAIL_S    = 0.5     # extra seconds after calculated duration before unblocking

# ═══════════════════════════════════════════════════════════════════════════
#  LOGGING
# ═══════════════════════════════════════════════════════════════════════════
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════════════════
#  SHARED STATE
# ═══════════════════════════════════════════════════════════════════════════
audio_playing = threading.Event()
_ws_clients: set = set()
_ws_lock = threading.Lock()
_loop: asyncio.AbstractEventLoop | None = None

# ═══════════════════════════════════════════════════════════════════════════
#  SERIAL (read-only monitor)
# ═══════════════════════════════════════════════════════════════════════════
_serial: serial.Serial | None = None

def init_serial() -> bool:
    global _serial, SERIAL_PORT
    if SERIAL_PORT is None:
        for port in serial.tools.list_ports.comports():
            desc = (port.description or "").lower()
            if any(k in desc for k in ("cp210", "ch340", "ch341", "uart", "esp")):
                SERIAL_PORT = port.device
                break
        if SERIAL_PORT is None:
            ports = serial.tools.list_ports.comports()
            if ports:
                SERIAL_PORT = ports[0].device
    if SERIAL_PORT is None:
        log.info("No serial port — ESP32 debug output not shown")
        return False
    try:
        _serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        log.info(f"Serial monitor: {SERIAL_PORT} @ {SERIAL_BAUD}")
        return True
    except Exception as exc:
        log.warning(f"Serial open failed ({SERIAL_PORT}): {exc}")
        return False

def serial_reader_loop():
    log.info("[Serial] Reader started")
    while True:
        try:
            if _serial and _serial.is_open and _serial.in_waiting:
                line = _serial.readline().decode("utf-8", errors="replace").strip()
                if line:
                    print(f"  [ESP32] {line}")
            else:
                time.sleep(0.02)
        except Exception as exc:
            log.warning(f"Serial read error: {exc}")
            time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════════════
#  WEBSOCKET BROADCAST
# ═══════════════════════════════════════════════════════════════════════════
async def _ws_broadcast_text(msg: str):
    with _ws_lock:
        clients = set(_ws_clients)
    if not clients:
        log.warning(f"[WS] No clients — '{msg}' dropped")
        return
    dead: set = set()
    for client in clients:
        try:
            await client.send(msg)
        except Exception:
            dead.add(client)
    if dead:
        with _ws_lock:
            _ws_clients -= dead

def send_gesture(cmd: str):
    """Send motor command to ESP32 via WebSocket TEXT. Blocked while audio plays."""
    if audio_playing.is_set():
        return
    print(f"\n{'='*46}")
    print(f"  [WS → ESP32]  GESTURE: {cmd}")
    print(f"{'='*46}")
    log.info(f"[GESTURE] {cmd}")
    if _loop:
        asyncio.run_coroutine_threadsafe(_ws_broadcast_text(cmd), _loop)

# ═══════════════════════════════════════════════════════════════════════════
#  TTS → PCM
# ═══════════════════════════════════════════════════════════════════════════
async def text_to_pcm(text: str) -> bytes:
    communicate = edge_tts.Communicate(text, VOICE)
    mp3_buf = io.BytesIO()
    async for chunk in communicate.stream():
        if chunk["type"] == "audio":
            mp3_buf.write(chunk["data"])
    mp3_buf.seek(0)
    audio = (
        AudioSegment.from_mp3(mp3_buf)
        .set_frame_rate(SAMPLE_RATE)
        .set_channels(CHANNELS)
        .set_sample_width(SAMPLE_WIDTH)
        .apply_gain(3)
    )
    log.info(f"PCM ready — {len(audio.raw_data):,} bytes ({audio.duration_seconds:.2f}s)")
    return audio.raw_data

# ═══════════════════════════════════════════════════════════════════════════
#  STREAM PCM → ESP32
#  KEY FIX: wait for actual playback duration, not a flat 0.3 s
# ═══════════════════════════════════════════════════════════════════════════
async def stream_audio_to_clients(pcm: bytes):
    with _ws_lock:
        clients = set(_ws_clients)
    if not clients:
        log.warning("No ESP32 clients — audio dropped")
        return

    bytes_per_second = SAMPLE_RATE * CHANNELS * SAMPLE_WIDTH  # 44100 B/s
    play_duration_s  = len(pcm) / bytes_per_second
    log.info(f"Blocking motors for {play_duration_s + AUDIO_TAIL_S:.2f}s (audio duration + tail)")

    audio_playing.set()
    try:
        for i in range(0, len(pcm), CHUNK_BYTES):
            chunk = pcm[i: i + CHUNK_BYTES]
            dead: set = set()
            for client in clients:
                try:
                    await client.send(chunk)
                except Exception:
                    dead.add(client)
            clients -= dead
            await asyncio.sleep(0)

        for client in clients:
            try:
                await client.send("END")
            except Exception:
                pass

        log.info(f"All bytes sent — waiting {play_duration_s:.2f}s for ESP32 to finish playing")
        # Wait for the ESP32's I2S DMA to actually finish playing
        await asyncio.sleep(play_duration_s + AUDIO_TAIL_S)

    finally:
        audio_playing.clear()
        log.info("Motors unblocked — gestures active")

# ═══════════════════════════════════════════════════════════════════════════
#  TTS + STREAM (thread-safe)
# ═══════════════════════════════════════════════════════════════════════════
async def _tts_and_stream(text: str):
    try:
        pcm = await text_to_pcm(text)
        await stream_audio_to_clients(pcm)
    except Exception as exc:
        log.error(f"TTS/stream error: {exc}")
        audio_playing.clear()

def tts_and_stream_sync(text: str):
    if _loop:
        future = asyncio.run_coroutine_threadsafe(_tts_and_stream(text), _loop)
        future.result(timeout=60)

# ═══════════════════════════════════════════════════════════════════════════
#  GROQ LLM
# ═══════════════════════════════════════════════════════════════════════════
_groq = Groq(api_key=GROQ_API_KEY)

def ask_llm(prompt: str) -> str:
    try:
        resp = _groq.chat.completions.create(
            model=GROQ_MODEL,
            messages=[
                {"role": "system", "content": (
                    "You are DuskBuddy, a friendly robot assistant. "
                    "Reply in one or two short sentences only."
                )},
                {"role": "user", "content": prompt},
            ],
        )
        return resp.choices[0].message.content.strip()
    except Exception as exc:
        log.error(f"LLM error: {exc}")
        return "Sorry, I could not process that."

# ═══════════════════════════════════════════════════════════════════════════
#  SPEECH LOOP
# ═══════════════════════════════════════════════════════════════════════════
def speech_loop():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    last_text = ""
    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=1)
    log.info("🎤 Microphone ready — listening...")
    while True:
        try:
            with mic as source:
                audio = recognizer.listen(source, phrase_time_limit=5)
            text = recognizer.recognize_google(audio).strip()
            if not text or text == last_text:
                continue
            last_text = text
            log.info(f'[STT] "{text}"')
            reply = ask_llm(text)
            log.info(f'[LLM] "{reply}"')
            tts_and_stream_sync(reply)
        except sr.UnknownValueError:
            pass
        except sr.RequestError as exc:
            log.warning(f"STT error: {exc}")
        except Exception as exc:
            log.error(f"Speech loop error: {exc}")

# ═══════════════════════════════════════════════════════════════════════════
#  WEBSOCKET SERVER
# ═══════════════════════════════════════════════════════════════════════════
async def ws_handler(websocket):
    addr = websocket.remote_address
    log.info(f"[WS] ESP32 connected from {addr}")
    with _ws_lock:
        _ws_clients.add(websocket)
    try:
        # Short greeting — "Ready" is ~0.5 s, not 4 s
        boot_pcm = await text_to_pcm("Ready")
        await stream_audio_to_clients(boot_pcm)

        async for message in websocket:
            if isinstance(message, str):
                log.debug(f"[WS] ESP32: {message}")
    except websockets.exceptions.ConnectionClosedOK:
        log.info(f"[WS] Disconnected (clean): {addr}")
    except websockets.exceptions.ConnectionClosedError as exc:
        log.warning(f"[WS] Connection error {addr}: {exc}")
    except Exception as exc:
        log.error(f"[WS] Error: {exc}", exc_info=True)
    finally:
        with _ws_lock:
            _ws_clients.discard(websocket)

# ═══════════════════════════════════════════════════════════════════════════
#  GESTURE DETECTION
# ═══════════════════════════════════════════════════════════════════════════
THUMB_TIP  = 4;  THUMB_IP   = 3
INDEX_TIP  = 8;  INDEX_PIP  = 6
MIDDLE_TIP = 12; MIDDLE_PIP = 10
RING_TIP   = 16; RING_PIP   = 14
PINKY_TIP  = 20; PINKY_PIP  = 18

def finger_states(lm) -> dict:
    pts = lm.landmark
    return dict(
        thumb  = pts[THUMB_TIP].x  < pts[THUMB_IP].x,
        index  = pts[INDEX_TIP].y  < pts[INDEX_PIP].y,
        middle = pts[MIDDLE_TIP].y < pts[MIDDLE_PIP].y,
        ring   = pts[RING_TIP].y   < pts[RING_PIP].y,
        pinky  = pts[PINKY_TIP].y  < pts[PINKY_PIP].y,
    )

def classify_gesture(hand_results) -> str:
    hands = hand_results.multi_hand_landmarks
    if not hands:
        return ""
    if len(hands) == 2:
        return "SPIN_CW"
    f = finger_states(hands[0])
    i, m, r, p = f["index"], f["middle"], f["ring"], f["pinky"]
    if     i and not m and not r and not p:  return "FORWARD"
    if     i and     m and not r and not p:  return "BACKWARD"
    if     i and     m and     r and not p:  return "LEFT"
    if     i and     m and     r and     p:  return "RIGHT"
    if not i and not m and not r and not p:  return "STOP"
    return ""

def gesture_loop():
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        min_detection_confidence=0.7,
        min_tracking_confidence=0.7,
        max_num_hands=2,
    )
    mp_draw = mp.solutions.drawing_utils
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        log.error("Cannot open camera — gesture control disabled")
        return
    log.info("📷 Camera opened — gesture control active")

    legend = [
        "1 finger  (index)            = FORWARD",
        "2 fingers (index+middle)     = BACKWARD",
        "3 fingers (+ring)            = LEFT",
        "4 fingers (+pinky)           = RIGHT",
        "Fist                         = STOP",
        "Both hands                   = SPIN CW",
    ]

    confirm_buf: list[str] = []
    confirmed_cmd = ""
    last_sent_time = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame = cv2.flip(frame, 1)
        rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res   = hands.process(rgb)

        if res.multi_hand_landmarks:
            for lm in res.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, lm, mp_hands.HAND_CONNECTIONS)

        raw_cmd = classify_gesture(res)

        # Confirmation buffer — must be stable for GESTURE_CONFIRM frames
        confirm_buf.append(raw_cmd)
        if len(confirm_buf) > GESTURE_CONFIRM:
            confirm_buf.pop(0)
        stable_cmd = (
            confirm_buf[0]
            if len(confirm_buf) == GESTURE_CONFIRM and len(set(confirm_buf)) == 1
            else ""
        )

        now = time.time()
        if stable_cmd:
            is_new    = (stable_cmd != confirmed_cmd)
            is_resend = (now - last_sent_time) >= GESTURE_RESEND
            if is_new or is_resend:
                send_gesture(stable_cmd)
                confirmed_cmd  = stable_cmd
                last_sent_time = now
        else:
            confirmed_cmd = ""

        # Status overlay
        if audio_playing.is_set():
            status_text, status_color = "AUDIO PLAYING — MOTORS LOCKED", (0, 0, 255)
        elif stable_cmd:
            status_text, status_color = f"Gesture: {stable_cmd}", (0, 220, 80)
        elif raw_cmd:
            status_text, status_color = f"Confirming: {raw_cmd}...", (0, 180, 220)
        else:
            status_text, status_color = "No gesture", (160, 160, 160)

        cv2.rectangle(frame, (0, 0), (frame.shape[1], 40), (0, 0, 0), -1)
        cv2.putText(frame, status_text, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2, cv2.LINE_AA)
        for idx, line in enumerate(legend):
            cv2.putText(frame, line, (10, frame.shape[0] - 10 - idx * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.imshow("DuskBuddy — Gesture Control  [ESC to quit]", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

# ═══════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════
async def main():
    global _loop
    _loop = asyncio.get_running_loop()
    log.info("=" * 60)
    log.info("  DuskBuddy Server v4")
    log.info(f"  WebSocket  : ws://0.0.0.0:{WS_PORT}")
    log.info(f"  TTS Voice  : {VOICE}  @ {SAMPLE_RATE} Hz")
    log.info(f"  LLM Model  : {GROQ_MODEL}")
    log.info(f"  Gesture confirm: {GESTURE_CONFIRM} frames | resend: {GESTURE_RESEND}s")
    log.info(f"  Audio tail guard: {AUDIO_TAIL_S}s")
    log.info("=" * 60)

    init_serial()
    threading.Thread(target=speech_loop,       daemon=True, name="speech").start()
    threading.Thread(target=gesture_loop,       daemon=True, name="gesture").start()
    threading.Thread(target=serial_reader_loop, daemon=True, name="serial_reader").start()

    async with websockets.serve(
        ws_handler, WS_HOST, WS_PORT,
        max_size=None,
        ping_interval=20,
        ping_timeout=30,
    ):
        log.info(f"[WS] Server listening on ws://0.0.0.0:{WS_PORT}")
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())