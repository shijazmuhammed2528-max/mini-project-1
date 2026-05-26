""" DuskBuddy Server v6
====================
FIXES vs v5:
  1. AUDIO STREAM PACING (ROOT CAUSE FIX for cut/broken audio):
     v5 sent all binary chunks with asyncio.sleep(0) — a full-speed burst that
     overwhelmed the ESP32's TCP receive buffer while its loop was busy doing
     two OLED I2C flushes (~10 ms each).  v6 paces each chunk to 80 % of
     real-time playback speed so the DMA buffer stays fed without flooding TCP.
  2. Remaining sleep after last chunk is calculated from actual elapsed time,
     so audio_playing is never unblocked too early or too late regardless of
     how fast/slow the network is.
  3. All other v5 fixes retained (gesture queue, per-client send, boot greeting,
     audio tail guard, gesture flush on new command, GESTURE_RESEND via queue).

CHANGES in this revision:
  • VOICE changed to en-US-JennyNeural — warm, natural American female voice.
  • VOLUME_LEVEL (0–100 %) added. Converts to dB gain via standard audio math
    so 50 % = unity (0 dB), 100 % = +6 dB boost, ~1 % = near silence.

GESTURE FIX:
  1-finger (index only)       = FORWARD
  2-fingers (index + middle)  = BACKWARD

Architecture:
  PC ──[Binary PCM, paced]──► ESP32 → I2S speaker
  PC ──[Text "END"]──────────► ESP32 → clear audioPlaying flag
  PC ──[Text gesture cmd]────► ESP32 → motor driver (L298N)

Install:
    pip install websockets edge-tts pydub opencv-python mediapipe
    pip install SpeechRecognition groq
    ffmpeg must be on PATH
"""

import asyncio
import threading
import logging
import io
import math
import time
import cv2
import mediapipe as mp
import speech_recognition as sr
import websockets
import edge_tts
from pydub import AudioSegment
from groq import Groq

# ═══════════════════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════════════════
WS_HOST      = "0.0.0.0"
WS_PORT      = 5000

# ─── Voice ──────────────────────────────────────────────────────────────────
# en-US-JennyNeural  → warm, natural American female  ★ recommended
# en-US-AriaNeural   → bright, expressive American female
# en-GB-SoniaNeural  → smooth British female
# en-AU-NatashaNeural→ friendly Australian female
VOICE        = "en-US-AriaNeural"

# ─── Volume ─────────────────────────────────────────────────────────────────
# 0–100 %.  50 % = unity gain (0 dB).  100 % = +6 dB boost.  ~1 % = silence.
# Formula: gain_dB = 20·log₁₀(VOLUME_LEVEL / 100) + 6
VOLUME_LEVEL = 10      # ← change this to adjust loudness (0–100)

SAMPLE_RATE  = 22050
CHANNELS     = 1
SAMPLE_WIDTH = 2          # bytes per sample (16-bit PCM)
CHUNK_BYTES  = 4096

GROQ_API_KEY = "API_KEY"
GROQ_MODEL   = "openai/gpt-oss-20b"  ##  Ai model

GESTURE_CONFIRM  = 3     # frames gesture must be stable before sending
GESTURE_RESEND   = 1.0   # re-send same command every N seconds while held
AUDIO_TAIL_S     = 0.5   # extra seconds after calculated duration before unblocking

# ── Chunk pacing ────────────────────────────────────────────────────────────
CHUNK_PACE_FACTOR = 0.80


# ═══════════════════════════════════════════════════════════════════════════
#  VOLUME HELPER
# ═══════════════════════════════════════════════════════════════════════════
def _volume_to_db(pct: int | float) -> float:
    """Convert a 0–100 % volume level to a pydub-compatible dB gain.

    Scale:
      100 % → +6 dB  (maximum boost)
       50 % →  0 dB  (unity / unchanged)
       25 % → −6 dB
        1 % → −34 dB (near-silence; clamp prevents log(0))
    """
    clamped = max(1, min(100, pct))          # guard against 0 / out-of-range
    return round(20 * math.log10(clamped / 100) + 6, 2)


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
audio_playing = threading.Event()   # set while audio is streaming / playing
_ws_clients: set = set()
_ws_lock = threading.Lock()
_loop: asyncio.AbstractEventLoop | None = None

# ─── GESTURE QUEUE ─────────────────────────────────────────────────────────
_gesture_queue: asyncio.Queue | None = None   # created in main()

# ═══════════════════════════════════════════════════════════════════════════
#  WEBSOCKET BROADCAST HELPERS
# ═══════════════════════════════════════════════════════════════════════════
async def _send_text_to_all(msg: str):
    """Send a text frame to every connected client. Dead clients are removed."""
    global _ws_clients
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


async def _send_binary_to_all(data: bytes):
    """Send a binary frame to every connected client. Dead clients removed."""
    global _ws_clients
    with _ws_lock:
        clients = set(_ws_clients)
    dead: set = set()
    for client in clients:
        try:
            await client.send(data)
        except Exception:
            dead.add(client)
    if dead:
        with _ws_lock:
            _ws_clients -= dead

# ═══════════════════════════════════════════════════════════════════════════
#  GESTURE SENDER COROUTINE
# ═══════════════════════════════════════════════════════════════════════════
async def gesture_sender_loop():
    log.info("[GestureSender] Started")
    while True:
        cmd = await _gesture_queue.get()

        while audio_playing.is_set():
            await asyncio.sleep(0.05)

        print(f"\n{'='*46}")
        print(f"  [WS → ESP32]  GESTURE: {cmd}")
        print(f"{'='*46}")
        log.info(f"[GESTURE→WS] {cmd}")
        await _send_text_to_all(cmd)
        _gesture_queue.task_done()


def enqueue_gesture(cmd: str):
    """Called from the gesture thread. Flushes stale pending commands first."""
    if _loop is None or _gesture_queue is None:
        return
    while not _gesture_queue.empty():
        try:
            _gesture_queue.get_nowait()
            _gesture_queue.task_done()
        except asyncio.QueueEmpty:
            break
    asyncio.run_coroutine_threadsafe(_gesture_queue.put(cmd), _loop)

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

    gain_db = _volume_to_db(VOLUME_LEVEL)   # ← volume applied here

    audio = (
        AudioSegment.from_mp3(mp3_buf)
        .set_frame_rate(SAMPLE_RATE)
        .set_channels(CHANNELS)
        .set_sample_width(SAMPLE_WIDTH)
        .apply_gain(gain_db)                # ← replaces the old hard-coded +3 dB
    )
    log.info(
        f"PCM ready — {len(audio.raw_data):,} bytes "
        f"({audio.duration_seconds:.2f}s)  "
        f"volume={VOLUME_LEVEL}% → {gain_db:+.1f} dB"
    )
    return audio.raw_data

# ═══════════════════════════════════════════════════════════════════════════
#  STREAM PCM → ESP32  (paced sends)
# ═══════════════════════════════════════════════════════════════════════════
async def stream_audio_to_clients(pcm: bytes):
    with _ws_lock:
        clients = set(_ws_clients)
    if not clients:
        log.warning("No ESP32 clients — audio dropped")
        return

    bytes_per_second  = SAMPLE_RATE * CHANNELS * SAMPLE_WIDTH
    play_duration_s   = len(pcm) / bytes_per_second
    total_block_s     = play_duration_s + AUDIO_TAIL_S

    chunk_duration_s  = CHUNK_BYTES / bytes_per_second
    send_interval_s   = chunk_duration_s * CHUNK_PACE_FACTOR

    log.info(
        f"Streaming {play_duration_s:.2f}s of audio | "
        f"chunk interval {send_interval_s*1000:.1f} ms | "
        f"total block {total_block_s:.2f}s"
    )

    audio_playing.set()
    try:
        loop       = asyncio.get_event_loop()
        send_start = loop.time()

        for idx, i in enumerate(range(0, len(pcm), CHUNK_BYTES)):
            chunk = pcm[i : i + CHUNK_BYTES]
            await _send_binary_to_all(chunk)

            expected = send_start + (idx + 1) * send_interval_s
            sleep_s  = expected - loop.time()
            if sleep_s > 0:
                await asyncio.sleep(sleep_s)

        await _send_text_to_all("END")

        elapsed     = loop.time() - send_start
        remaining_s = max(0.0, total_block_s - elapsed)
        log.info(f"All bytes sent (elapsed {elapsed:.2f}s) — "
                 f"waiting {remaining_s:.2f}s for ESP32 DMA drain")
        await asyncio.sleep(remaining_s)

    finally:
        audio_playing.clear()
        log.info("Motors unblocked — gesture queue active")

# ═══════════════════════════════════════════════════════════════════════════
#  TTS + STREAM (thread-safe wrapper)
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
        boot_pcm = await text_to_pcm("Ready")
        await stream_audio_to_clients(boot_pcm)

        async for message in websocket:
            if isinstance(message, str):
                log.debug(f"[WS] ESP32 says: {message}")
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
    if     i and not m and not r and not p:  return "BACKWARD"
    if     i and     m and not r and not p:  return "FORWARD"
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
        "1 finger  (index)            = BACKWARD",
        "2 fingers (index+middle)     = FORWARD",
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
                enqueue_gesture(stable_cmd)
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
    global _loop, _gesture_queue
    _loop = asyncio.get_running_loop()

    _gesture_queue = asyncio.Queue(maxsize=0)

    gain_db = _volume_to_db(VOLUME_LEVEL)
    log.info("=" * 60)
    log.info("  DuskBuddy Server v6")
    log.info(f"  WebSocket  : ws://0.0.0.0:{WS_PORT}")
    log.info(f"  TTS Voice  : {VOICE}  @ {SAMPLE_RATE} Hz")
    log.info(f"  Volume     : {VOLUME_LEVEL}%  →  {gain_db:+.1f} dB")
    log.info(f"  LLM Model  : {GROQ_MODEL}")
    log.info(f"  Gesture confirm  : {GESTURE_CONFIRM} frames | resend: {GESTURE_RESEND}s")
    log.info(f"  Audio tail guard : {AUDIO_TAIL_S}s")
    log.info(f"  Chunk pace factor: {CHUNK_PACE_FACTOR}")
    log.info("=" * 60)

    threading.Thread(target=speech_loop,  daemon=True, name="speech").start()
    threading.Thread(target=gesture_loop, daemon=True, name="gesture").start()

    asyncio.create_task(gesture_sender_loop())

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