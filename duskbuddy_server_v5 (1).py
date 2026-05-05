"""
DuskBuddy Server v5
====================
FIXES vs v4:
  1. ROOT CAUSE FIX: Gesture commands were queued behind the async event loop
     that was busy streaming audio binary chunks — arriving at ESP32 while
     audioPlaying was still true. Fixed with a dedicated asyncio.Queue for
     gesture commands, drained by a separate coroutine that always runs.
  2. Serial gesture input removed (was redundant, caused confusion).
  3. audio_playing cleared ONLY after play_duration_s + AUDIO_TAIL_S so the
     gesture queue stays blocked until ESP32 has actually finished playing.
  4. Per-client gesture send: each client gets its own send so dead clients
     don't block others.
  5. Gesture queue is flushed (old commands dropped) when new command arrives
     so there's no backlog of stale commands after audio ends.
  6. GESTURE_RESEND now uses the queue, not fire-and-forget coroutines.
  7. Boot greeting is "Ready" (~0.5s) — same as v4.

Architecture:
  PC ──[Binary PCM]──► ESP32 → I2S speaker
  PC ──[Text "END"]──► ESP32 → clear audioPlaying flag
  PC ──[Text "FORWARD"/"BACKWARD"/"LEFT"/"RIGHT"/"STOP"/"SPIN_CW"]
                     ──► ESP32 → motor driver (L298N)

Install:
    pip install websockets edge-tts pydub opencv-python mediapipe
    pip install SpeechRecognition groq
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

GROQ_API_KEY = "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"
GROQ_MODEL   = "llama-3.3-70b-versatile"

GESTURE_CONFIRM = 3       # frames gesture must be stable before sending
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
audio_playing = threading.Event()   # set while audio is streaming / playing
_ws_clients: set = set()
_ws_lock = threading.Lock()
_loop: asyncio.AbstractEventLoop | None = None

# ─── GESTURE QUEUE ─────────────────────────────────────────────────────────
# The gesture thread puts commands here; the async gesture_sender drains it.
# Using maxsize=1 so stale commands are dropped — only latest matters.
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
#  GESTURE SENDER COROUTINE  ← THE KEY FIX
#  Runs forever on the event loop. Drains _gesture_queue.
#  Blocked while audio is playing (checks audio_playing before dequeue).
# ═══════════════════════════════════════════════════════════════════════════
async def gesture_sender_loop():
    """
    Dedicated coroutine that forwards gesture commands to ESP32.
    It waits until audio finishes before sending anything, ensuring
    the ESP32's audioPlaying flag is already cleared when the command arrives.
    """
    log.info("[GestureSender] Started")
    while True:
        cmd = await _gesture_queue.get()

        # Wait for audio to finish BEFORE sending the command
        while audio_playing.is_set():
            await asyncio.sleep(0.05)

        print(f"\n{'='*46}")
        print(f"  [WS → ESP32]  GESTURE: {cmd}")
        print(f"{'='*46}")
        log.info(f"[GESTURE→WS] {cmd}")
        await _send_text_to_all(cmd)
        _gesture_queue.task_done()


def enqueue_gesture(cmd: str):
    """
    Called from the gesture thread. Puts the command into the async queue.
    Flushes any stale pending command first (we only want the latest).
    """
    if _loop is None or _gesture_queue is None:
        return
    # Flush stale commands so the queue never backlog
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
# ═══════════════════════════════════════════════════════════════════════════
async def stream_audio_to_clients(pcm: bytes):
    global _ws_clients
    with _ws_lock:
        clients = set(_ws_clients)
    if not clients:
        log.warning("No ESP32 clients — audio dropped")
        return

    bytes_per_second = SAMPLE_RATE * CHANNELS * SAMPLE_WIDTH   # 44100 B/s
    play_duration_s  = len(pcm) / bytes_per_second
    total_block_s    = play_duration_s + AUDIO_TAIL_S
    log.info(f"Blocking motors for {total_block_s:.2f}s (audio {play_duration_s:.2f}s + tail {AUDIO_TAIL_S}s)")

    audio_playing.set()
    try:
        # Stream binary chunks
        for i in range(0, len(pcm), CHUNK_BYTES):
            chunk = pcm[i: i + CHUNK_BYTES]
            await _send_binary_to_all(chunk)
            await asyncio.sleep(0)

        # Signal ESP32 that binary stream is done
        await _send_text_to_all("END")

        # Wait for ESP32 I2S DMA to finish playing
        log.info(f"All bytes sent — waiting {total_block_s:.2f}s for ESP32 playback")
        await asyncio.sleep(total_block_s)

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
    global _ws_clients
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
                enqueue_gesture(stable_cmd)   # ← uses queue, not direct send
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

    # Create the gesture queue in the async context so it belongs to this loop
    _gesture_queue = asyncio.Queue(maxsize=0)   # unbounded; we flush manually

    log.info("=" * 60)
    log.info("  DuskBuddy Server v5")
    log.info(f"  WebSocket  : ws://0.0.0.0:{WS_PORT}")
    log.info(f"  TTS Voice  : {VOICE}  @ {SAMPLE_RATE} Hz")
    log.info(f"  LLM Model  : {GROQ_MODEL}")
    log.info(f"  Gesture confirm : {GESTURE_CONFIRM} frames | resend: {GESTURE_RESEND}s")
    log.info(f"  Audio tail guard: {AUDIO_TAIL_S}s")
    log.info("=" * 60)

    # Start background threads
    threading.Thread(target=speech_loop,  daemon=True, name="speech").start()
    threading.Thread(target=gesture_loop, daemon=True, name="gesture").start()

    # Start gesture sender coroutine on the event loop
    asyncio.create_task(gesture_sender_loop())

    async with websockets.serve(
        ws_handler, WS_HOST, WS_PORT,
        max_size=None,
        ping_interval=20,
        ping_timeout=30,
    ):
        log.info(f"[WS] Server listening on ws://0.0.0.0:{WS_PORT}")
        await asyncio.Future()   # run forever


if __name__ == "__main__":
    asyncio.run(main())
