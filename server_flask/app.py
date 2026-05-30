#!/usr/bin/env python3
"""
Main Application - Fixed Version
================================
Drone autonomy system backend for Raspberry Pi 5.
"""

import sys
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path

# --- FIX: MODULE PATH SETUP ---
# Mendapatkan path absolut dari file app.py dan menambahkannya ke sys.path
# Ini mencegah ModuleNotFoundError: No module named 'pipeline'
current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir))
sys.path.append('/usr/lib/python3/dist-packages')

from queue import Queue
import asyncio
import logging
import threading
import time

# pyrefly: ignore [missing-import]
from dotenv import load_dotenv
# pyrefly: ignore [missing-import]
from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO
# pyrefly: ignore [missing-import]
from mavsdk import System

# Import komponen lokal
from core.mavsdk_listener import MAVSDKListener
from core.autonomy_scout import AutonomyController
from core.watchdog import ConnectionWatchdog
from pipeline.gst_pipeline import GStreamerPipeline
from pipeline.yolo_consumer import YOLOConsumer
from web.routes import register_monitoring_routes

load_dotenv()

# --- Persistent File Logging Setup ---
from logging.handlers import RotatingFileHandler

_log_dir = current_dir / "logs"
_log_dir.mkdir(parents=True, exist_ok=True)

_log_format = "%(asctime)s %(levelname)s %(name)s: %(message)s"
_log_datefmt = "%Y-%m-%d %H:%M:%S"

logging.basicConfig(
    level=logging.INFO,
    format=_log_format,
    datefmt=_log_datefmt,
    force=True,
)

_file_handler = RotatingFileHandler(
    str(_log_dir / "backend.log"),
    maxBytes=5 * 1024 * 1024,   # 5 MB per file
    backupCount=3,               # keep 3 rotated backups
    encoding="utf-8",
)
_file_handler.setLevel(logging.INFO)
_file_handler.setFormatter(logging.Formatter(_log_format, datefmt=_log_datefmt))
logging.getLogger().addHandler(_file_handler)

logger = logging.getLogger("main")

# ============================================================================
# CONFIG (dari .env)
# ============================================================================

MAVSDK_ADDR     = os.getenv("MAVSDK_SYSTEM_ADDRESS", "udp://:14540")
MODEL_PATH      = os.getenv("YOLO_MODEL_PATH", "yolov8n320.onnx")
CAMERA_DEVICE   = os.getenv("CAMERA_DEVICE", "/dev/video0")
CAMERA_WIDTH    = int(os.getenv("CAMERA_WIDTH", 1280))
CAMERA_HEIGHT   = int(os.getenv("CAMERA_HEIGHT", 720))
CAMERA_FPS      = int(os.getenv("CAMERA_FPS", 30))
YOLO_WIDTH      = int(os.getenv("YOLO_WIDTH", 320))
YOLO_HEIGHT     = int(os.getenv("YOLO_HEIGHT", 320))
UDP_STREAM_HOST = os.getenv("UDP_STREAM_HOST", "0.0.0.0")
UDP_STREAM_PORT = int(os.getenv("UDP_STREAM_PORT", 5600))
FLASK_PORT      = int(os.getenv("PORT", 3000))
YOLO_CONF       = float(os.getenv("YOLO_CONF", 0.5))
CORS_ORIGINS    = os.getenv("CORS_ORIGINS", "http://localhost:5173,http://localhost:3000").split(",")

# ============================================================================
# FLASK + SOCKETIO
# ============================================================================

app = Flask(__name__)
app.config["SECRET_KEY"] = os.getenv("SECRET_KEY", "drone-secret")

CORS(app, resources={"/*": {"origins": CORS_ORIGINS, "supports_credentials": True}})

socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode="threading",
    ping_timeout=60,
    ping_interval=25,
    logger=False,
    engineio_logger=False,
)

# ============================================================================
# COMPONENT INSTANCES
# ============================================================================

gst_pipeline: GStreamerPipeline = None
yolo_consumer: YOLOConsumer = None
autonomy: AutonomyController = None
mavsdk_listener: MAVSDKListener = None
watchdog: ConnectionWatchdog = None
drone: System = None

# ============================================================================
# GPS TIME SYNC (one-shot, startup only)
# ============================================================================

# Flag to guarantee the sync runs only once per process lifetime.
_gps_time_synced: bool = False


async def sync_time_from_gps(drone_instance, timeout_sec=60):
    """
    Sync Raspberry Pi system clock from GPS UTC time via MAVSDK unix_epoch_time().
    One-shot per startup — no internet required.

    Source: drone.telemetry.unix_epoch_time()
      - Populated by PX4 SYSTEM_TIME MAVLink message (GPS-derived UTC)
      - Valid only after u-blox M10 has a 3-D GPS fix
      - Returns time_usec: microseconds since Unix epoch (UTC)

    Sudoers note: add this line to /etc/sudoers.d/gps-timesync:
      <user> ALL=(ALL) NOPASSWD: /bin/date
    """
    global _gps_time_synced

    # Duplicate-sync guard: one call per process
    if _gps_time_synced:
        return

    logger.info("TIME → waiting GPS time sync")

    try:
        deadline = asyncio.get_event_loop().time() + timeout_sec

        async for epoch in drone_instance.telemetry.unix_epoch_time():
            # Timeout guard
            if asyncio.get_event_loop().time() > deadline:
                logger.warning("TIME → GPS time sync skipped (invalid/no fix) — timeout %ds", timeout_sec)
                return

            epoch_us = epoch.time_usec  # microseconds since Unix epoch (UTC)

            # Sanity: zero means no GPS time yet
            if epoch_us == 0:
                continue

            gps_dt = datetime.fromtimestamp(epoch_us / 1_000_000.0, tz=timezone.utc)

            # Sanity: reject implausible years (e.g., GPS week rollover, cold boot artefacts)
            if gps_dt.year < 2024:
                logger.warning(
                    "TIME → GPS time sync skipped (invalid/no fix) — implausible year %d", gps_dt.year
                )
                return

            logger.info("TIME → GPS UTC acquired: %s", gps_dt.strftime("%Y-%m-%d %H:%M:%S UTC"))

            time_str = gps_dt.strftime("%Y-%m-%d %H:%M:%S")
            result = subprocess.run(
                ["sudo", "date", "-u", "-s", time_str],
                capture_output=True, text=True, timeout=5,
            )

            if result.returncode == 0:
                _gps_time_synced = True          # prevent any future duplicate calls
                logger.info("TIME → system clock synchronized from GPS (%s UTC)", time_str)
            else:
                logger.error(
                    "TIME → system clock set failed: %s", result.stderr.strip()
                )
            return  # one-shot — exit after first attempt regardless of outcome

    except asyncio.CancelledError:
        logger.warning("TIME → GPS time sync cancelled.")
    except Exception as e:
        logger.error("TIME → GPS time sync error: %s", e)

# ============================================================================
# ASYNCIO MAIN (MAVSDK Core)
# ============================================================================

async def async_main():
    global drone, mavsdk_listener, autonomy, watchdog

    logger.info("Connecting to drone: %s", MAVSDK_ADDR)
    drone = System()
    await drone.connect(system_address=MAVSDK_ADDR)

    logger.info("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            logger.info("Drone connected via MAVSDK")
            break

    # ---- One-shot GPS time sync ----
    await sync_time_from_gps(drone)

    # ---- Callbacks untuk transisi mode ----
    async def on_offboard_enter():
        logger.info("OFFBOARD DETECTED → Autonomy Started")
        await autonomy.start()

    async def on_offboard_exit():
        logger.info("OFFBOARD EXITED → Autonomy Stopped")
        await autonomy.stop()

    async def on_disconnect():
        logger.error("CONNECTION LOST → Emergency RTL")
        if mavsdk_listener:
            mavsdk_listener.state["connected"] = False
        await autonomy.emergency_stop()
        socketio.emit("watchdog:status", {"status": "disconnected", "message": "CONNECTION LOST"}, namespace="/")

    async def on_reconnect():
        logger.info("CONNECTION RESTORED")
        if mavsdk_listener:
            mavsdk_listener.state["connected"] = True
            await mavsdk_listener.start()
        socketio.emit("watchdog:status", {"status": "connected", "message": "Connection restored"}, namespace="/")

    # ---- Inisialisasi Komponen Autonomy ----
    safe_detection_queue = yolo_consumer.detection_queue if yolo_consumer else Queue()

    autonomy = AutonomyController(
        drone=drone,
        detection_queue=safe_detection_queue,
        telemetry_state=None, # Akan di-wire ke mavsdk_listener.state
        socketio=socketio
    )

    mavsdk_listener = MAVSDKListener(
        drone=drone,
        socketio=socketio,
        on_offboard_enter=on_offboard_enter,
        on_offboard_exit=on_offboard_exit,
    )
    mavsdk_listener.state["connected"] = True
    autonomy.state_dict = mavsdk_listener.state # Wiring state

    # Injeksi Camera Source untuk Screenshot di Scout Mode
    if gst_pipeline and hasattr(gst_pipeline, "latest_frame"):
        autonomy.set_camera_source(gst_pipeline)
        logger.info("GStreamer source injected to AutonomyController")

    watchdog = ConnectionWatchdog(
        drone=drone,
        socketio=socketio,
        on_disconnect=on_disconnect,
        on_reconnect=on_reconnect,
    )

    # Start Async Services
    await mavsdk_listener.start()
    await watchdog.start()

    # Loop Monitoring Detections ke Frontend
    while True:
        if yolo_consumer and yolo_consumer.latest_result:
            res = yolo_consumer.latest_result
            socketio.emit(
                "yolo:detections",
                {
                    "fps": round(res.fps, 1),
                    "detections": [
                        {"class_name": d.class_name, "confidence": round(d.confidence, 2)}
                        for d in res.detections
                    ],
                },
                namespace="/",
            )
        await asyncio.sleep(0.1)

def run_async_main():
    """Event loop handler untuk thread MAVSDK."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(async_main())
    except Exception as e:
        logger.error("MAVSDK Thread Crash: %s", e)

# ============================================================================
# PIPELINE STARTUP
# ============================================================================

def start_pipeline():
    global gst_pipeline, yolo_consumer

    # Validasi path model
    full_model_path = str(current_dir / MODEL_PATH)
    if not os.path.exists(full_model_path):
        logger.error(f"YOLO Model not found at: {full_model_path}")
        raise FileNotFoundError(f"Model {full_model_path} missing.")

    logger.info("Initializing GStreamer Pipeline...")
    gst_pipeline = GStreamerPipeline(
        device=CAMERA_DEVICE,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        fps=CAMERA_FPS,
        yolo_width=YOLO_WIDTH,
        yolo_height=YOLO_HEIGHT,
        udp_host=UDP_STREAM_HOST,
        udp_port=UDP_STREAM_PORT,
    )
    gst_pipeline.start()

    logger.info("Initializing YOLO Consumer...")
    yolo_consumer = YOLOConsumer(
        frame_queue=gst_pipeline.frame_queue,
        model_path=full_model_path,
        conf_threshold=YOLO_CONF,
    )
    yolo_consumer.start()

# ============================================================================
# WEB SOCKET HANDLERS
# ============================================================================

@socketio.on("connect")
def handle_connect():
    logger.info("Web client connected")
    socketio.emit("connection:status", {
        "connected": True,
        "message": "Connected to Drone Primary System",
        "mode": "monitoring_only"
    })
    if mavsdk_listener:
        socketio.emit("drone:status", mavsdk_listener.state)

# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    print("\n" + "="*50)
    print("  DRONE AUTONOMY BACKEND - RASPBERRY PI 5 EDITION")
    print("="*50 + "\n")

    # 1. Jalankan Pipeline Video & AI
    try:
        start_pipeline()
    except Exception as e:
        logger.critical(f"FATAL: Pipeline failed to start: {e}")
        sys.exit(1)

    # 2. Registrasi API Routes
    register_monitoring_routes(
        app,
        get_listener=lambda: mavsdk_listener,
        get_autonomy=lambda: autonomy,
        get_pipeline=lambda: gst_pipeline,
        get_yolo=lambda: yolo_consumer
    )

    # 3. Jalankan MAVSDK di Background Thread
    mav_thread = threading.Thread(target=run_async_main, daemon=True, name="MAVSDK-Thread")
    mav_thread.start()

    # 4. Jalankan Flask Server
    logger.info(f"Flask Monitoring active on port {FLASK_PORT}")
    try:
        socketio.run(
            app,
            host="0.0.0.0",
            port=FLASK_PORT,
            debug=False,
            use_reloader=False # Reloader dimatikan agar thread tidak berlipat ganda
        )
    except KeyboardInterrupt:
        logger.info("Shutting down system...")
