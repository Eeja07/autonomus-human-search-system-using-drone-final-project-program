#!/usr/bin/env python3
"""
GStreamer Pipeline with Tee
===========================
Camera → tee → Branch 1: H264 encode → UDP stream
              → Branch 2: appsink (low-res) → YOLO queue

No OpenCV capture loop. Frames come from appsink only.
"""

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib

import logging
import numpy as np
import threading
import time
from queue import Queue, Full

logger = logging.getLogger(__name__)

Gst.init(None)


class GStreamerPipeline:
    """
    Single GStreamer pipeline with tee:
    - Branch 1: H264 UDP stream for web viewing
    - Branch 2: appsink for YOLO (416x416, RGB)
    """

    # Max frames buffered for YOLO. Drop oldest on overflow (no blocking).
    YOLO_QUEUE_SIZE = 2

    def __init__(
        self,
        device: str = "/dev/video0",
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        yolo_width: int = 320,
        yolo_height: int = 320,
        udp_host: str = "0.0.0.0",
        udp_port: int = 5600,
    ):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.yolo_width = yolo_width
        self.yolo_height = yolo_height
        self.udp_host = udp_host
        self.udp_port = udp_port

        self.pipeline = None
        self.appsink = None
        self.running = False

        # Thread-safe queue for YOLO frames (numpy BGR uint8)
        self.frame_queue: Queue = Queue(maxsize=self.YOLO_QUEUE_SIZE)
        self.latest_frame = None  # <--- TAMBAHKAN INI UNTUK WEB STREAM
        self._gst_frame_times = []
        self.current_fps = 0.0

        self._glib_loop = None
        self._glib_thread = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        """Build and start the GStreamer pipeline."""
        if self.running:
            return

        pipeline_str = self._build_pipeline_string()
        logger.info("GST pipeline:\n%s", pipeline_str)

        self.pipeline = Gst.parse_launch(pipeline_str)

        # Grab appsink element to connect new-sample callback
        self.appsink = self.pipeline.get_by_name("yolo_sink")
        if self.appsink is None:
            raise RuntimeError("appsink element 'yolo_sink' not found in pipeline")

        self.appsink.set_property("emit-signals", True)
        self.appsink.set_property("sync", False)
        self.appsink.connect("new-sample", self._on_new_sample)

        # Bus watch for errors
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)
        bus.connect("message::eos", self._on_bus_eos)

        self.pipeline.set_state(Gst.State.PLAYING)
        self.running = True

        # GLib main loop (required for bus signals)
        self._glib_loop = GLib.MainLoop()
        self._glib_thread = threading.Thread(
            target=self._glib_loop.run, daemon=True, name="gst-glib"
        )
        self._glib_thread.start()
        logger.info("GStreamer pipeline started")

    def stop(self):
        """Stop and cleanup pipeline."""
        if not self.running:
            return
        self.running = False

        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None

        if self._glib_loop and self._glib_loop.is_running():
            self._glib_loop.quit()

        # Drain queue
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except Exception:
                break

        logger.info("GStreamer pipeline stopped")

    def is_running(self) -> bool:
        return self.running

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _build_pipeline_string(self) -> str:
        """
        Build GStreamer pipeline string.

        Camera → videoconvert → tee
          tee. → queue → videoscale → H264 → rtph264pay → udpsink    (stream)
          tee. → queue → videoscale → videoconvert → appsink           (YOLO)
        """
        src = (
            f"v4l2src device={self.device} "
            f"! videoconvert ! videoscale "
            f"! video/x-raw,width={self.width},height={self.height} "
            f"! tee name=t "
        )

        # Branch 1: H264 UDP stream
        # Use v4l2h264enc (RPi hardware) with fallback note; omit if not available
        stream_branch = (
            f"t. ! queue max-size-buffers=2 leaky=downstream "
            f"! videoscale ! video/x-raw,width={self.width},height={self.height} "
            f"! videoconvert "  # <--- TAMBAHKAN BARIS INI
            #   f"! v4l2h264enc "
            f"! x264enc tune=zerolatency speed-preset=ultrafast "
            f"! h264parse "
            f"! rtph264pay config-interval=1 pt=96 "
            f"! udpsink host={self.udp_host} port={self.udp_port} sync=false "
        )

        # Branch 2: appsink for YOLO (low-res RGB)
        yolo_branch = (
            f"t. ! queue max-size-buffers=1 leaky=downstream "
            f"! videoscale ! video/x-raw,width={self.yolo_width},height={self.yolo_height} "
            f"! videoconvert ! video/x-raw,format=BGR "
            f"! appsink name=yolo_sink max-buffers=1 drop=true emit-signals=true "
        )

        return src + stream_branch + yolo_branch

    def _on_new_sample(self, appsink) -> Gst.FlowReturn:
        # 1. Ambil sample dari GStreamer
        sample = appsink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()
        
        # Ekstraksi dimensi gambar
        structure = caps.get_structure(0)
        h = structure.get_value("height")
        w = structure.get_value("width")

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        try:
            # 2. SEGERA HITUNG FPS (Lakukan sebelum proses lain agar akurat)
            now = time.time()
            self._gst_frame_times.append(now)
            
            # Gunakan window 30 frame untuk rata-rata yang stabil
            if len(self._gst_frame_times) > 30:
                self._gst_frame_times.pop(0)
            
            if len(self._gst_frame_times) >= 2:
                # Kalkulasi selisih waktu antara frame pertama dan terakhir di list
                duration = self._gst_frame_times[-1] - self._gst_frame_times[0]
                if duration > 0:
                    self.current_fps = len(self._gst_frame_times) / duration

            # 3. Konversi buffer ke numpy array
            frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((h, w, 3)).copy()
            self.latest_frame = frame.copy()

            # 4. KIRIM KE YOLO (Non-blocking / Tidak Menunggu)
            # Menggunakan put_nowait agar thread GStreamer tidak tertahan jika YOLO lambat
            try:
                self.frame_queue.put_nowait(frame)
            except Full:
                # Jika antrean penuh, buang frame lama dan masukkan yang terbaru
                try:
                    self.frame_queue.get_nowait() # Buang frame "basi"
                    self.frame_queue.put_nowait(frame) # Masukkan frame segar
                except:
                    pass

        except Exception as e:
            # Gunakan logging agar tidak crash saat terbang
            logging.error(f"Error in _on_new_sample: {e}")
        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

    def _on_bus_error(self, bus, message):
        err, debug = message.parse_error()
        logger.error("GStreamer error: %s | %s", err, debug)
        self.stop()

    def _on_bus_eos(self, bus, message):
        logger.warning("GStreamer EOS received")
        self.stop()

    @property
    def camera(self):
        """Mock property agar main.py bisa menginjeksi objek ini."""
        return self

    def isOpened(self):
        """Mock method agar pengecekan di helper tidak gagal."""
        return self.running