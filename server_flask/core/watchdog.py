#!/usr/bin/env python3
# Shebang ini memberi tahu sistem Unix/Linux bahwa file dijalankan menggunakan interpreter Python 3.
"""
Connection Watchdog
===================
Monitors MAVSDK connection state.
On disconnect: stops autonomy, stops telemetry, emits CONNECTION_LOST to frontend.
On reconnect: auto-resumes telemetry and re-checks flight mode.
"""

# asyncio digunakan untuk menjalankan watchdog sebagai task asynchronous di background.
import asyncio
# logging digunakan untuk mencatat status koneksi, warning disconnect, dan error callback/emit.
import logging
# time digunakan untuk membuat timestamp saat mengirim status watchdog ke frontend.
import time
# Optional digunakan untuk type hint task yang bisa None, Callable untuk type hint callback connect/disconnect.
from typing import Optional, Callable

# logger dibuat untuk modul ini agar pesan log dapat ditelusuri berasal dari watchdog.py.
logger = logging.getLogger(__name__)


# Class ini bertugas memantau koneksi MAVSDK dan memberi tahu sistem saat drone disconnect/reconnect.
class ConnectionWatchdog:
    """
    Watches mavsdk System.core.connection_state().
    Calls registered callbacks on connect/disconnect events.
    """

    # Konstruktor menerima objek drone, socketio, dan callback yang akan dipanggil saat koneksi berubah.
    def __init__(self, drone, socketio, on_disconnect: Callable, on_reconnect: Callable):
        """
        Args:
            drone: mavsdk.System instance
            socketio: Flask-SocketIO instance for emitting to frontend
            on_disconnect: async callable() called when connection lost
            on_reconnect: async callable() called when connection restored
        """
        # drone adalah instance mavsdk.System yang menyediakan stream core.connection_state().
        self.drone = drone
        # socketio digunakan untuk mengirim event status watchdog ke frontend.
        self.socketio = socketio
        # _on_disconnect adalah callback asynchronous saat koneksi drone hilang.
        self._on_disconnect = on_disconnect
        # _on_reconnect adalah callback asynchronous saat koneksi drone kembali tersambung.
        self._on_reconnect = on_reconnect

        # connected menyimpan status koneksi terakhir yang diketahui oleh watchdog.
        self.connected: bool = False
        # _running menandakan apakah watchdog sedang aktif memantau koneksi.
        self._running: bool = False
        # _task menyimpan task asyncio untuk loop watchdog; awalnya None sebelum start().
        self._task: Optional[asyncio.Task] = None

    # start menjalankan watchdog loop sebagai task background.
    async def start(self):
        """Start watchdog loop as background asyncio task."""
        # Jika watchdog sudah berjalan, jangan membuat task baru agar tidak ada loop ganda.
        if self._running:
            # Return membuat start() aman dipanggil berulang kali.
            return
        # Mengaktifkan flag running sehingga _watch_loop boleh berjalan.
        self._running = True
        # Membuat task asynchronous untuk memantau state koneksi MAVSDK.
        self._task = asyncio.create_task(self._watch_loop())
        # Log info menandakan watchdog sudah aktif.
        logger.info("ConnectionWatchdog started")

    # stop menghentikan watchdog dan membatalkan task pemantauan koneksi.
    async def stop(self):
        # Mengubah flag menjadi False agar loop berhenti pada iterasi berikutnya.
        self._running = False
        # Jika task watchdog sudah pernah dibuat, task perlu dibatalkan.
        if self._task:
            # Mengirim sinyal cancel ke task _watch_loop.
            self._task.cancel()
            # try digunakan karena await task yang dibatalkan akan melempar CancelledError.
            try:
                # Menunggu task benar-benar selesai.
                await self._task
            # CancelledError dianggap normal saat proses stop().
            except asyncio.CancelledError:
                # pass berarti pembatalan task tidak dianggap error.
                pass
        # Log info menandakan watchdog sudah berhenti.
        logger.info("ConnectionWatchdog stopped")

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    # _watch_loop membaca stream connection_state MAVSDK dan mendeteksi transisi koneksi.
    async def _watch_loop(self):
        """Stream connection_state and fire callbacks on transitions."""
        # Log awal untuk menunjukkan watchdog mulai memantau koneksi.
        logger.info("Watchdog: watching connection state...")
        # try menangkap error stream MAVSDK agar tercatat.
        try:
            # async for menerima update connection_state secara streaming dari MAVSDK.
            async for state in self.drone.core.connection_state():
                # Jika watchdog sudah diminta berhenti, keluar dari loop.
                if not self._running:
                    # break menghentikan pembacaan stream connection_state.
                    break

                # was_connected menyimpan status koneksi sebelum update terbaru.
                was_connected = self.connected
                # connected diperbarui berdasarkan state.is_connected dari MAVSDK.
                self.connected = state.is_connected

                # Kondisi ini mendeteksi transisi dari tidak tersambung menjadi tersambung.
                if self.connected and not was_connected:
                    # Log info karena koneksi drone berhasil dipulihkan.
                    logger.info("Watchdog: CONNECTION RESTORED")
                    # Mengirim status connected ke frontend melalui Socket.IO.
                    self._emit_status("connected", "Connection restored")
                    # Memanggil callback reconnect secara aman.
                    await self._safe_callback(self._on_reconnect)

                # Kondisi ini mendeteksi transisi dari tersambung menjadi tidak tersambung.
                elif not self.connected and was_connected:
                    # Log warning karena kehilangan koneksi adalah kondisi berisiko.
                    logger.warning("Watchdog: CONNECTION LOST")
                    # Mengirim status disconnected ke frontend melalui Socket.IO.
                    self._emit_status("disconnected", "CONNECTION LOST")
                    # Memanggil callback disconnect secara aman.
                    await self._safe_callback(self._on_disconnect)

        # CancelledError terjadi saat task watchdog dibatalkan oleh stop().
        except asyncio.CancelledError:
            # pass membuat pembatalan task dianggap alur normal.
            pass
        # Exception lain dari stream watchdog dicatat sebagai error.
        except Exception as e:
            # Log error membantu diagnosis masalah stream koneksi MAVSDK.
            logger.error("Watchdog error: %s", e)

    # _safe_callback menjalankan callback async dan menangkap error agar watchdog tidak berhenti.
    async def _safe_callback(self, cb: Callable):
        # try menjaga agar exception dari callback tidak merusak loop watchdog.
        try:
            # Menjalankan callback yang diberikan, misalnya prosedur disconnect atau reconnect.
            await cb()
        # Jika callback gagal, error dicatat.
        except Exception as e:
            # Log error callback untuk debugging.
            logger.error("Watchdog callback error: %s", e)

    # _emit_status mengirim status koneksi watchdog ke frontend melalui Socket.IO.
    def _emit_status(self, status: str, message: str):
        # try menjaga agar kegagalan emit tidak menghentikan watchdog.
        try:
            # Emit event watchdog:status dengan status, pesan, dan timestamp Unix.
            self.socketio.emit("watchdog:status", {
                # status berisi nilai seperti "connected" atau "disconnected".
                "status": status,
                # message berisi teks penjelas yang dapat ditampilkan frontend.
                "message": message,
                # timestamp memberi waktu kejadian koneksi dalam format epoch seconds.
                "timestamp": time.time(),
            }, namespace="/")
        # Jika emit gagal, error dicatat.
        except Exception as e:
            # Log error emit watchdog untuk diagnosis Socket.IO/frontend.
            logger.error("Watchdog emit error: %s", e)
