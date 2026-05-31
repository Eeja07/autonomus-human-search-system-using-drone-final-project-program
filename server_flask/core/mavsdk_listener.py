#!/usr/bin/env python3
# Shebang ini memberi tahu sistem Unix/Linux bahwa file dijalankan menggunakan interpreter Python 3.
"""
MAVSDK Listener
===============
Streams telemetry from MAVSDK into shared state dict.
Detects flight mode transitions.
Triggers AutonomyController start/stop based on OFFBOARD mode.

This is the ONLY trigger for autonomy. Web cannot start/stop autonomy.

Perubahan dari versi sebelumnya:
  - Tambah _offboard_keepalive(): stream setpoint nol terus-menerus ke PX4
    meskipun belum dalam mode OFFBOARD. Ini wajib agar RC switch ke OFFBOARD
    bisa diterima PX4 kapan saja saat drone di udara — karena PX4 hanya mau
    masuk OFFBOARD kalau companion computer sudah aktif kirim setpoint.
"""

# asyncio digunakan untuk menjalankan banyak stream telemetri MAVSDK secara asynchronous bersamaan.
import asyncio
# logging digunakan untuk mencatat status listener, transisi mode, error telemetry, dan keepalive.
import logging
# time digunakan untuk timestamp pembaruan state dan perhitungan interval log keepalive.
import time
# Dict, Optional, dan Callable digunakan sebagai type hint untuk state dictionary dan callback.
from typing import Dict, Optional, Callable

# System adalah class utama MAVSDK yang merepresentasikan koneksi ke drone/Pixhawk.
# pyrefly: ignore [missing-import]
from mavsdk import System
# VelocityNedYaw adalah format setpoint offboard berisi velocity north/east/down dan yaw.
# pyrefly: ignore [missing-import]
from mavsdk.offboard import VelocityNedYaw
# FlightMode adalah enum MAVSDK untuk membandingkan mode terbang, terutama OFFBOARD.
# pyrefly: ignore [missing-import]
from mavsdk.telemetry import FlightMode

# logger dibuat per modul agar log dapat ditelusuri berasal dari mavsdk_listener.py.
logger = logging.getLogger(__name__)


# Class ini bertanggung jawab membaca telemetri MAVSDK, menyimpan ke state bersama, dan memicu autonomy saat OFFBOARD.
class MAVSDKListener:
    """
    Streams all telemetry into self.state dict.
    Monitors flight_mode: if OFFBOARD → start autonomy, else → stop.
    """

    # Konstruktor menerima objek drone, socketio, serta callback saat masuk/keluar OFFBOARD.
    def __init__(
        # self adalah instance MAVSDKListener.
        self,
        # drone adalah objek MAVSDK System yang menyediakan telemetry, offboard, dan action API.
        drone: System,
        # socketio digunakan untuk mengirim status drone ke frontend secara real-time.
        socketio,
        # on_offboard_enter adalah callback yang dipanggil ketika flight mode berubah masuk OFFBOARD.
        on_offboard_enter: Callable,
        # on_offboard_exit adalah callback yang dipanggil ketika flight mode keluar dari OFFBOARD.
        on_offboard_exit: Callable,
    ):
        # Menyimpan referensi drone agar semua method listener bisa membaca telemetri dan mengirim setpoint.
        self.drone = drone
        # Menyimpan socketio agar telemetry broadcast loop bisa emit event ke frontend.
        self.socketio = socketio
        # Callback untuk memulai autonomy saat OFFBOARD aktif.
        self._on_offboard_enter = on_offboard_enter
        # Callback untuk menghentikan autonomy saat OFFBOARD tidak aktif.
        self._on_offboard_exit = on_offboard_exit

        # _running menandakan apakah listener sedang aktif menjalankan task-task stream.
        self._running = False
        # _tasks menyimpan semua task asyncio agar bisa dibatalkan saat stop().
        self._tasks = []

        # Shared telemetry state dict (read by other components)
        # state adalah pusat data telemetri bersama yang dibaca modul lain seperti autonomy dan API.
        self.state: Dict = {
            # connected menunjukkan status koneksi drone; nilai awal False sampai ada komponen lain memperbarui.
            "connected": False,
            # armed menunjukkan apakah drone sedang armed.
            "armed": False,
            # flight_mode menyimpan mode terbang sebagai string yang mudah dikirim ke frontend.
            "flight_mode": "UNKNOWN",
            # in_offboard bernilai True jika flight mode saat ini adalah OFFBOARD.
            "in_offboard": False,
            # battery_pct menyimpan persentase baterai; default 100.0 sebelum telemetri baterai masuk.
            "battery_pct": 100.0,
            # voltage menyimpan tegangan baterai dalam volt jika tersedia dari MAVSDK.
            "voltage": 0.0,
            # position menyimpan latitude, longitude, dan altitude relatif drone.
            "position": {"lat": 0.0, "lon": 0.0, "alt": 0.0},
            # attitude menyimpan roll, pitch, dan yaw dalam derajat.
            "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            # velocity menyimpan kecepatan drone pada frame NED: north, east, down.
            "velocity": {"vx": 0.0, "vy": 0.0, "vz": 0.0},
            # gps menyimpan jumlah satelit dan kualitas fix GPS.
            "gps": {"satellites": 0, "fix": 0},
            # heading menyimpan heading kompas dalam derajat.
            "heading": 0.0,
            # last_update menyimpan timestamp pembaruan terakhir, terutama dari stream flight mode.
            "last_update": 0.0,
        }

        # Track previous offboard state to detect transitions
        # _was_offboard menyimpan status OFFBOARD sebelumnya agar transisi masuk/keluar bisa dideteksi.
        self._was_offboard: bool = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    # start mengaktifkan listener dan membuat semua task stream telemetri.
    async def start(self):
        # Jika listener sudah berjalan, start tidak membuat task ganda.
        if self._running:
            # Return langsung membuat start() aman dipanggil berkali-kali.
            return
        # Menandai listener aktif sehingga semua loop internal boleh berjalan.
        self._running = True

        # Membuat daftar task asynchronous yang berjalan paralel untuk setiap jenis telemetri.
        self._tasks = [
            # Task untuk membaca flight mode dan memicu callback OFFBOARD.
            asyncio.create_task(self._stream_flight_mode()),
            # Task untuk membaca posisi GPS dan altitude.
            asyncio.create_task(self._stream_position()),
            # Task untuk membaca attitude roll/pitch/yaw.
            asyncio.create_task(self._stream_attitude()),
            # Task untuk membaca persentase dan tegangan baterai.
            asyncio.create_task(self._stream_battery()),
            # Task untuk membaca informasi GPS seperti satelit dan fix.
            asyncio.create_task(self._stream_gps()),
            # Task untuk membaca velocity NED.
            asyncio.create_task(self._stream_velocity()),
            # Task untuk membaca status armed/disarmed.
            asyncio.create_task(self._stream_armed()),
            # Task untuk membaca heading kompas.
            asyncio.create_task(self._stream_heading()),
            # Task untuk mengirim ringkasan telemetri ke frontend pada interval tetap.
            asyncio.create_task(self._telemetry_broadcast_loop()),
            # Task baru: stream setpoint nol terus-menerus agar RC switch
            # ke OFFBOARD selalu bisa diterima PX4 saat drone di udara
            # Task keepalive mengirim setpoint nol agar PX4 siap menerima mode OFFBOARD.
            asyncio.create_task(self._offboard_keepalive()),
        ]
        # Log bahwa semua task listener sudah dibuat.
        logger.info("MAVSDKListener started")

    # stop mematikan listener dan membatalkan semua task stream.
    async def stop(self):
        # Mengubah flag agar loop-loop internal berhenti.
        self._running = False
        # Melakukan cancel ke setiap task yang dibuat saat start().
        for t in self._tasks:
            # cancel mengirim sinyal pembatalan ke task asyncio.
            t.cancel()
        # Menunggu semua task selesai; return_exceptions=True membuat CancelledError tidak dilempar ulang.
        await asyncio.gather(*self._tasks, return_exceptions=True)
        # Mengosongkan daftar task karena listener sudah berhenti.
        self._tasks = []
        # Log bahwa listener telah berhenti.
        logger.info("MAVSDKListener stopped")

    # ------------------------------------------------------------------
    # Offboard keepalive
    # ------------------------------------------------------------------

    # _offboard_keepalive mengirim setpoint nol secara periodik agar PX4 menganggap companion siap OFFBOARD.
    async def _offboard_keepalive(self):
        # Blok awal mencoba mengirim satu setpoint sebelum loop dimulai.
        try:
            # Mengirim velocity nol dan yaw nol sebagai setpoint awal.
            await self.drone.offboard.set_velocity_ned(
                # VelocityNedYaw(0,0,0,0) berarti tidak ada gerak north/east/down dan yaw 0 derajat.
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )
        # Error awal diabaikan karena koneksi/offboard mungkin belum siap pada saat startup.
        except Exception:
            # pass membuat listener tetap lanjut meskipun setpoint awal gagal.
            pass 

        # Log bahwa stream keepalive mulai berjalan.
        logger.info("Offboard keepalive stream dimulai")

        # Loop keepalive berjalan selama listener aktif.
        while self._running:
            # try menjaga agar error pengiriman setpoint tidak menghentikan loop keepalive.
            try:
                # UBAH: Cek autonomy_running, bukan in_offboard.
                # Keepalive akan terus menembak setpoint NOL sampai AutonomyController benar-benar mengambil alih.
                # Jika autonomy belum berjalan, listener mengirim setpoint nol sebagai pengganti command autonomy.
                if not self.state.get("autonomy_running", False):
                    # Mengirim setpoint velocity nol ke Pixhawk.
                    await self.drone.offboard.set_velocity_ned(
                        # Setpoint nol menjaga PX4 tetap menerima stream offboard tanpa menggerakkan drone.
                        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                    )
                    # === BARIS SEMENTARA UNTUK SCREENSHOT BAB 3 ===
                    # Hanya log setiap 2 detik agar tidak spam
                    # Import lokal ini dipakai khusus untuk interval log keepalive sementara.
                    import time as _time
                    # Kondisi ini memastikan log keepalive tidak muncul terlalu sering.
                    if not hasattr(self, '_last_keepalive_log') or (_time.time() - self._last_keepalive_log) > 1.0:
                        # Log ini menunjukkan bahwa setpoint nol terus dikirim ke Pixhawk.
                        logger.info("KEEPALIVE → VelocityNedYaw(0.0, 0.0, 0.0, 0.0) → Pixhawk (stream aktif)")
                        # Menyimpan waktu log terakhir untuk pembatasan frekuensi log.
                        self._last_keepalive_log = _time.time()
                    # === AKHIR BARIS SEMENTARA ===
            # Error pengiriman keepalive diabaikan agar loop terus mencoba pada iterasi berikutnya.
            except Exception:
                # pass menjaga keepalive tidak berhenti karena error sesaat.
                pass
            # Delay 0.05 detik berarti keepalive berjalan sekitar 20 Hz.
            await asyncio.sleep(0.05)

    # ------------------------------------------------------------------
    # Telemetry streams
    # ------------------------------------------------------------------

    # _stream_flight_mode membaca mode terbang dan memicu start/stop autonomy berdasarkan OFFBOARD.
    async def _stream_flight_mode(self):
        """
        Critical: detect OFFBOARD transitions and fire callbacks.
        """
        # try menangkap error dari stream MAVSDK agar tercatat di log.
        try:
            # async for membaca data flight_mode secara streaming dari MAVSDK.
            async for fm in self.drone.telemetry.flight_mode():
                # Jika listener sudah dihentikan, keluar dari loop stream.
                if not self._running:
                    # break menghentikan async for.
                    break

                # Mengubah enum FlightMode menjadi string tanpa prefix "FlightMode.".
                mode_str = str(fm).replace("FlightMode.", "")
                # Menyimpan mode terbang ke shared state.
                self.state["flight_mode"] = mode_str
                # Menyimpan timestamp update terakhir.
                self.state["last_update"] = time.time()

                # is_offboard bernilai True jika flight mode MAVSDK sama dengan OFFBOARD.
                is_offboard = (fm == FlightMode.OFFBOARD)
                # Menyimpan status OFFBOARD ke state agar modul lain bisa membacanya.
                self.state["in_offboard"] = is_offboard

                # Transition: enter OFFBOARD
                # Kondisi ini mendeteksi perubahan dari bukan OFFBOARD menjadi OFFBOARD.
                if is_offboard and not self._was_offboard:
                    # Log transisi masuk OFFBOARD.
                    logger.info("Flight mode → OFFBOARD: starting autonomy")
                    # Status sebelumnya diperbarui agar callback enter tidak dipanggil berulang-ulang.
                    self._was_offboard = True
                    # try menjaga callback autonomy tidak membuat stream flight mode berhenti.
                    try:
                        # Memanggil callback untuk memulai AutonomyController.
                        await self._on_offboard_enter()
                    # Jika callback gagal, error dicatat.
                    except Exception as e:
                        # Log error callback enter OFFBOARD.
                        logger.error("on_offboard_enter error: %s", e)

                # Transition: exit OFFBOARD
                # Kondisi ini mendeteksi perubahan dari OFFBOARD menjadi mode lain.
                elif not is_offboard and self._was_offboard:
                    # Log transisi keluar OFFBOARD beserta mode baru.
                    logger.info("Flight mode → %s: stopping autonomy", mode_str)
                    # Status sebelumnya diperbarui agar callback exit tidak dipanggil berulang-ulang.
                    self._was_offboard = False
                    # try menjaga callback stop autonomy tidak mematikan stream.
                    try:
                        # Memanggil callback untuk menghentikan AutonomyController.
                        await self._on_offboard_exit()
                    # Jika callback gagal, error dicatat.
                    except Exception as e:
                        # Log error callback exit OFFBOARD.
                        logger.error("on_offboard_exit error: %s", e)

        # CancelledError terjadi saat task stream dibatalkan oleh stop().
        except asyncio.CancelledError:
            # pass berarti pembatalan dianggap normal.
            pass
        # Exception umum dari stream flight mode dicatat.
        except Exception as e:
            # Log error membantu diagnosis masalah MAVSDK flight_mode stream.
            logger.error("_stream_flight_mode error: %s", e)

    # _stream_position membaca latitude, longitude, dan altitude relatif dari MAVSDK.
    async def _stream_position(self):
        # try menangkap error stream posisi.
        try:
            # async for menerima update posisi dari MAVSDK secara terus-menerus.
            async for pos in self.drone.telemetry.position():
                # Jika listener berhenti, keluar dari loop.
                if not self._running:
                    # break menghentikan stream posisi.
                    break
                # Memperbarui posisi di shared state dengan tipe float.
                self.state["position"] = {
                    # latitude_deg adalah latitude global dalam derajat.
                    "lat": float(pos.latitude_deg),
                    # longitude_deg adalah longitude global dalam derajat.
                    "lon": float(pos.longitude_deg),
                    # relative_altitude_m adalah altitude relatif terhadap home/takeoff dalam meter.
                    "alt": float(pos.relative_altitude_m),
                }
        # Pembatalan task dianggap normal saat listener dihentikan.
        except asyncio.CancelledError:
            # pass agar stop() tetap bersih.
            pass
        # Error stream posisi dicatat.
        except Exception as e:
            # Log error posisi untuk debugging koneksi/telemetri.
            logger.error("_stream_position error: %s", e)

    # _stream_attitude membaca orientasi drone dalam bentuk Euler angle.
    async def _stream_attitude(self):
        # try menangkap error attitude stream.
        try:
            # async for menerima update attitude roll, pitch, yaw dari MAVSDK.
            async for att in self.drone.telemetry.attitude_euler():
                # Jika listener berhenti, keluar dari loop.
                if not self._running:
                    # break menghentikan stream attitude.
                    break
                # Memperbarui attitude di shared state dengan satuan derajat.
                self.state["attitude"] = {
                    # roll_deg adalah kemiringan kiri-kanan drone.
                    "roll": float(att.roll_deg),
                    # pitch_deg adalah kemiringan depan-belakang drone.
                    "pitch": float(att.pitch_deg),
                    # yaw_deg adalah arah hadap drone.
                    "yaw": float(att.yaw_deg),
                }
        # CancelledError normal saat task dibatalkan.
        except asyncio.CancelledError:
            # pass membuat pembatalan tidak dianggap error.
            pass
        # Error attitude stream dicatat.
        except Exception as e:
            # Log error attitude untuk diagnosis.
            logger.error("_stream_attitude error: %s", e)

    # _stream_battery membaca persentase dan tegangan baterai dari MAVSDK.
    async def _stream_battery(self):
        # try menangkap error battery stream.
        try:
            # async for menerima update baterai dari drone.
            async for bat in self.drone.telemetry.battery():
                # Jika listener berhenti, keluar dari loop.
                if not self._running:
                    # break menghentikan stream baterai.
                    break
                # remaining_percent disimpan sebagai battery_pct.
                self.state["battery_pct"] = float(bat.remaining_percent)
                # Beberapa versi MAVSDK menyediakan voltage_v, sehingga perlu dicek dulu.
                if hasattr(bat, "voltage_v"):
                    # voltage_v disimpan sebagai tegangan baterai.
                    self.state["voltage"] = float(bat.voltage_v)
        # Pembatalan task adalah kondisi normal.
        except asyncio.CancelledError:
            # pass agar tidak memunculkan error saat stop().
            pass
        # Error stream baterai dicatat.
        except Exception as e:
            # Log error baterai untuk debugging telemetri.
            logger.error("_stream_battery error: %s", e)

    # _stream_gps membaca jumlah satelit dan tipe fix GPS.
    async def _stream_gps(self):
        # try menangkap error GPS stream.
        try:
            # async for menerima info GPS dari MAVSDK.
            async for gps in self.drone.telemetry.gps_info():
                # Jika listener berhenti, keluar dari loop.
                if not self._running:
                    # break menghentikan stream GPS.
                    break
                # fix_type diubah menjadi string agar mudah dicek kandungan "2D" atau "3D".
                fix_str = str(gps.fix_type)
                # fix diberi nilai numerik sederhana: 3 untuk 3D, 2 untuk 2D, 0 untuk belum fix.
                fix = 3 if "3D" in fix_str else (2 if "2D" in fix_str else 0)
                # Memperbarui informasi GPS di shared state.
                self.state["gps"] = {
                    # num_satellites adalah jumlah satelit GPS yang terbaca.
                    "satellites": int(gps.num_satellites),
                    # fix adalah kualitas fix GPS dalam bentuk angka sederhana.
                    "fix": fix,
                }
        # CancelledError normal saat task dihentikan.
        except asyncio.CancelledError:
            # pass agar pembatalan tidak dicatat sebagai error.
            pass
        # Error stream GPS dicatat.
        except Exception as e:
            # Log error GPS untuk diagnosis.
            logger.error("_stream_gps error: %s", e)

    # _stream_velocity membaca velocity drone pada frame NED.
    async def _stream_velocity(self):
        # try menangkap error velocity stream.
        try:
            # async for menerima update velocity_ned dari MAVSDK.
            async for vel in self.drone.telemetry.velocity_ned():
                # Jika listener berhenti, keluar dari loop.
                if not self._running:
                    # break menghentikan stream velocity.
                    break
                # Memperbarui velocity di shared state.
                self.state["velocity"] = {
                    # north_m_s adalah kecepatan ke arah utara dalam meter/detik.
                    "vx": float(vel.north_m_s),
                    # east_m_s adalah kecepatan ke arah timur dalam meter/detik.
                    "vy": float(vel.east_m_s),
                    # down_m_s adalah kecepatan ke bawah dalam meter/detik.
                    "vz": float(vel.down_m_s),
                }
        # CancelledError normal saat task dihentikan.
        except asyncio.CancelledError:
            # pass agar stop() tidak menghasilkan error.
            pass
        # Error stream velocity dicatat.
        except Exception as e:
            # Log error velocity untuk debugging.
            logger.error("_stream_velocity error: %s", e)

    # _stream_armed membaca status armed/disarmed drone.
    async def _stream_armed(self):
        # try menangkap error armed stream.
        try:
            # async for menerima status armed dari MAVSDK.
            async for armed in self.drone.telemetry.armed():
                # Jika listener berhenti, keluar dari loop.
                if not self._running:
                    # break menghentikan stream armed.
                    break
                # Menyimpan status armed sebagai boolean.
                self.state["armed"] = bool(armed)
        # CancelledError normal ketika stop() membatalkan task.
        except asyncio.CancelledError:
            # pass agar pembatalan task bersih.
            pass
        # Error stream armed dicatat.
        except Exception as e:
            # Log error armed untuk diagnosis.
            logger.error("_stream_armed error: %s", e)

    # _stream_heading membaca heading kompas drone dari MAVSDK.
    async def _stream_heading(self):
        # try menangkap error heading stream.
        try:
            # async for menerima update heading dari MAVSDK.
            async for heading in self.drone.telemetry.heading():
                # Jika listener berhenti, keluar dari loop.
                if not self._running:
                    # break menghentikan stream heading.
                    break
                # heading_deg disimpan sebagai float dalam satuan derajat.
                self.state["heading"] = float(heading.heading_deg)
        # CancelledError normal saat task dibatalkan oleh stop().
        except asyncio.CancelledError:
            # pass agar pembatalan tidak dianggap error.
            pass
        # Error stream heading dicatat.
        except Exception as e:
            # Log error heading untuk debugging.
            logger.error("_stream_heading error: %s", e)

    # _telemetry_broadcast_loop mengirim ringkasan state telemetri ke frontend pada 5 Hz.
    async def _telemetry_broadcast_loop(self):
        """Broadcast telemetry state to frontend at 5Hz."""
        while self._running:
            try:
                raw_state = self.state
                transformed = {
                    "connected": raw_state.get("connected", False),
                    "armed": raw_state.get("armed", False),
                    "mode": raw_state.get("flight_mode", "UNKNOWN"),
                    "battery": raw_state.get("battery_pct", 0.0),
                    "voltage": raw_state.get("voltage", 0.0),
                    "current": raw_state.get("current", 0.0),
                    "heading": raw_state.get("heading", 0.0),
                    "in_offboard": raw_state.get("in_offboard", False),
                    "last_update": raw_state.get("last_update", 0.0),
                }

                pos = raw_state.get("position", {})
                transformed["gps"] = {
                    "lat": pos.get("lat", 0.0),
                    "lon": pos.get("lon", 0.0),
                    "alt": pos.get("alt", 0.0),
                    "satellites": raw_state.get("gps", {}).get("satellites", 0),
                }

                vel = raw_state.get("velocity", {})
                vx = vel.get("vx", 0.0)
                vy = vel.get("vy", 0.0)
                ground_speed = (vx ** 2 + vy ** 2) ** 0.5
                transformed["velocity"] = {
                    "vx": vx,
                    "vy": vy,
                    "vz": vel.get("vz", 0.0),
                    "ground_speed": ground_speed,
                }

                att = raw_state.get("attitude", {})
                transformed["attitude"] = {
                    "roll": att.get("roll", 0.0),
                    "pitch": att.get("pitch", 0.0),
                    "yaw": att.get("yaw", 0.0),
                }

                self.socketio.emit("drone:status", transformed, namespace="/")

                await asyncio.sleep(0.2)
            # CancelledError terjadi saat task broadcast dibatalkan.
            except asyncio.CancelledError:
                # break keluar dari loop karena listener sedang dihentikan.
                break
            # Exception umum dicatat agar loop broadcast bisa mencoba lagi.
            except Exception as e:
                # Log error emit/transformasi telemetry.
                logger.error("telemetry broadcast error: %s", e)
                # Delay lebih panjang saat error agar tidak terjadi spam error cepat.
                await asyncio.sleep(1)
