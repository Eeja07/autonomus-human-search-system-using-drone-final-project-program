#!/usr/bin/env python3
# Shebang ini memberi tahu sistem Unix/Linux bahwa file dijalankan menggunakan interpreter Python 3.
"""
autonomy_standard.py
====================

Standard mode AutonomyController.
Mewarisi AutonomyBase dan menambahkan:
  - start() / stop()
  - _control_loop() dan _tick() — state machine 20Hz
  - _compute_tracking_command() — PID tracking orang

State machine standard:
  IDLE → SEARCHING → TRACKING → SEARCHING
                              → EMERGENCY (baterai habis)
"""

# asyncio digunakan untuk membuat task asynchronous dan mengatur loop kontrol tanpa memblokir program utama.
import asyncio
# logging digunakan untuk mencatat status, debug, warning, dan error selama autonomy standard berjalan.
import logging
# time digunakan untuk membandingkan umur data deteksi terhadap timestamp saat ini.
import time

# VelocityNedYaw adalah struktur perintah MAVSDK untuk mengirim velocity north/east/down dan yaw ke drone.
# pyrefly: ignore [missing-import]
from mavsdk.offboard import VelocityNedYaw
# AutonomyBase menyediakan state dasar, queue reader, emit event, dan helper; AutonomyState berisi enum state standard.
from .autonomy_base import AutonomyBase, AutonomyState

# logger dibuat untuk modul ini agar pesan log dapat ditelusuri berasal dari autonomy_standard.py.
logger = logging.getLogger(__name__)


# Class ini mengimplementasikan mode autonomy standard dengan mewarisi fasilitas dasar dari AutonomyBase.
class AutonomyStandard(AutonomyBase):
    """
    Standard mode controller.

    Dipanggil oleh MAVSDKListener saat flight mode masuk OFFBOARD.
    Menjalankan control loop 20Hz yang membaca detection queue
    dan mengirim perintah kecepatan ke drone.
    """

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    # start adalah API untuk mengaktifkan autonomy standard, biasanya saat flight mode masuk OFFBOARD.
    async def start(self):
        """Masuk autonomy. Dipanggil saat flight mode menjadi OFFBOARD."""
        # Jika autonomy sudah berjalan, fungsi tidak membuat task baru agar tidak terjadi double control loop.
        if self._running:
            # Warning dicatat karena start dipanggil saat sistem sudah aktif.
            logger.warning("AutonomyController sudah berjalan")
            # Return menghentikan fungsi karena tidak ada perubahan yang perlu dilakukan.
            return

        # Log info menandakan proses startup autonomy dimulai.
        logger.info("AutonomyController: starting")
        # _running menjadi True agar detection reader dan control loop boleh berjalan.
        self._running = True
        # Menyimpan status autonomy ke state_dict agar modul lain/frontend dapat mengetahui sistem aktif.
        self.state_dict["autonomy_running"] = True  # <--- TAMBAHKAN INI
        # State awal standard mode adalah SEARCHING karena sistem mulai dengan mencari target person.
        self._state = AutonomyState.SEARCHING
        # _target_alt mengambil altitude saat ini sebagai altitude target; default 2.0 jika data posisi belum tersedia.
        self._target_alt = self.state_dict.get("position", {}).get("alt", 2.0)
        # _search_yaw mengambil yaw saat ini sebagai arah awal pencarian.
        self._search_yaw = self.state_dict.get("attitude", {}).get("yaw", 0.0)
        # Reset smoothing target X karena autonomy baru dimulai dan belum ada target stabil.
        self._smooth_cx = None
        # Reset smoothing target Y karena autonomy baru dimulai dan belum ada target stabil.
        self._smooth_cy = None
        # Menghapus target lock lama agar tracking standard mulai dari kondisi bersih.
        self._clear_person_target_lock()

        # Mulai pembaca queue deteksi
        # Task ini membaca detection_queue YOLO secara asynchronous dan menyimpan hasil terbaru ke _latest_detection.
        self._detection_reader_task = asyncio.create_task(self._detection_reader())

        # Mulai control loop
        # Task ini menjalankan state machine kontrol drone secara periodik sesuai CONTROL_HZ.
        self._task = asyncio.create_task(self._control_loop())

        # Mengirim event ke frontend/client bahwa autonomy sudah dimulai.
        self._emit("autonomy:started", {"state": self._state.name})
        # Log info mengonfirmasi state awal setelah startup.
        logger.info("AutonomyController: STARTED, state=SEARCHING")

        # >>> TAMBAHKAN BARIS INI AGAR SCOUT LANGSUNG MENYALA <<<
        # Mengecek apakah instance juga memiliki method start_scout, karena method ini ada pada controller gabungan scout.
        if hasattr(self, 'start_scout'):
            # Fungsi lokal asynchronous ini membungkus pemanggilan start_scout agar bisa dijalankan sebagai task background.
            async def _auto_start_scout():
                # Memulai scout mode secara otomatis setelah standard autonomy aktif.
                res = await self.start_scout()
                # Mencatat hasil pemanggilan auto-start scout untuk debugging.
                logger.info("Auto-start scout trigger result: %s", res)
            
            # Menjadwalkan auto-start scout tanpa menunggu selesai agar start() tidak tertahan.
            asyncio.create_task(_auto_start_scout())

    # stop adalah API untuk mematikan autonomy standard, biasanya saat flight mode keluar dari OFFBOARD.
    async def stop(self):
        """Stop autonomy. Dipanggil saat flight mode keluar dari OFFBOARD."""
        # Jika autonomy tidak sedang berjalan, tidak ada yang perlu dihentikan.
        if not self._running:
            # Return langsung menjaga stop() tetap aman dipanggil berkali-kali.
            return

        # Log info menandakan proses shutdown autonomy dimulai.
        logger.info("AutonomyController: stopping")
        # _running False membuat detection reader dan control loop berhenti pada iterasi berikutnya.
        self._running = False
        # Status autonomy di state_dict diperbarui agar modul lain tahu autonomy sudah berhenti.
        self.state_dict["autonomy_running"] = False
        # State dikembalikan ke IDLE karena standard autonomy tidak aktif.
        self._state = AutonomyState.IDLE
        # Menghapus target lock agar target lama tidak terbawa saat autonomy dimulai lagi.
        self._clear_person_target_lock()

        # Jika task pembaca deteksi sedang ada, task perlu dibatalkan.
        if self._detection_reader_task:
            # Mengirim sinyal cancel ke task detection reader.
            self._detection_reader_task.cancel()
            # Menunggu pembatalan selesai; return_exceptions=True mencegah CancelledError memutus stop().
            await asyncio.gather(self._detection_reader_task, return_exceptions=True)

        # Jika task control loop sedang ada, task perlu dibatalkan.
        if self._task:
            # Mengirim sinyal cancel ke control loop.
            self._task.cancel()
            # Menunggu control loop selesai dibatalkan secara aman.
            await asyncio.gather(self._task, return_exceptions=True)

        # Mengirim event bahwa autonomy berhenti karena flight mode keluar dari OFFBOARD.
        self._emit("autonomy:stopped", {"reason": "flight mode exited OFFBOARD"})
        # Log info mengonfirmasi autonomy sudah berhenti.
        logger.info("AutonomyController: STOPPED")

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    # _control_loop adalah loop utama standard mode yang berjalan pada frekuensi CONTROL_HZ.
    async def _control_loop(self):
        # Log awal loop menyebutkan frekuensi kontrol yang dipakai.
        logger.info("Autonomy control loop started at %dHz", self.CONTROL_HZ)
        # tick menghitung jumlah iterasi loop untuk kebutuhan logging periodik dan state machine.
        tick = 0

        # Loop berjalan selama _running True.
        while self._running:
            # t_start menyimpan waktu mulai tick untuk menghitung durasi eksekusi.
            t_start = asyncio.get_event_loop().time()
            # Counter tick dinaikkan setiap iterasi.
            tick += 1

            # try melindungi loop agar error satu tick tidak langsung mematikan seluruh task.
            try:
                # Menjalankan satu siklus state machine kontrol.
                await self._tick(tick)
            # CancelledError muncul saat task control loop dibatalkan oleh stop().
            except asyncio.CancelledError:
                # break keluar dari while loop ketika task diminta berhenti.
                break
            # Exception umum dicatat agar masalah tick bisa dianalisis.
            except Exception as e:
                # Log error berisi pesan exception dari tick.
                logger.error("Control loop tick error: %s", e)

            # elapsed adalah waktu yang sudah dipakai untuk menjalankan satu tick.
            elapsed = asyncio.get_event_loop().time() - t_start
            # sleep_t menjaga periode loop tetap mendekati CONTROL_PERIOD dengan mengurangi waktu proses tick.
            sleep_t = max(0.0, self.CONTROL_PERIOD - elapsed)
            # Sleep asynchronous menjaga loop 20Hz tanpa memblokir event loop lain.
            await asyncio.sleep(sleep_t)

        # Log akhir loop setelah _running False atau task dibatalkan.
        logger.info("Autonomy control loop stopped")

    # _tick menjalankan satu iterasi kontrol: cek safety, hitung altitude, pilih state, dan kirim command.
    async def _tick(self, tick: int):
        """
        Satu siklus kontrol. Membaca state, menjalankan state machine,
        dan mengirim perintah ke drone.

        Perintah dikirim setiap tick (20Hz) — wajib untuk protokol
        PX4 OFFBOARD agar tidak timeout.

        FIX: vz sekarang selalu dihitung dan diteruskan.
        Saat scout aktif, _scout_main_loop-lah yang pakai vz-nya sendiri.
        Standard tick hanya skip kirim perintah, tapi tidak skip hitung vz.
        """
        # tstate adalah referensi singkat ke state_dict yang berisi telemetri drone dan status sistem.
        tstate = self.state_dict
        # === BARIS SEMENTARA UNTUK SCREENSHOT BAB 3 ===
        # Log periodik setiap CONTROL_HZ tick; karena CONTROL_HZ=20, ini sekitar satu kali per detik.
        if tick % self.CONTROL_HZ == 0:  # Log setiap 1 detik
            # Log ini menampilkan state machine, flight mode, baterai, altitude, dan status scout untuk dokumentasi/debug.
            logger.info("STATE MACHINE → state=%s | mode=%s | battery=%.1f%% | alt=%.1fm | scout=%s",
                # Data log diambil dari property current_state dan telemetri terbaru di state_dict.
                self.current_state, tstate.get("flight_mode"), 
                # battery_pct dan altitude diberi default agar logging tetap aman walau data belum lengkap.
                tstate.get("battery_pct", 0), tstate.get("position", {}).get("alt", 0),
                # Teks ACTIVE/INACTIVE menunjukkan apakah scout mode sedang mengambil alih perintah drone.
                "ACTIVE" if self._scout_mode else "INACTIVE")
        # === AKHIR BARIS SEMENTARA ===
        # --- Safety check baterai (setiap tick) ---
        # Mengambil persentase baterai dari telemetri; default 100.0 jika belum ada data.
        battery = tstate.get("battery_pct", 100.0)
        # Jika baterai di bawah ambang keselamatan, sistem masuk kondisi emergency.
        if battery < self.BATTERY_RTL_THRESHOLD:
            # Perintah emergency hanya dipicu sekali saat state belum EMERGENCY.
            if self._state != AutonomyState.EMERGENCY:
                # Mencatat warning karena baterai kritis membutuhkan tindakan RTL.
                logger.warning("Baterai kritis %.1f%% — pre-RTL climb + RTL", battery)
                # State standard dipindah ke EMERGENCY.
                self._state = AutonomyState.EMERGENCY
                # Mengirim event emergency ke frontend/client.
                self._emit("autonomy:emergency", {"reason": "low battery", "battery": battery})
                # === PRE-RTL SAFETY CLIMB ===
                # Barometer drift: climb paksa sebelum RTL agar tidak crash di ketinggian rendah.
                cur_yaw = tstate.get("attitude", {}).get("yaw", 0.0)
                logger.info("STD PRE-RTL → climbing %.1fs at vz=%.1f", self.SCOUT_PRE_RTL_CLIMB_S, self.SCOUT_PRE_RTL_VZ)
                climb_deadline = asyncio.get_event_loop().time() + self.SCOUT_PRE_RTL_CLIMB_S
                while asyncio.get_event_loop().time() < climb_deadline:
                    try:
                        await self.drone.offboard.set_velocity_ned(
                            VelocityNedYaw(0.0, 0.0, self.SCOUT_PRE_RTL_VZ, cur_yaw)
                        )
                    except Exception:
                        break
                    await asyncio.sleep(self.CONTROL_PERIOD)
                # Stop sebelum handoff ke RTL.
                try:
                    await self.drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, 0.0, cur_yaw)
                    )
                except Exception:
                    pass
                logger.info("STD PRE-RTL → climb complete, sending RTL")
                # try digunakan agar kegagalan RTL tercatat tanpa membuat loop crash.
                try:
                    # Memerintahkan drone return_to_launch melalui MAVSDK action.
                    await self.drone.action.return_to_launch()
                # Jika perintah RTL gagal, error dicatat.
                except Exception as e:
                    # Log error menyimpan detail kegagalan RTL.
                    logger.error("RTL gagal: %s", e)
            # Return menghentikan tick agar tidak ada perintah tracking/searching setelah kondisi emergency.
            return

        # Mengambil altitude aktual dari telemetri posisi.
        alt = tstate.get("position", {}).get("alt", 0.0)
        # Mengambil yaw aktual dari telemetri attitude.
        yaw = tstate.get("attitude", {}).get("yaw", 0.0)

        # target_alt memakai nilai target yang disimpan; jika belum ada, gunakan altitude saat ini.
        target_alt = self._target_alt if self._target_alt is not None else alt
        # alt_error adalah selisih altitude target dan altitude aktual.
        alt_error = target_alt - alt
        # vz adalah koreksi velocity vertikal, dibatasi agar tidak melebihi MAX_VZ.
        vz = max(-self.MAX_VZ, min(self.MAX_VZ, -self.KP_ALT * alt_error))

        # Jika scout mode aktif, scout loop yang mengirim perintah
        # Ketika scout aktif, standard tick tidak mengirim command agar tidak konflik dengan _scout_main_loop.
        if self._scout_mode:
            # Return setelah hitung vz karena command akan dikirim oleh mode scout.
            return

        # --- Standard state machine ---
        # detection adalah hasil YOLO terbaru yang dibaca oleh _detection_reader.
        detection = self._latest_detection
        # has_person menentukan apakah ada deteksi person yang masih cukup baru untuk dipakai tracking.
        has_person = (
            # Harus ada object detection terlebih dahulu.
            detection is not None
            # List detections harus berisi minimal satu item.
            and len(detection.detections) > 0
            # Ubah dari 0.5 menjadi 2.5 detik (atau 3.0 detik)
            # Deteksi dianggap valid hanya jika umur timestamp kurang dari 2.5 detik.
            and (time.time() - detection.timestamp) < 2.5 
        )

        # Jika ada target person valid, mode standard masuk TRACKING.
        if has_person:
            # State diubah menjadi TRACKING.
            self._state = AutonomyState.TRACKING
            # _search_yaw disinkronkan dengan yaw saat ini agar searching berikutnya mulai dari arah terakhir.
            self._search_yaw = yaw
            # Menghitung perintah velocity dan yaw untuk mengikuti target.
            vn, ve, vz_track, yaw_cmd = self._compute_tracking_command(
                # Input berupa daftar deteksi, yaw saat ini, altitude saat ini, dan koreksi vertical speed.
                detection.detections, yaw, alt, vz
            )
            # Mengirim perintah velocity NED dan yaw ke drone melalui MAVSDK offboard.
            await self.drone.offboard.set_velocity_ned(
                # VelocityNedYaw berisi vn, ve, vz, dan yaw_cmd sebagai command tracking.
                VelocityNedYaw(vn, ve, vz_track, yaw_cmd)
            )
            # Logging dan event tracking dikirim periodik setiap satu detik.
            if tick % self.CONTROL_HZ == 0:
                # Log debug memperlihatkan yaw command dan velocity vertical saat tracking.
                logger.debug("TRACKING: yaw_cmd=%.1f vz=%.2f", yaw_cmd, vz_track)
                # Event tracking memberi status dan informasi altitude ke frontend/client.
                self._emit("autonomy:tracking", {
                    # state menjelaskan bahwa controller berada pada mode TRACKING.
                    "state": "TRACKING",
                    # yaw adalah yaw command yang dikirim ke drone.
                    "yaw": yaw_cmd,
                    # altitude berisi target, aktual, error, dan koreksi vz untuk monitoring.
                    "altitude": {"target": target_alt, "current": alt, "error": alt_error, "vz_correction": vz_track},
                })
        # Jika tidak ada deteksi person valid, mode standard masuk SEARCHING.
        else:
            # State diubah menjadi SEARCHING.
            self._state = AutonomyState.SEARCHING
            # Target lock dihapus karena target tidak terlihat atau data sudah terlalu lama.
            self._clear_person_target_lock()
            # Yaw pencarian ditambah berdasarkan rate pencarian dan periode kontrol.
            self._search_yaw += self.SCOUT_SEARCH_YAW_RATE * self.CONTROL_PERIOD
            # Jika yaw melewati 360 derajat, dikurangi 360 agar tetap dalam satu putaran.
            if self._search_yaw >= 360.0:
                # Normalisasi yaw pencarian kembali ke rentang 0 sampai 360.
                self._search_yaw -= 360.0

            # Mengirim perintah drone diam horizontal, koreksi altitude, dan berputar yaw untuk mencari target.
            await self.drone.offboard.set_velocity_ned(
                # VelocityNedYaw dengan vn=0 dan ve=0 berarti tidak bergerak horizontal saat searching.
                VelocityNedYaw(0.0, 0.0, vz, self._search_yaw)
            )
            # Logging dan event searching dikirim periodik setiap satu detik.
            if tick % self.CONTROL_HZ == 0:
                # Log debug memperlihatkan yaw pencarian dan koreksi vertical speed.
                logger.debug("SEARCHING: search_yaw=%.1f vz=%.2f", self._search_yaw, vz)
                # Event searching memberi status yaw pencarian dan altitude ke frontend/client.
                self._emit("autonomy:searching", {
                    # search_yaw adalah arah yaw yang sedang dipakai untuk scan target.
                    "search_yaw": self._search_yaw,
                    # altitude berisi data kontrol altitude untuk monitoring.
                    "altitude": {"target": target_alt, "current": alt, "error": alt_error, "vz_correction": vz},
                })

    # ------------------------------------------------------------------
    # Tracking computation
    # ------------------------------------------------------------------

    # Fungsi ini menghitung command tracking berdasarkan target person terpilih dan error posisi pada frame kamera.
    def _compute_tracking_command(
        # detections adalah list hasil YOLO, current_yaw/current_alt adalah telemetri saat ini, vz_alt adalah koreksi altitude.
        self, detections, current_yaw: float, current_alt: float, vz_alt: float
    ):
        """
        PID tracking: pilih target person stabil, hitung perintah kecepatan.
        Return: (vn, ve, vz, yaw_cmd)
        """
        # Memilih target person stabil menggunakan helper target lock dari AutonomyHelpers.
        target = self._select_person_target(
            # frame_width dan frame_height dipakai untuk memahami posisi bbox dalam frame.
            detections, self.frame_width, self.frame_height, time.time()
        )

        # Jika tidak ada target valid, drone tidak bergerak horizontal dan yaw dipertahankan.
        if target is None:
            # Output tetap membawa vz_alt agar altitude masih dapat dikoreksi.
            return 0.0, 0.0, vz_alt, current_yaw

        # Jika smoothing belum punya nilai awal, langsung isi dengan pusat target normalized.
        if self._smooth_cx is None:
            # _smooth_cx menyimpan posisi X target yang sudah dihaluskan.
            self._smooth_cx = target["cx_norm"]
            # _smooth_cy menyimpan posisi Y target yang sudah dihaluskan.
            self._smooth_cy = target["cy_norm"]
        # Jika smoothing sudah ada, perbarui memakai exponential moving average.
        else:
            # EMA untuk X mengurangi noise bounding box agar yaw command tidak terlalu bergetar.
            self._smooth_cx = self._alpha * target["cx_norm"] + (1 - self._alpha) * self._smooth_cx
            # EMA untuk Y mengurangi noise posisi vertikal target pada frame.
            self._smooth_cy = self._alpha * target["cy_norm"] + (1 - self._alpha) * self._smooth_cy

        # cx_error adalah selisih posisi X target terhadap tengah frame normalized 0.5.
        cx_error = self._smooth_cx - 0.5
        # yaw_cmd adalah yaw target yang dikoreksi proporsional terhadap error horizontal.
        yaw_cmd = current_yaw + self.KP_YAW * cx_error

        # cy_error adalah selisih posisi Y target terhadap tengah frame normalized 0.5.
        cy_error = self._smooth_cy - 0.5
        # vn adalah velocity north/maju sederhana yang dihitung dari error vertikal frame.
        vn = self.KP_FORWARD * cy_error
        # vn dibatasi agar tidak melebihi kecepatan maksimum mode standard.
        vn = max(-self.MAX_VEL, min(self.MAX_VEL, vn))

        # Output tuple berisi velocity north, velocity east, velocity vertical, dan yaw command.
        return vn, 0.0, vz_alt, yaw_cmd
