#!/usr/bin/env python3
# Shebang di atas memberi tahu sistem Unix/Linux bahwa file ini dijalankan memakai interpreter Python 3.
"""
autonomy_scout.py
=================
Logika AutonomyController dan Machine State Scout.
"""

# asyncio dipakai untuk menjalankan proses asynchronous, misalnya loop scout dan perintah drone tanpa memblokir program utama.
import asyncio
# logging dipakai untuk mencatat informasi, peringatan, dan error selama mode scout berjalan.
import logging
# math dipakai untuk operasi matematika seperti cos, sin, radians, dan sqrt pada perhitungan arah/kecepatan.
import math
# time dipakai untuk cek freshness deteksi YOLO di state scout.
import time
# datetime dipakai untuk membuat timestamp saat mengirim event perubahan state scout.
from datetime import datetime
# Dict dipakai sebagai type hint bahwa fungsi mengembalikan dictionary.
from typing import Dict
# VelocityNedYaw adalah tipe perintah MAVSDK untuk mengirim kecepatan dalam frame NED dan yaw ke Pixhawk.
# pyrefly: ignore [missing-import]
from mavsdk.offboard import VelocityNedYaw

# AutonomyStandard adalah class dasar yang menyediakan fungsi/atribut standar autonomy seperti state drone, helper kalkulasi, dan event emit.
from .autonomy_standard import AutonomyStandard
# ScoutState adalah enum/state yang merepresentasikan tahapan mode scout seperti IDLE, SCAN, APPROACH, DOCUMENT, RETURN_HOME.
from .autonomy_base import ScoutState

# logger dibuat untuk file ini agar pesan log memiliki nama modul sesuai __name__.
logger = logging.getLogger(__name__)
# SCOUT_LOOP_PERIOD menentukan periode loop utama scout dalam detik; 0.05 detik berarti sekitar 20 Hz.
SCOUT_LOOP_PERIOD = 0.05

# Class ini mewarisi AutonomyStandard sehingga semua kemampuan standar autonomy tetap tersedia dan ditambah mode Scout.
class AutonomyController(AutonomyStandard):
    """
    Full AutonomyController yang menggabungkan Standard Mode dan Scout Mode.
    """

    # Fungsi asynchronous untuk memulai mode scout dan mengembalikan status dalam bentuk dictionary.
    async def start_scout(self) -> Dict:
        # Jika scout sudah aktif, fungsi langsung gagal agar tidak membuat dua loop scout berjalan bersamaan.
        if self._scout_mode: return {"success": False, "error": "Scout aktif"}
        # Jika autonomy utama belum berjalan, scout tidak boleh dimulai karena kontrol drone belum siap.
        if not self._running: return {"success": False, "error": "Autonomy mati"}

        # Mengambil persentase baterai dari state_dict; jika tidak ada data, diasumsikan 100.0 persen.
        battery = self.state_dict.get("battery_pct", 100.0)
        # Jika baterai di bawah ambang RTL, scout ditolak demi keselamatan penerbangan.
        if battery < self.BATTERY_RTL_THRESHOLD:
            # Dictionary ini menjadi output error yang dapat dikirim ke pemanggil/API.
            return {"success": False, "error": "Baterai rendah"}

        # Mengambil altitude saat ini dari state posisi drone dan mengubahnya ke float.
        alt = float(self.state_dict.get("position", {}).get("alt", 0.0))
        # Mencatat bahwa mode scout sedang dimulai.
        logger.info("Scout mode: starting")
        
        # Menandai bahwa mode scout aktif sehingga loop utama scout boleh berjalan.
        self._scout_mode = True
        # Menyiapkan daftar koordinat target/person yang sudah didokumentasikan agar tidak diulang.
        self._scout_visited_coords = []
        # Reset flag hysteresis baterai agar scout baru mulai bersih.
        self._battery_rtl_triggered = False
        # Flag ini menentukan apakah setelah kembali ke home scout drone harus berhenti/hold.
        self._scout_return_then_hold = False
        # Menyimpan altitude target scout berdasarkan altitude saat scout dimulai.
        # Clamp ke MIN_SAFE_ALTITUDE_M: barometer drift sering membuat relative_altitude_m
        # lebih tinggi dari AGL nyata (misal log 1.6m padahal drone <60cm).
        # Tanpa clamp, altitude hold bisa mempertahankan ketinggian yang terlalu rendah.
        self._target_altitude_scout = max(alt, self.MIN_SAFE_ALTITUDE_M)
        # Menyimpan posisi home khusus scout berdasarkan posisi drone saat mode scout dimulai.
        self._scout_home_position = {
            # Latitude home scout diambil dari state posisi; default 0 jika data belum tersedia.
            "lat": float(self.state_dict.get("position", {}).get("lat", 0)),
            # Longitude home scout diambil dari state posisi; default 0 jika data belum tersedia.
            "lon": float(self.state_dict.get("position", {}).get("lon", 0)),
            # Altitude home scout memakai altitude target yang sudah disimpan.
            "alt": self._target_altitude_scout
        }
        # Variabel smoothing koordinat tengah target pada frame kamera direset karena scout baru dimulai.
        self._smooth_cx, self._smooth_cy = None, None
        # Waktu kehilangan target direset karena belum ada target yang sedang dilacak.
        self._target_lost_time = None
        # Menghapus lock target person lama agar pemilihan target scout dimulai dari kondisi bersih.
        self._clear_person_target_lock()
        # Reset state PD controller, konfirmasi multi-frame, dan hysteresis movement.
        self._prev_err_x = 0.0
        self._approach_confirm_count = 0
        self._document_confirm_count = 0
        self._last_movement_zone = "idle"
        # Reset displacement tracking state.
        self._scout_displacement_start_pos = None
        self._scout_displacement_start_time = None

        # Membuat task asynchronous untuk menjalankan loop utama scout di background.
        self._scout_task = asyncio.create_task(self._scout_main_loop())
        # Mengirim event ke sistem lain/frontend bahwa scout sudah dimulai beserta data home dan baterai.
        self._emit("scout:started", {"home": self._scout_home_position, "battery": battery})
        # Mengembalikan output sukses ke pemanggil, termasuk posisi home scout.
        return {"success": True, "message": "Scout started", "home": self._scout_home_position}

    # Fungsi asynchronous untuk menghentikan mode scout yang sedang aktif.
    async def stop_scout(self) -> Dict:
        # Jika scout tidak aktif, fungsi mengembalikan error karena tidak ada proses scout yang perlu dihentikan.
        if not self._scout_mode: return {"success": False, "error": "Tidak aktif"}
        
        # Mencatat bahwa proses penghentian scout sedang dilakukan.
        logger.info("Scout mode: stopping")
        # Mengubah flag menjadi False agar while loop di _scout_main_loop berhenti.
        self._scout_mode = False
        # Mengembalikan state scout ke IDLE sebagai kondisi tidak aktif.
        self._scout_state = ScoutState.IDLE
        # Membersihkan target lock agar target dari scout sebelumnya tidak terbawa.
        self._clear_person_target_lock()
        # Jika task scout pernah dibuat, task tersebut perlu dibatalkan.
        if self._scout_task:
            # Mengirim sinyal cancel ke task loop scout.
            self._scout_task.cancel()
            # Menunggu task benar-benar selesai; return_exceptions=True mencegah CancelledError merusak alur stop.
            await asyncio.gather(self._scout_task, return_exceptions=True)

        # TEST_F: Log summary misi saat stop manual agar data selalu tercetak.
        self._log_test_f_summary("stop_manual")
        # Mengembalikan ringkasan jumlah koordinat/person yang ditemukan selama scout aktif.
        return {"success": True, "summary": {"people_found": len(self._scout_visited_coords)}}

    # Fungsi helper untuk mengatur ulang proses scanning, opsional memutar arah awal menjauh dari target lama.
    def _reset_scout_scan(self, rotate_away=False):
        # Mengubah state scout menjadi SCAN agar drone kembali mencari target.
        self._scout_state = ScoutState.SCAN
        # Mengatur ulang jumlah derajat rotasi scan yang sudah dilakukan.
        self._scout_scan_rotated = 0.0
        # Mengambil yaw drone saat ini dari state attitude; default 0 jika belum ada data.
        cur_yaw = self.state_dict.get("attitude", {}).get("yaw", 0.0)
        # Jika rotate_away True, yaw scan dimulai 180 derajat dari yaw sekarang; jika False tetap memakai yaw sekarang.
        self._scout_scan_yaw = (cur_yaw + 180.0) % 360.0 if rotate_away else cur_yaw
        # Reset smoothing koordinat target karena proses scan baru tidak boleh memakai data target lama.
        self._smooth_cx, self._smooth_cy = None, None
        # Menghapus lock target person agar target berikutnya bisa dipilih ulang.
        self._clear_person_target_lock()
        # Reset konfirmasi multi-frame dan D-term state saat kembali ke scan.
        self._approach_confirm_count = 0
        self._document_confirm_count = 0
        self._prev_err_x = 0.0
        self._last_movement_zone = "idle"

    # Fungsi helper untuk mengirim informasi perubahan state scout ke sistem event.
    def _emit_scout_state(self, state: str, extra: dict = None):
        # Payload adalah data yang dikirim ke listener/frontend tentang status scout saat ini.
        payload = {
            # state berisi nama state yang sedang dilaporkan dan timestamp memberi waktu kejadian dalam format ISO.
            "state": state, "timestamp": datetime.now().isoformat(),
            # visited_count memberi jumlah lokasi/person yang sudah didokumentasikan.
            "visited_count": len(self._scout_visited_coords),
            # scan_progress_deg memberi progres rotasi scan dalam derajat.
            "scan_progress_deg": float(self._scout_scan_rotated)
        }
        # Jika ada data tambahan, data tersebut digabungkan ke payload utama.
        if extra: payload.update(extra)
        # Mengirim event scout:state_changed agar komponen lain mengetahui perubahan state.
        self._emit("scout:state_changed", payload)

    # Handler state SCAN; drone berputar mencari deteksi person dari kamera/YOLO.
    async def _scout_state_scan(self, vz: float, loop_count: int):
        # Mengambil daftar deteksi terbaru jika tersedia; jika tidak ada, gunakan list kosong.
        dets = self._latest_detection.detections if self._latest_detection and hasattr(self._latest_detection, "detections") else []
        # Cek freshness deteksi: abaikan jika terlalu lama (Issue #4 dari analisis)
        if self._latest_detection and hasattr(self._latest_detection, "timestamp"):
            if (time.time() - self._latest_detection.timestamp) > self.SCOUT_DETECTION_MAX_AGE_S:
                dets = []
        # Memilih target person paling relevan berdasarkan deteksi dan ukuran frame kamera.
        target = self._select_person_target(dets, self.frame_width, self.frame_height)

        # Multi-frame confirmation: target harus terdeteksi beberapa frame berturut-turut
        # sebelum transisi ke APPROACH, mencegah false positive 1 frame (Issue #7).
        if target is not None:
            self._approach_confirm_count += 1
            if self._approach_confirm_count >= self.SCOUT_APPROACH_CONFIRM_FRAMES:
                self._scout_state = ScoutState.APPROACH
                self._target_lost_time, self._smooth_cx, self._smooth_cy = None, None, None
                self._prev_err_x = 0.0
                self._last_movement_zone = "idle"
                self._approach_start_time = asyncio.get_event_loop().time()
                self._approach_confirm_count = 0
                # TEST_A: Log transisi SCAN→APPROACH.
                self._log_fsm_transition("SCAN", "APPROACH")
                self._emit_scout_state("APPROACH", {"message": "Mendekat"})
                return
        else:
            # Reset counter jika target hilang di antara frame konfirmasi.
            self._approach_confirm_count = 0

        # Menghitung perubahan yaw per loop berdasarkan yaw rate pencarian dan periode loop.
        dyaw = self.SCOUT_SEARCH_YAW_RATE * SCOUT_LOOP_PERIOD
        # Memperbarui yaw scan dan memakai modulo 360 agar nilai yaw tetap dalam rentang satu putaran.
        self._scout_scan_yaw = (self._scout_scan_yaw + dyaw) % 360.0
        # Menambah total rotasi scan yang sudah ditempuh.
        self._scout_scan_rotated += dyaw

        # Mengirim perintah offboard ke drone: tidak bergerak horizontal, tetap koreksi vertikal vz, dan yaw mengikuti scan.
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, vz, self._scout_scan_yaw))
        # Telemetry snapshot ~5Hz selama SCAN untuk analisis pergerakan dan altitude hold.
        self._log_telemetry_snapshot(
            scout_state="SCAN",
            cmd_vz=vz,
        )

        # Jika rotasi sudah mencapai satu putaran penuh, progres scan direset.
        if self._scout_scan_rotated >= 360.0:
            # Reset progres rotasi agar siklus scan berikutnya mulai dari 0 derajat lagi.
            self._scout_scan_rotated = 0.0
            # Mengirim event bahwa scan 360 derajat selesai dan dilanjutkan.
            self._emit_scout_state("SCAN", {"message": "Selesai 360, lanjut"})

    # Handler state APPROACH; drone mengarahkan dan mendekati target person yang terdeteksi.
    async def _scout_state_approach(self, vz: float, loop_count: int):
        # Mengambil waktu loop event saat ini untuk perhitungan timeout.
        cur_time = asyncio.get_event_loop().time()
        # Mengambil yaw drone saat ini sebagai referensi arah.
        cur_yaw = self.state_dict.get("attitude", {}).get("yaw", 0.0)

        # Jika durasi approach melebihi batas timeout, proses pendekatan dibatalkan.
        if self._approach_start_time and (cur_time - self._approach_start_time) > self.SCOUT_APPROACH_TIMEOUT_S:
            # TEST_A: Log transisi APPROACH→SCAN (timeout).
            self._log_fsm_transition("APPROACH", "SCAN", "timeout")
            self._reset_scout_scan()
            self._emit_scout_state("SCAN", {"message": "Timeout"})
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, vz, cur_yaw))
            return

        # Mengambil daftar deteksi terbaru; jika tidak ada data deteksi, gunakan list kosong.
        dets = self._latest_detection.detections if self._latest_detection and hasattr(self._latest_detection, "detections") else []
        # Cek freshness deteksi: abaikan jika timestamp terlalu lama (Issue #4 dari analisis).
        if self._latest_detection and hasattr(self._latest_detection, "timestamp"):
            if (time.time() - self._latest_detection.timestamp) > self.SCOUT_DETECTION_MAX_AGE_S:
                dets = []
        # Memilih target person dari deteksi terbaru untuk dilacak selama approach.
        target = self._select_person_target(dets, self.frame_width, self.frame_height)

        # Jika target tidak terlihat pada frame saat ini, masuk logika kehilangan target.
        if target is None:
            # Jika ini pertama kali target hilang, simpan waktu hilangnya target.
            if not self._target_lost_time: self._target_lost_time = cur_time
            # Timeout kehilangan target lebih toleran (Issue #9: dari 2s ke parameter yang dikonfigurasi).
            if (cur_time - self._target_lost_time) > self.SCOUT_TARGET_LOST_TIMEOUT_S:
                # TEST_A: Log transisi APPROACH→SCAN (target lost).
                self._log_fsm_transition("APPROACH", "SCAN", "lost")
                self._reset_scout_scan()
                self._emit_scout_state("SCAN", {"message": "Hilang"})
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, vz, cur_yaw))
            return
        
        # Jika target kembali terlihat, reset indikator waktu hilang.
        self._target_lost_time = None
        # bbox_ratio adalah tinggi bounding box target dibanding tinggi frame; makin besar berarti target makin dekat di kamera.
        bbox_ratio = target["height"] / self.frame_height
        # cx adalah koordinat pusat bounding box target pada sumbu X frame.
        cx = target["cx"]
        # cy adalah koordinat pusat bounding box target pada sumbu Y frame.
        cy = target["cy"]

        # Adaptive alpha: lebih responsif saat target dekat karena bbox bergerak cepat (Issue #3).
        effective_alpha = 0.30 if bbox_ratio > 0.2 else self._alpha
        # Smoothing cx meredam noise deteksi; nilai pertama langsung dipakai, berikutnya memakai exponential moving average.
        self._smooth_cx = cx if self._smooth_cx is None else effective_alpha * cx + (1 - effective_alpha) * self._smooth_cx
        # Smoothing cy meredam lonjakan posisi vertikal target pada frame kamera.
        self._smooth_cy = cy if self._smooth_cy is None else effective_alpha * cy + (1 - effective_alpha) * self._smooth_cy
        
        # Menghitung error posisi target terhadap tengah frame kamera.
        err = self._calculate_frame_error(self._smooth_cx, self._smooth_cy, self.frame_width, self.frame_height)
        # DOCUMENT trigger: hanya bergantung pada err_x (horizontal centering) dan bbox_ratio.
        # err_y TIDAK dipakai sebagai syarat keras karena kamera fixed forward-facing low altitude:
        # posisi Y pada frame sangat dipengaruhi perspektif dan geometri kamera; bukan altitude error nyata.
        err_x_n = abs(err["error_x_normalized"])
        err_y_n = abs(err["error_y_normalized"])  # debug/info only — NOT a hard requirement
        # is_centered: cukup horizontal centering saja untuk trigger DOCUMENT.
        is_centered = err_x_n < self.SCOUT_CENTERED_THRESHOLD
        # err_y disimpan untuk telemetry logging saja; TIDAK dipakai untuk koreksi altitude.
        err_y_signed = err["error_y_normalized"]  # debug only



        # DOCUMENT trigger: err_x (horizontal centering) + bbox_ratio cukup.
        # err_y TIDAK disyaratkan — kamera fixed forward-facing low altitude membuat Y bergantung perspektif.
        if is_centered and bbox_ratio >= self.SCOUT_ARRIVAL_BBOX_RATIO:
            # Multi-frame confirmation: target harus stabil beberapa frame sebelum DOCUMENT.
            self._document_confirm_count += 1
            if self._document_confirm_count >= self.SCOUT_DOCUMENT_CONFIRM_FRAMES:
                logger.info("DOCUMENT TRIGGER → err_x=%.3f err_y=%.3f(info) bbox=%.3f confirm=%d",
                    err_x_n, err_y_n, bbox_ratio, self._document_confirm_count)
                # TEST_A: Log transisi APPROACH→DOCUMENT.
                self._log_fsm_transition("APPROACH", "DOCUMENT")
                # TEST_D: Log posisi GPS saat DOCUMENT trigger untuk akurasi navigasi.
                d_pos = self.state_dict.get("position", {})
                logger.info("TEST_D → document_trigger lat=%.7f lon=%.7f alt=%.2f bbox=%.3f err_x=%.3f",
                    float(d_pos.get("lat",0)), float(d_pos.get("lon",0)), float(d_pos.get("alt",0)), bbox_ratio, err_x_n)
                self._scout_state = ScoutState.DOCUMENT
                self._document_snapshot_detection = self._latest_detection
                self._document_snapshot_frame = None
                if self._camera_source and hasattr(self._camera_source, 'latest_frame') and self._camera_source.latest_frame is not None:
                    self._document_snapshot_frame = self._camera_source.latest_frame.copy()
                self._emit_scout_state("DOCUMENT")
                self._document_confirm_count = 0
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, vz, cur_yaw))
                return
        else:
            # Soft-decrement: jitter sesaat tidak menghapus semua progress konfirmasi.
            # Ini mencegah reset total dari noise Y-axis sesaat.
            self._document_confirm_count = max(0, self._document_confirm_count - 1)

        # --- PD Controller untuk yaw (Issue #11: D-term damping) ---
        err_x = err["error_x_normalized"]
        # Derivative error yaw: perubahan error per tick untuk damping oscillation.
        d_err_x = (err_x - self._prev_err_x) / SCOUT_LOOP_PERIOD
        self._prev_err_x = err_x
        # Yaw command dengan P + D term untuk mengurangi overshoot/jitter.
        yaw_t = cur_yaw + (self.SCOUT_KP_YAW * err_x) + (self.SCOUT_KD_YAW * d_err_x)
        # Normalisasi yaw target agar berada pada rentang -180 sampai 180 derajat.
        while yaw_t > 180.0: yaw_t -= 360.0
        while yaw_t < -180.0: yaw_t += 360.0

        # --- Distance-based speed control dengan hysteresis deadband (Issues #1, #2, #5) ---
        # Tentukan zona movement berdasarkan bbox_ratio, dengan hysteresis mencegah getar.
        prev_zone = self._last_movement_zone
        if bbox_ratio < self.SCOUT_BBOX_FAR:
            zone = "far"
            v_forward = self.SCOUT_VEL_FAR
        elif bbox_ratio < self.SCOUT_BBOX_MED:
            zone = "medium"
            v_forward = self.SCOUT_VEL_MED
        elif bbox_ratio < self.SCOUT_BBOX_IDEAL_LO:
            zone = "near"
            v_forward = self.SCOUT_VEL_NEAR
        elif bbox_ratio <= self.SCOUT_BBOX_IDEAL_HI:
            zone = "ideal"
            v_forward = 0.0
        elif bbox_ratio <= self.SCOUT_BBOX_TOO_CLOSE:
            # Zona transisi: jika sebelumnya di ideal, tetap ideal (hysteresis).
            if prev_zone == "ideal":
                zone = "ideal"
                v_forward = 0.0
            else:
                zone = "close"
                v_forward = self.SCOUT_VEL_RETREAT
        else:
            zone = "retreat"
            v_forward = self.SCOUT_VEL_RETREAT
        # Hysteresis: jika baru saja pindah zona, pertahankan zona lama agar tidak getar.
        # Hanya zona ideal→close dan close→ideal yang di-hysteresis karena paling rawan oscillation.
        if prev_zone == "ideal" and zone == "near":
            # Cegah getar: tetap ideal jika perbedaan bbox kecil
            if bbox_ratio > (self.SCOUT_BBOX_IDEAL_LO - 0.03):
                zone = "ideal"
                v_forward = 0.0
        self._last_movement_zone = zone

        # Hitung velocity horizontal berdasarkan arah yaw, hanya dari v_forward (bbox-based).
        # TIDAK menggunakan cy_error sebagai forward speed (Issue #1: root cause oscillation).
        yaw_rad = math.radians(cur_yaw)
        vx = v_forward * math.cos(yaw_rad)
        vy = v_forward * math.sin(yaw_rad)

        # Clamp resultan velocity agar tidak melebihi batas maksimum.
        cspeed = math.sqrt(vx**2 + vy**2)
        if cspeed > self.SCOUT_MAX_VEL:
            vx, vy = (vx / cspeed) * self.SCOUT_MAX_VEL, (vy / cspeed) * self.SCOUT_MAX_VEL

        # TEST_G: Log raw vs smoothed untuk analisis EMA.
        if target is not None:
            logger.info("TEST_G_EMA → raw_cx=%.3f smooth_cx=%.3f raw_cy=%.3f smooth_cy=%.3f alpha=%.2f",
                cx, self._smooth_cx or 0, cy, self._smooth_cy or 0, effective_alpha)

        # TEST_C: Bawa capture_timestamp dari deteksi untuk pipeline latency.
        cap_ts = 0.0
        if self._latest_detection and hasattr(self._latest_detection, 'capture_timestamp'):
            cap_ts = self._latest_detection.capture_timestamp

        self._log_telemetry_snapshot(
            scout_state=self._scout_state.name,
            bbox_ratio=bbox_ratio,
            err_x=err["error_x_normalized"],
            err_y=err_y_signed,
            is_centered=is_centered,
            cmd_vn=vx,
            cmd_ve=vy,
            cmd_vz=vz,
            capture_timestamp=cap_ts,
        )

        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, vz, yaw_t))
        # TEST_C: t5 — VelocityNedYaw dikirim ke Pixhawk. Log total pipeline E2E dari frame capture.
        if cap_ts > 0:
            logger.info("TEST_C_CMD → t5_cmd_sent total_e2e=%.1fms", (time.time() - cap_ts) * 1000)

    # Handler state DOCUMENT; drone mencatat lokasi target dan mengambil screenshot/dokumentasi.
    async def _scout_state_document(self, vz: float):
        # Mengambil posisi drone terkini dari state_dict.
        pos = self.state_dict.get("position", {})
        # Mengambil latitude dan longitude dokumentasi dalam bentuk float.
        d_lat, d_lon = float(pos.get("lat", 0)), float(pos.get("lon", 0))
        # Mengambil yaw drone saat dokumentasi agar perintah hold tetap mempertahankan orientasi.
        cur_yaw = self.state_dict.get("attitude", {}).get("yaw", 0.0)

        # Menghentikan gerakan horizontal saat dokumentasi sambil tetap menjaga altitude melalui vz.
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, vz, cur_yaw))

        # Mengecek semua koordinat yang sudah pernah dikunjungi agar target yang sama tidak didokumentasikan ulang.
        for v in self._scout_visited_coords:
            if self._haversine_distance_m(v["lat"], v["lon"], d_lat, d_lon) < self.SCOUT_EXCLUSION_RADIUS_M:
                logger.info("MISSION → duplicate skipped (%.5f, %.5f)", d_lat, d_lon)
                # TEST_F: Increment duplicate skip counter.
                self._test_duplicates_skipped += 1
                logger.info("TEST_F → duplicate_skip total=%d at=(%.5f,%.5f)", self._test_duplicates_skipped, d_lat, d_lon)
                self._emit_scout_state("SCAN", {"message": "Sudah dikunjungi, displacement"})
                # TEST_A: Log transisi DOCUMENT→DISPLACING (duplicate).
                self._log_fsm_transition("DOCUMENT", "DISPLACING")
                self._start_displacement(cur_yaw)
                return

        # Mengambil screenshot scout menggunakan snapshot frame dari saat target centered (jika tersedia).
        fname = self._take_scout_screenshot(
            self._camera_source,
            self._document_snapshot_detection or self._latest_detection,
            pos, self.photos_dir, self.frame_width, self.frame_height,
            snapshot_frame=self._document_snapshot_frame
        )
        # Membersihkan snapshot setelah digunakan.
        self._document_snapshot_frame = None
        self._document_snapshot_detection = None
        self._scout_visited_coords.append({"lat": d_lat, "lon": d_lon})
        self._clear_person_target_lock()
        logger.info("MISSION → target documented (%.5f, %.5f) file=%s", d_lat, d_lon, fname)
        # TEST_F: Increment documented count dan log cycle time.
        self._test_targets_documented += 1
        cycle_time = time.time() - self._test_last_cycle_start if self._test_last_cycle_start > 0 else 0
        if cycle_time > 0:
            self._test_cycle_times.append(cycle_time)
        self._test_last_cycle_start = time.time()
        flight_elapsed = time.time() - self._test_mission_start_time if self._test_mission_start_time > 0 else 0
        logger.info("TEST_F → documented total=%d cycle_time=%.1fs flight_elapsed=%.0fs duplicates=%d",
            self._test_targets_documented, cycle_time, flight_elapsed, self._test_duplicates_skipped)
        # TEST_D: Log posisi dokumentasi aktual.
        logger.info("TEST_D → documented_pos lat=%.7f lon=%.7f alt=%.2f", d_lat, d_lon, float(self.state_dict.get("position",{}).get("alt",0)))
        self._emit("scout:documented", {"lat": d_lat, "lon": d_lon, "screenshot": fname})
        await asyncio.sleep(2.0)
        # TEST_A: Log transisi DOCUMENT→DISPLACING.
        self._log_fsm_transition("DOCUMENT", "DISPLACING")
        self._start_displacement(cur_yaw)

    # Helper untuk memulai state DISPLACING: simpan posisi awal dan waktu mulai.
    def _start_displacement(self, cur_yaw: float):
        """Inisiasi perpindahan ~SCOUT_DISPLACEMENT_M meter ke KANAN relatif dari yaw saat ini. State → DISPLACING."""
        pos = self.state_dict.get("position", {})
        self._scout_displacement_start_pos = {
            "lat": float(pos.get("lat", 0)),
            "lon": float(pos.get("lon", 0)),
        }
        self._scout_displacement_start_time = asyncio.get_event_loop().time()
        # Arah gerakan adalah yaw + 90° (kanan relatif dari arah hadap drone).
        # cur_yaw disimpan terpisah di _scout_scan_yaw sebagai movement vector yaw.
        # Orientasi/hadap drone TIDAK diubah selama displacement.
        right_yaw = (cur_yaw + 90.0) % 360.0
        self._scout_scan_yaw = right_yaw
        # _scout_displacement_facing_yaw menyimpan yaw hadap asli agar VelocityNedYaw bisa mempertahankan orientasi.
        self._scout_displacement_facing_yaw = cur_yaw
        self._scout_state = ScoutState.DISPLACING
        logger.info("MISSION → right displacement start facing_yaw=%.0f move_yaw=%.0f", cur_yaw, right_yaw)
        # TEST_D: Log posisi GPS sebelum displacement.
        dp = self.state_dict.get("position", {})
        logger.info("TEST_D → displacement_start lat=%.7f lon=%.7f", float(dp.get("lat",0)), float(dp.get("lon",0)))
        self._emit_scout_state("DISPLACING", {"message": "Right displacement dimulai"})

    # Handler state DISPLACING; drone bergerak maju ~5m lalu kembali ke SCAN.
    async def _scout_state_displacing(self, vz: float):
        """Gerakkan drone ~SCOUT_DISPLACEMENT_M ke depan lalu masuk SCAN."""
        cur_time = asyncio.get_event_loop().time()
        pos = self.state_dict.get("position", {})
        cur_lat = float(pos.get("lat", 0))
        cur_lon = float(pos.get("lon", 0))
        cur_yaw = self.state_dict.get("attitude", {}).get("yaw", 0.0)

        # Hitung jarak dari titik awal displacement.
        start = self._scout_displacement_start_pos
        dist_moved = self._haversine_distance_m(start["lat"], start["lon"], cur_lat, cur_lon) if start else 0.0

        # Cek timeout displacement agar tidak tersangkut.
        elapsed = cur_time - (self._scout_displacement_start_time or cur_time)
        timeout = elapsed > self.SCOUT_DISPLACEMENT_TIMEOUT_S

        # Cek apakah jarak target sudah tercapai atau timeout.
        if dist_moved >= self.SCOUT_DISPLACEMENT_M or timeout:
            if timeout:
                logger.warning("MISSION → displacement timeout after %.1fs", elapsed)
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, vz, cur_yaw))
            # TEST_D: Log posisi GPS setelah displacement dan jarak aktual vs target.
            logger.info("TEST_D → displacement_end lat=%.7f lon=%.7f moved=%.2fm target=%.1fm error=%.2fm",
                cur_lat, cur_lon, dist_moved, self.SCOUT_DISPLACEMENT_M, abs(dist_moved - self.SCOUT_DISPLACEMENT_M))
            self._scout_displacement_start_pos = None
            self._scout_displacement_start_time = None
            # TEST_A: Log transisi DISPLACING→SCAN.
            self._log_fsm_transition("DISPLACING", "SCAN")
            logger.info("MISSION → right displacement complete (moved=%.1fm), re-entering SCAN", dist_moved)
            self._reset_scout_scan()
            self._emit_scout_state("SCAN", {"message": "Right displacement selesai, scan lagi"})
            return

        # Masih dalam displacement: gerak ke KANAN relatif dari hadap drone.
        # _scout_scan_yaw sudah berisi right_yaw (facing_yaw + 90°) dari _start_displacement.
        move_yaw_rad = math.radians(self._scout_scan_yaw)
        spd = self.SCOUT_DISPLACEMENT_SPEED
        vx = spd * math.cos(move_yaw_rad)
        vy = spd * math.sin(move_yaw_rad)
        # Pertahankan orientasi hadap asli drone (_scout_displacement_facing_yaw), bukan move_yaw.
        facing_yaw = getattr(self, "_scout_displacement_facing_yaw", cur_yaw)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, vz, facing_yaw))
        self._log_telemetry_snapshot(
            scout_state="DISPLACING",
            cmd_vn=vx,
            cmd_ve=vy,
            cmd_vz=vz,
        )


    # Handler state RETURN_HOME; drone bergerak kembali ke titik home scout.
    async def _scout_state_return_home(self, vz: float, loop_count: int):
        # Mengambil posisi drone saat ini.
        pos = self.state_dict.get("position", {})
        # Mengambil latitude dan longitude saat ini dalam bentuk float.
        cur_lat, cur_lon = float(pos.get("lat", 0)), float(pos.get("lon", 0))
        
        # Menghitung velocity menuju home, jarak ke home, dan heading/yaw yang harus diarahkan.
        vx, vy, dist, hdg = self._compute_home_velocity(
            # Parameter awal adalah posisi drone saat ini dan posisi home scout yang disimpan saat start_scout.
            cur_lat, cur_lon, self._scout_home_position["lat"], self._scout_home_position["lon"],
            # Parameter berikutnya adalah yaw saat ini dan batas kecepatan pulang.
            self.state_dict.get("attitude", {}).get("yaw", 0.0), self.SCOUT_RETURN_MAX_SPEED
        )

        # Jika jarak ke home sudah lebih kecil/sama dengan ambang arrival, drone dianggap sampai.
        if dist <= self.SCOUT_RETURN_ARRIVAL_M:
            # Menghentikan gerak horizontal di home sambil tetap koreksi altitude.
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, vz, hdg))
            # Delay pendek untuk memberi waktu drone stabil di sekitar home.
            await asyncio.sleep(0.5)
            # Jika flag return_then_hold aktif, scout dihentikan setelah sampai home.
            if self._scout_return_then_hold:
                # Reset flag agar tidak memengaruhi operasi berikutnya.
                self._scout_return_then_hold = False
                # Mematikan mode scout sehingga loop utama berhenti.
                self._scout_mode = False
                # Mengembalikan state ke IDLE karena misi scout selesai/berhenti.
                self._scout_state = ScoutState.IDLE
                # Mengirim event bahwa home sudah dicapai.
                self._emit_scout_state("HOME_REACHED", {"message": "Sudah sampai home"})
                # Return karena proses return-home sudah selesai.
                return
            # Jika tidak diminta hold, scout kembali ke SCAN untuk memulai siklus pencarian baru.
            self._reset_scout_scan()
            # Mengirim event bahwa cycle scout baru dimulai.
            self._emit_scout_state("SCAN", {"message": "Mulai cycle baru"})
        # Jika belum sampai home, drone terus diberi velocity menuju home.
        else:
            # Mengirim perintah velocity horizontal menuju home dengan yaw heading hasil perhitungan.
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, vz, hdg))
            # Telemetry snapshot ~5Hz selama RETURN_HOME untuk analisis lintasan pulang.
            self._log_telemetry_snapshot(
                scout_state="RETURN_HOME",
                cmd_vn=vx,
                cmd_ve=vy,
                cmd_vz=vz,
            )

    # Loop utama scout yang menjalankan state machine selama _scout_mode masih True.
    async def _scout_main_loop(self):
        # Saat loop dimulai, scout selalu masuk state SCAN terlebih dahulu.
        self._reset_scout_scan()
        self._emit_scout_state("SCAN")
        loop_count = 0
        # TEST_F: Catat waktu mulai misi untuk durasi total.
        self._test_mission_start_time = time.time()
        self._test_last_cycle_start = time.time()

        # Selama mode scout aktif, loop ini terus menjalankan handler sesuai state saat ini.
        while self._scout_mode:
            # Menambah counter setiap iterasi loop.
            loop_count += 1
            # try digunakan agar error pada satu iterasi tidak langsung mematikan loop scout secara permanen.
            try:
                # Mengecek baterai dengan hysteresis untuk mencegah trigger palsu dari noise sensor.
                battery = self.state_dict.get("battery_pct", 100)
                if battery < self.BATTERY_RTL_THRESHOLD and not self._battery_rtl_triggered:
                    self._battery_rtl_triggered = True
                    # TEST_E: Record trigger timestamp for response time measurement.
                    _rtl_trigger_time = time.time()
                    logger.info("MISSION → battery RTL triggered (%.0f%%), pre-RTL climb phase", battery)
                    logger.info("TEST_E → battery_rtl_trigger batt=%.1f%% timestamp=%.3f", battery, _rtl_trigger_time)
                    self._emit_scout_state("RETURN_HOME", {"message": "Baterai rendah, climbing sebelum RTL"})
                    self._scout_mode = False
                    self._scout_state = ScoutState.IDLE
                    # TEST_F: Log mission summary saat battery RTL via helper (juga dipanggil di stop_scout).
                    self._log_test_f_summary("battery_rtl")
                    cur_yaw = self.state_dict.get("attitude", {}).get("yaw", 0.0)
                    alt_before = float(self.state_dict.get("position", {}).get("alt", 0.0))
                    logger.info("PRE-RTL → climbing at vz=%.1f for %.1fs", self.SCOUT_PRE_RTL_VZ, self.SCOUT_PRE_RTL_CLIMB_S)
                    climb_deadline = asyncio.get_event_loop().time() + self.SCOUT_PRE_RTL_CLIMB_S
                    while asyncio.get_event_loop().time() < climb_deadline:
                        try:
                            await self.drone.offboard.set_velocity_ned(
                                VelocityNedYaw(0.0, 0.0, self.SCOUT_PRE_RTL_VZ, cur_yaw)
                            )
                        except Exception as climb_err:
                            logger.error("PRE-RTL climb cmd failed: %s", climb_err)
                            break
                        await asyncio.sleep(SCOUT_LOOP_PERIOD)
                    try:
                        await self.drone.offboard.set_velocity_ned(
                            VelocityNedYaw(0.0, 0.0, 0.0, cur_yaw)
                        )
                    except Exception:
                        pass
                    alt_after = float(self.state_dict.get("position", {}).get("alt", 0.0))
                    _rtl_complete_time = time.time()
                    # TEST_E: Log response time and altitude gain.
                    logger.info("TEST_E → pre_rtl_complete response_time=%.0fms alt_before=%.2f alt_after=%.2f gain=%.2fm expected=%.2fm",
                        (_rtl_complete_time - _rtl_trigger_time) * 1000, alt_before, alt_after,
                        alt_after - alt_before, abs(self.SCOUT_PRE_RTL_VZ) * self.SCOUT_PRE_RTL_CLIMB_S)
                    logger.info("PRE-RTL → climb complete, sending PX4 RTL")
                    try:
                        await self.drone.action.return_to_launch()
                    except Exception as rtl_err:
                        logger.error("Battery RTL command failed: %s", rtl_err)
                    break
                elif battery > self.BATTERY_RTL_RECOVER:
                    # Reset flag hysteresis saat baterai sudah di atas ambang pemulihan.
                    self._battery_rtl_triggered = False

                # Mengambil altitude aktual drone.
                alt = float(self.state_dict.get("position", {}).get("alt", 0.0))
                # Menentukan altitude target scout; jika belum ada target, gunakan altitude aktual.
                target_alt = self._target_altitude_scout if self._target_altitude_scout is not None else alt
                # Altitude floor enforcement: jika altitude terlapor turun di bawah MIN_SAFE_ALTITUDE_M,
                # paksa target altitude ke floor agar vz mendorong drone naik.
                # Ini mengatasi barometer drift di mana log menunjukkan 1.6m tapi drone sebenarnya <60cm.
                if target_alt < self.MIN_SAFE_ALTITUDE_M:
                    target_alt = self.MIN_SAFE_ALTITUDE_M
                    self._target_altitude_scout = target_alt
                    # TEST_E: Increment altitude floor clamp counter.
                    self._test_alt_floor_clamp_count += 1
                    logger.warning("ALT FLOOR → target_alt clamped to %.1fm (baro drift protection) TEST_E count=%d",
                        target_alt, self._test_alt_floor_clamp_count)
                # Dampen altitude gain saat di zona ideal/dekat document transition untuk kurangi oscillation vz.
                kp_alt_eff = self.SCOUT_KP_ALT
                if self._scout_state == ScoutState.APPROACH and self._last_movement_zone in ("ideal", "near"):
                    kp_alt_eff = self.SCOUT_KP_ALT_DAMPED
                # Menghitung velocity vertikal untuk menjaga altitude, dibatasi antara -SCOUT_MAX_VZ dan SCOUT_MAX_VZ.
                vz = max(-self.SCOUT_MAX_VZ, min(self.SCOUT_MAX_VZ, -kp_alt_eff * (target_alt - alt)))
                # TEST_H: Log resource snapshot periodik (setiap 5 detik).
                self._log_resource_snapshot(self._scout_state.name)

                if self._scout_state == ScoutState.SCAN: await self._scout_state_scan(vz, loop_count)
                elif self._scout_state == ScoutState.APPROACH: await self._scout_state_approach(vz, loop_count)
                elif self._scout_state == ScoutState.DOCUMENT: await self._scout_state_document(vz)
                elif self._scout_state == ScoutState.DISPLACING: await self._scout_state_displacing(vz)
                elif self._scout_state == ScoutState.RETURN_HOME: await self._scout_state_return_home(vz, loop_count)

                # Sleep sesuai periode loop agar frekuensi kontrol sekitar 20 Hz.
                await asyncio.sleep(SCOUT_LOOP_PERIOD)
            # Menangkap semua exception agar error dicatat dan loop bisa mencoba lanjut pada iterasi berikutnya.
            except Exception as e:
                # Mencatat pesan error scout untuk debugging.
                logger.error("Scout err: %s", e)
                # Memberi jeda pendek setelah error agar loop tidak berputar terlalu cepat saat ada masalah berulang.
                await asyncio.sleep(0.1)
