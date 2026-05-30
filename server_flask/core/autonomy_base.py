#!/usr/bin/env python3
# Shebang ini memberi tahu sistem Unix/Linux bahwa file dijalankan menggunakan interpreter Python 3.
"""
autonomy_base.py
================
Base class for AutonomyController.
Memuat state machine dasar, inisialisasi parameter, dan pembacaan Queue YOLO.
"""
# asyncio digunakan untuk menjalankan task asynchronous seperti pembaca queue deteksi dan kontrol drone.
import asyncio
# time digunakan untuk rate-limiting telemetry snapshot agar tidak ditulis setiap frame kontrol.
import time
# logging digunakan untuk mencatat informasi, warning, dan error dari modul autonomy base.
import logging
# Enum dan auto digunakan untuk membuat kumpulan state bernama yang aman dibaca dan dibandingkan.
from enum import Enum, auto
# Queue adalah struktur antrian thread-safe untuk menerima hasil YOLO, Empty adalah exception saat queue kosong.
from queue import Queue, Empty
# Optional dan Dict digunakan sebagai type hint agar tipe parameter dan return value lebih jelas.
from typing import Optional, Dict
# AutonomyHelpers adalah mixin/helper yang menyediakan fungsi matematika, parsing deteksi, target lock, dan screenshot.
from .autonomy_helpers import AutonomyHelpers
# logger dibuat per modul agar pesan log mencantumkan sumber dari autonomy_base.py.
logger = logging.getLogger(__name__)
# AutonomyState mendefinisikan state utama untuk mode autonomy standar.
class AutonomyState(Enum):
    # IDLE berarti sistem autonomy standar tidak sedang melakukan tracking/searching.
    IDLE = auto()
    # TRACKING berarti sistem sedang mengikuti target yang terdeteksi.
    TRACKING = auto()
    # SEARCHING berarti sistem sedang mencari ulang target yang hilang.
    SEARCHING = auto()
    # EMERGENCY berarti sistem berada pada kondisi darurat.
    EMERGENCY = auto()
# ScoutState mendefinisikan state khusus untuk mode scout/pencarian orang secara mandiri.
class ScoutState(Enum):
    # IDLE berarti scout tidak aktif.
    IDLE = auto()
    # SCAN berarti drone sedang memutar/mencari target person.
    SCAN = auto()
    # APPROACH berarti drone sedang mendekati target person yang dipilih.
    APPROACH = auto()
    # DOCUMENT berarti drone sedang mendokumentasikan target/lokasi.
    DOCUMENT = auto()
    # DISPLACING berarti drone sedang melakukan perpindahan kecil setelah dokumentasi untuk eksplorasi area baru.
    DISPLACING = auto()
    # RETURN_HOME dipertahankan untuk kompatibilitas; tidak dipakai dalam normal persistent mission flow.
    RETURN_HOME = auto()
# AutonomyBase adalah class dasar yang menyimpan konfigurasi, state bersama, dan utility untuk class turunannya.
class AutonomyBase(AutonomyHelpers):
    """Base class untuk Autonomy Standard dan Scout."""
    # CONTROL_HZ adalah frekuensi kontrol standar dalam Hertz; 20 berarti 20 iterasi per detik.
    CONTROL_HZ = 20
    # CONTROL_PERIOD adalah periode tiap iterasi kontrol dalam detik.
    CONTROL_PERIOD = 1.0 / CONTROL_HZ
    # TELEMETRY_LOG_INTERVAL adalah interval minimum antar snapshot TEL dalam detik (5Hz = 0.2s).
    TELEMETRY_LOG_INTERVAL: float = 0.20
    # KP_YAW adalah gain koreksi yaw, KP_FORWARD gain gerak maju, dan KP_ALT gain koreksi altitude untuk mode standar.
    KP_YAW, KP_FORWARD, KP_ALT = 30.0, 0.5, 0.8
    # MAX_VEL membatasi kecepatan horizontal mode standar, MAX_VZ membatasi kecepatan vertikal.
    MAX_VEL, MAX_VZ = 0.5, 0.4
    # Konfigurasi gain khusus mode scout untuk yaw, gerak maju, dan altitude.
    SCOUT_KP_YAW, SCOUT_KP_FORWARD, SCOUT_KP_ALT = 20.0, 0.3, 0.8
    # Batas kecepatan horizontal dan vertikal khusus mode scout.
    SCOUT_MAX_VEL, SCOUT_MAX_VZ = 0.5, 0.4
    # SCOUT_SEARCH_YAW_RATE adalah kecepatan rotasi yaw saat drone melakukan scanning target.
    SCOUT_SEARCH_YAW_RATE = 20.0
    # Rasio tinggi bounding box terhadap tinggi frame yang menandakan drone sudah cukup dekat dengan target.
    # BUGFIX: diturunkan dari 0.35 → 0.28 agar threshold reachable (bbox ideal zone mulai 0.28).
    SCOUT_ARRIVAL_BBOX_RATIO = 0.28
    # Batas waktu maksimum proses approach sebelum dianggap gagal/timeout.
    SCOUT_APPROACH_TIMEOUT_S = 30.0
    # --- Distance-based speed control with hysteresis deadband ---
    # Batas bbox_ratio zona: jauh, sedang, dekat, ideal (deadband), terlalu dekat
    SCOUT_BBOX_FAR = 0.15           # bbox < ini → jauh, maju cepat
    SCOUT_BBOX_MED = 0.25           # bbox < ini → sedang, maju sedang
    SCOUT_BBOX_IDEAL_LO = 0.28      # batas bawah deadband zona ideal
    SCOUT_BBOX_IDEAL_HI = 0.42      # batas atas deadband zona ideal
    SCOUT_BBOX_TOO_CLOSE = 0.50     # bbox > ini → terlalu dekat, mundur
    # Kecepatan maju/mundur per zona
    SCOUT_VEL_FAR = 0.5             # m/s saat jauh
    SCOUT_VEL_MED = 0.3             # m/s saat sedang
    SCOUT_VEL_NEAR = 0.15           # m/s saat dekat (di bawah arrival tapi di atas ideal)
    SCOUT_VEL_RETREAT = -0.2        # m/s mundur saat terlalu dekat
    # --- D-term gains untuk damping oscillation ---
    SCOUT_KD_YAW = 5.0              # Derivative gain yaw
    # --- Multi-frame confirmation SCAN→APPROACH ---
    SCOUT_APPROACH_CONFIRM_FRAMES = 3  # 3 frame di 20Hz = 150ms
    # --- Multi-frame confirmation APPROACH→DOCUMENT ---
    SCOUT_DOCUMENT_CONFIRM_FRAMES = 5  # 5 frame di 20Hz = 250ms hover stabil
    # --- Detection freshness ---
    SCOUT_DETECTION_MAX_AGE_S = 1.5 # Deteksi stale jika lebih tua dari ini
    # --- Target lost timeout yang lebih toleran ---
    SCOUT_TARGET_LOST_TIMEOUT_S = 3.0  # detik sebelum kembali ke SCAN
    # --- Centered threshold yang lebih ketat ---
    SCOUT_CENTERED_THRESHOLD = 0.10    # normalized, lebih presisi dari 0.15 lama
    SCOUT_CENTERED_THRESHOLD_Y = 0.18  # Y-axis lebih toleran karena altitude jitter di outdoor
    # --- Damped altitude gain saat dekat document transition ---
    SCOUT_KP_ALT_DAMPED = 0.35         # gain altitude lebih rendah saat bbox ideal, kurangi oscillation vz
    # --- Vertical centering correction: err_y → vz adjustment ---
    # BUGFIX: gain untuk koreksi altitude berbasis posisi target di frame kamera.
    # Nilai kecil (0.15) cukup untuk mendorong drone naik/turun perlahan menuju target.
    SCOUT_KP_VZ_VERTICAL = 0.15        # gain err_y_normalized → vz correction
    # Radius eksklusi dalam meter agar target/lokasi yang sudah didokumentasikan tidak diulang.
    SCOUT_EXCLUSION_RADIUS_M = 3.0
    # Jarak perpindahan lateral setelah dokumentasi atau duplicate skip, dalam meter.
    SCOUT_DISPLACEMENT_M = 2.0
    # Kecepatan maju saat displacement post-document (m/s).
    SCOUT_DISPLACEMENT_SPEED = 0.4
    # Timeout displacement dalam detik agar tidak tersangkut jika ada hambatan.
    SCOUT_DISPLACEMENT_TIMEOUT_S = 15.0
    # Jarak maksimum dari home agar drone dianggap sudah sampai home scout.
    SCOUT_RETURN_ARRIVAL_M = 2.0
    # Batas kecepatan maksimum saat kembali ke home scout.
    SCOUT_RETURN_MAX_SPEED = 1.0
    # Ambang baterai minimum; jika baterai di bawah ini, sistem diarahkan return-to-launch/return-home.
    BATTERY_RTL_THRESHOLD = 15.0
    # Ambang pemulihan baterai; harus naik di atas ini agar flag RTL direset (hysteresis).
    BATTERY_RTL_RECOVER = 18.0
    # Altitude minimum keselamatan dalam meter.
    # Barometer drift sering menyebabkan relative_altitude_m lebih tinggi dari AGL asli.
    # Contoh: log menunjukkan 1.6m padahal drone sebenarnya <60cm di atas tanah.
    # Floor ini memastikan target altitude scout tidak pernah lebih rendah dari batas aman.
    MIN_SAFE_ALTITUDE_M = 1.2
    # Durasi climb darurat sebelum mengirim PX4 RTL (detik).
    # Memberi drone waktu naik ke altitude aman sebelum RTL yang membawa drone horizontal dengan kecepatan tinggi.
    SCOUT_PRE_RTL_CLIMB_S = 3.0
    # Kecepatan climb (m/s, NED down negatif = naik) selama fase pre-RTL.
    SCOUT_PRE_RTL_VZ = -0.8
    # Durasi maksimum target lock dipertahankan saat target tidak terlihat.
    TARGET_LOCK_MAX_AGE_S = 1.5
    # Rasio skor yang dibutuhkan agar sistem boleh berpindah dari target terkunci ke target baru.
    TARGET_SWITCH_SCORE_RATIO = 1.35
    # Jarak maksimum di frame normalized agar kandidat dianggap masih target yang sama.
    TARGET_MATCH_MAX_DIST = 0.35
    # Konstruktor menerima objek drone, queue deteksi YOLO, state telemetri, Socket.IO, dan folder foto opsional.
    def __init__(self, drone, detection_queue: Queue, telemetry_state: Dict, socketio, photos_dir: Optional[str] = None):
        # drone adalah objek MAVSDK atau wrapper drone yang dipakai untuk mengirim perintah aksi/offboard.
        self.drone = drone
        # detection_queue adalah antrian hasil deteksi dari pipeline YOLO ke modul autonomy.
        self.detection_queue = detection_queue
        # state_dict menyimpan telemetri bersama seperti posisi, attitude, baterai, dan deteksi terakhir.
        self.state_dict = telemetry_state
        # socketio dipakai untuk mengirim event status ke frontend/client secara real-time.
        self.socketio = socketio
        # photos_dir adalah folder penyimpanan screenshot scout; default "./photos" jika tidak diberikan.
        self.photos_dir = photos_dir or "./photos"
        # _camera_source menyimpan sumber kamera untuk screenshot; awalnya None sampai di-set lewat set_camera_source().
        self._camera_source = None
        # Standard Mode State
        # _state adalah state machine utama untuk mode autonomy standar.
        self._state = AutonomyState.IDLE
        # _running menandakan apakah sistem autonomy utama sedang berjalan.
        self._running = False
        # _task menyimpan task asynchronous kontrol standar jika dibuat oleh class turunan.
        self._task = None
        # _latest_detection menyimpan hasil deteksi YOLO terbaru yang dibaca dari queue.
        self._latest_detection = None
        # _detection_reader_task menyimpan task pembaca queue deteksi.
        self._detection_reader_task = None
        # _search_yaw menyimpan yaw pencarian saat mode standar kehilangan target.
        self._search_yaw = 0.0
        # _target_alt menyimpan altitude target untuk mode standar.
        self._target_alt = None
        # Smoothing
        # _alpha adalah faktor exponential moving average untuk memperhalus posisi target pada frame kamera.
        self._alpha = 0.20
        # _smooth_cx menyimpan posisi X target yang sudah dihaluskan.
        self._smooth_cx = None
        # _smooth_cy menyimpan posisi Y target yang sudah dihaluskan.
        self._smooth_cy = None
        # _target_lock menyimpan target person yang sedang dikunci agar tracking tidak mudah berpindah.
        self._target_lock = None
        # Scout Mode State
        # _scout_mode menandakan apakah mode scout sedang aktif.
        self._scout_mode = False
        # _scout_state adalah state machine khusus scout.
        self._scout_state = ScoutState.IDLE
        # _scout_task menyimpan task asynchronous loop scout.
        self._scout_task = None
        # _scout_visited_coords menyimpan daftar koordinat yang sudah didokumentasikan oleh scout.
        self._scout_visited_coords = []
        # _scout_home_position menyimpan posisi home saat scout dimulai.
        self._scout_home_position = None
        # _scout_displacement_start_pos menyimpan posisi lat/lon saat displacement dimulai.
        self._scout_displacement_start_pos = None
        # _scout_displacement_start_time menyimpan waktu saat displacement dimulai untuk timeout.
        self._scout_displacement_start_time = None
        # _scout_scan_rotated menyimpan total derajat rotasi scan yang sudah dilakukan.
        self._scout_scan_rotated = 0.0
        # _scout_scan_yaw menyimpan yaw target saat proses scan.
        self._scout_scan_yaw = 0.0
        # _approach_start_time menyimpan waktu mulai approach untuk kebutuhan timeout.
        self._approach_start_time = None
        # _target_altitude_scout menyimpan altitude target yang dipertahankan selama scout.
        self._target_altitude_scout = None
        # _scout_return_then_hold menandakan apakah scout harus berhenti setelah pulang ke home.
        self._scout_return_then_hold = False
        # _battery_rtl_triggered adalah flag hysteresis agar battery RTL tidak trigger berulang dari noise sensor.
        self._battery_rtl_triggered = False
        # _document_snapshot_frame menyimpan frame kamera saat target centered untuk screenshot yang akurat.
        self._document_snapshot_frame = None
        # _document_snapshot_detection menyimpan deteksi saat target centered.
        self._document_snapshot_detection = None
        # --- State tambahan untuk PD controller dan konfirmasi approach ---
        # D-term: error yaw sebelumnya untuk menghitung derivative
        self._prev_err_x = 0.0
        # Counter konfirmasi multi-frame sebelum transisi SCAN→APPROACH
        self._approach_confirm_count = 0
        # Counter konfirmasi multi-frame sebelum transisi APPROACH→DOCUMENT
        self._document_confirm_count = 0
        # State terakhir zona movement untuk hysteresis (mencegah getar maju-mundur)
        self._last_movement_zone = "idle"
        # frame_width adalah lebar frame kamera default sebelum metadata deteksi terbaru tersedia.
        self.frame_width = 640
        # frame_height adalah tinggi frame kamera default sebelum metadata deteksi terbaru tersedia.
        self.frame_height = 480
        # _telemetry_last_log menyimpan timestamp terakhir snapshot TEL ditulis; rate ~5Hz (0.2s interval).
        self._telemetry_last_log: float = 0.0
    # start harus diimplementasikan oleh class turunan karena base class belum tahu strategi kontrol spesifik.
    async def start(self): raise NotImplementedError
    # stop harus diimplementasikan oleh class turunan karena proses penghentian bisa berbeda per mode.
    async def stop(self): raise NotImplementedError
    # emergency_stop menghentikan mode aktif dan memerintahkan drone return_to_launch sebagai prosedur darurat.
    async def emergency_stop(self):
        # Log warning dipakai karena emergency stop adalah kejadian serius.
        logger.warning("EMERGENCY STOP Triggered")
        # Jika scout sedang aktif, hentikan scout terlebih dahulu agar task scout dan state-nya dibersihkan.
        if self._scout_mode:
            # stop_scout berasal dari class turunan yang menggabungkan AutonomyBase dengan mode scout.
            await self.stop_scout()
        # Menghentikan mode autonomy utama melalui implementasi stop() pada class turunan.
        await self.stop()
        # === PRE-RTL SAFETY CLIMB ===
        # Barometer drift: relative_altitude_m bisa jauh lebih tinggi dari AGL nyata.
        # Climb paksa sebelum RTL agar drone tidak crash di ketinggian rendah.
        try:
            from mavsdk.offboard import VelocityNedYaw as _VNY
            cur_yaw = self.state_dict.get("attitude", {}).get("yaw", 0.0)
            logger.info("EMERGENCY PRE-RTL → climbing %.1fs at vz=%.1f", self.SCOUT_PRE_RTL_CLIMB_S, self.SCOUT_PRE_RTL_VZ)
            import asyncio as _aio
            climb_end = _aio.get_event_loop().time() + self.SCOUT_PRE_RTL_CLIMB_S
            while _aio.get_event_loop().time() < climb_end:
                try:
                    await self.drone.offboard.set_velocity_ned(_VNY(0.0, 0.0, self.SCOUT_PRE_RTL_VZ, cur_yaw))
                except Exception:
                    break
                await _aio.sleep(0.05)
            # Stop horizontal before RTL handoff.
            try:
                await self.drone.offboard.set_velocity_ned(_VNY(0.0, 0.0, 0.0, cur_yaw))
            except Exception:
                pass
        except Exception as climb_err:
            logger.error("EMERGENCY PRE-RTL climb error: %s", climb_err)
        # try digunakan agar kegagalan perintah RTL tidak membuat exception tidak tertangani.
        try:
            # return_to_launch adalah perintah MAVSDK agar drone pulang ke launch/home point autopilot.
            await self.drone.action.return_to_launch()
        # Jika perintah RTL gagal, error dicatat untuk diagnosis.
        except Exception as e:
            # Mencatat pesan kegagalan return-to-launch.
            logger.error("RTL failed: %s", e)
    # get_scout_status mengembalikan snapshot status scout untuk API/frontend.
    def get_scout_status(self) -> Dict:
        """Kembalikan status scout mode saat ini."""
        # Dictionary ini adalah output status yang bisa dikonsumsi oleh endpoint atau Socket.IO.
        return {
            # active menunjukkan apakah scout mode sedang berjalan.
            "active": self._scout_mode,
            # state berisi nama state scout jika aktif; jika tidak aktif dikembalikan IDLE.
            "state": self._scout_state.name if self._scout_mode else "IDLE",
            # home_position adalah koordinat home scout yang disimpan saat start_scout.
            "home_position": self._scout_home_position,
            # visited_count adalah jumlah koordinat/person yang sudah didokumentasikan.
            "visited_count": len(self._scout_visited_coords),
            # visited_coords adalah salinan daftar koordinat agar pemanggil tidak mengubah list internal langsung.
            "visited_coords": self._scout_visited_coords.copy(),
            # scan_progress_deg adalah progres rotasi scan dalam derajat.
            "scan_progress_deg": float(self._scout_scan_rotated),
            # config berisi sebagian parameter scout penting untuk ditampilkan/ditinjau.
            "config": {
                # arrival_bbox_ratio adalah ambang ukuran bbox untuk menentukan target sudah dekat.
                "arrival_bbox_ratio": self.SCOUT_ARRIVAL_BBOX_RATIO,
                # exclusion_radius_m adalah radius lokasi yang dianggap sudah dikunjungi.
                "exclusion_radius_m": self.SCOUT_EXCLUSION_RADIUS_M,
                # return_arrival_m adalah ambang jarak agar drone dianggap sampai home scout.
                "return_arrival_m": self.SCOUT_RETURN_ARRIVAL_M,
            },
        }
    # _detection_reader adalah task asynchronous yang terus membaca hasil deteksi YOLO dari queue.
    async def _detection_reader(self):
        # Mengambil event loop aktif agar operasi blocking queue bisa dijalankan di executor.
        loop = asyncio.get_event_loop()
        # Reader terus berjalan selama autonomy utama aktif.
        while self._running:
            # try menjaga reader tetap hidup walaupun terjadi error pembacaan deteksi.
            try:
                # _poll_detection_queue bersifat blocking, jadi dijalankan di executor agar tidak memblokir event loop.
                res = await loop.run_in_executor(None, self._poll_detection_queue)
                # Jika ada hasil deteksi baru, simpan sebagai deteksi terbaru.
                if res is not None:
                    # Menyimpan object deteksi terbaru untuk dipakai mode tracking/scout.
                    self._latest_detection = res
                    # Menaruh deteksi terbaru ke state_dict agar bagian lain sistem dapat membacanya.
                    self.state_dict["latest_detection"] = res
                    # Jika hasil deteksi membawa metadata frame_width, ukuran frame internal ikut diperbarui.
                    if hasattr(res, "frame_width"):
                        # frame_width dan frame_height dipakai untuk parsing bbox dan perhitungan error frame.
                        self.frame_width, self.frame_height = res.frame_width, res.frame_height
                # Jika queue kosong, beri jeda sangat pendek agar loop tidak sibuk penuh.
                else:
                    # Sleep kecil mengurangi penggunaan CPU ketika tidak ada deteksi baru.
                    await asyncio.sleep(0.005)
            # CancelledError terjadi saat task reader dibatalkan ketika autonomy berhenti.
            except asyncio.CancelledError: break
            # Exception umum dicatat agar masalah reader bisa dianalisis tanpa langsung mematikan program.
            except Exception as e:
                # Mencatat error pembacaan queue deteksi.
                logger.error("Reader err: %s", e)
                # Delay setelah error mencegah loop error berulang terlalu cepat.
                await asyncio.sleep(0.1)
    # _poll_detection_queue mengambil satu item dari detection_queue dengan timeout pendek.
    def _poll_detection_queue(self):
        # Jika ada item dalam 0.05 detik, item tersebut langsung dikembalikan.
        try: return self.detection_queue.get(timeout=0.05)
        # Jika queue kosong sampai timeout, fungsi mengembalikan None.
        except Empty: return None
    # set_camera_source menyimpan object kamera agar helper screenshot scout bisa mengambil frame.
    def set_camera_source(self, camera):
        # camera bisa berupa object dengan latest_frame atau object dengan method read().
        self._camera_source = camera
    # _emit mengirim event ke client melalui Socket.IO jika socketio tersedia.
    def _emit(self, event: str, data: dict):
        # Jika socketio None, tidak ada event yang dikirim.
        if self.socketio:
            # try menjaga agar error emit tidak menghentikan proses autonomy.
            try: self.socketio.emit(event, data, namespace="/")
            # Jika emit gagal, error dicatat ke log.
            except Exception as e: logger.error("Emit err: %s", e)
    # _log_telemetry_snapshot menulis snapshot telemetri ringkas ~5Hz untuk analisis post-flight.
    # Semua data diambil dari state_dict dan parameter yang dipassing dari caller.
    # Rate-limited: hanya menulis jika sudah melewati TELEMETRY_LOG_INTERVAL sejak snapshot terakhir.
    def _log_telemetry_snapshot(
        self,
        *,
        scout_state: str = "",
        bbox_ratio: float = 0.0,
        err_x: float = 0.0,
        err_y: float = 0.0,
        is_centered: bool = False,
        cmd_vn: float = 0.0,
        cmd_ve: float = 0.0,
        cmd_vz: float = 0.0,
    ) -> None:
        """Write a compact TEL → snapshot at ~5Hz. No-op if called too frequently."""
        now = time.monotonic()
        # Guard: skip if called more often than TELEMETRY_LOG_INTERVAL.
        if (now - self._telemetry_last_log) < self.TELEMETRY_LOG_INTERVAL:
            return
        self._telemetry_last_log = now
        # Baca state telemetri drone dari state_dict (thread-safe baca sederhana).
        pos   = self.state_dict.get("position", {})
        att   = self.state_dict.get("attitude", {})
        vel   = self.state_dict.get("velocity", {})
        lat   = pos.get("lat", 0.0)
        lon   = pos.get("lon", 0.0)
        alt   = pos.get("alt", 0.0)
        yaw   = att.get("yaw", 0.0)
        batt  = self.state_dict.get("battery_pct", 0.0)
        # Kecepatan aktual NED dari telemetri Pixhawk.
        act_vn = vel.get("vn", 0.0)
        act_ve = vel.get("ve", 0.0)
        act_vz = vel.get("vd", 0.0)   # Pixhawk reports vd (down), same sign as NED vz
        logger.info(
            "TEL \u2192 state=%s lat=%.5f lon=%.5f alt=%.1f yaw=%.0f "
            "bbox=%.2f err=(%.2f,%.2f) centered=%s "
            "actual_vn=%.2f actual_ve=%.2f actual_vz=%.2f "
            "cmd_vn=%.2f cmd_ve=%.2f cmd_vz=%.2f batt=%.0f%%",
            scout_state,
            lat, lon, alt, yaw,
            bbox_ratio, err_x, err_y, is_centered,
            act_vn, act_ve, act_vz,
            cmd_vn, cmd_ve, cmd_vz,
            batt,
        )
    # current_state adalah property read-only untuk membaca state autonomy sebagai string.
    @property
    def current_state(self) -> str:
        # Jika scout aktif, state dikembalikan dengan prefix SCOUT_; jika tidak, gunakan state standar.
        return f"SCOUT_{self._scout_state.name}" if self._scout_mode else self._state.name
    # is_running adalah property read-only untuk mengetahui apakah autonomy utama sedang aktif.
    @property
    # Nilai boolean ini biasanya dipakai oleh API/frontend untuk membaca status sistem.
    def is_running(self) -> bool: return self._running
