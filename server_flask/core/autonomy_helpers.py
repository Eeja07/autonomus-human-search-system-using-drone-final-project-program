#!/usr/bin/env python3
# Shebang ini memberi tahu sistem Unix/Linux bahwa file dijalankan menggunakan interpreter Python 3.
"""
autonomy_helpers.py
===================
Fungsi-fungsi pembantu independen untuk:
 - Penghitungan Jarak (Haversine)
 - Bounding Box (BBox) Parsing
 - Smoothing EMA & Hitungan Kecepatan
 - Pengambilan Screenshot Overlay YOLO
"""

# math digunakan untuk operasi matematika seperti trigonometri, akar kuadrat, dan konversi sudut.
import math
# os digunakan untuk operasi sistem file, terutama membuat folder dan menggabungkan path file screenshot.
import os
# logging digunakan untuk mencatat error atau informasi proses helper.
import logging
# time digunakan untuk timestamp target lock agar sistem tahu kapan target terakhir terlihat.
import time
# datetime digunakan untuk membuat timestamp pada nama file screenshot dan teks overlay waktu.
from datetime import datetime
# Optional dan Dict digunakan sebagai type hint agar input/output fungsi lebih jelas.
from typing import Optional, Dict

# logger adalah objek pencatat log khusus modul ini, sehingga pesan log berasal dari nama file/modul yang benar.
logger = logging.getLogger(__name__)

# Blok try dipakai karena OpenCV adalah dependensi opsional untuk fitur screenshot.
try:
    # cv2 adalah library OpenCV untuk membaca/menulis gambar dan menggambar overlay bounding box.
    # pyrefly: ignore [missing-import]
    import cv2
# Jika OpenCV belum terinstall, program tidak langsung crash; hanya fitur screenshot yang tidak bisa digunakan.
except ImportError:
    # Nilai None menjadi penanda bahwa OpenCV tidak tersedia.
    cv2 = None


# Kelas ini berisi fungsi-fungsi pembantu yang nantinya diwarisi oleh AutonomyBase atau controller autonomy lain.
class AutonomyHelpers:
    """Kelas Helper yang di-inherit oleh AutonomyBase."""

    # Fungsi ini menghitung jarak permukaan bumi antara dua titik GPS menggunakan rumus Haversine.
    def _haversine_distance_m(
        # lat1/lon1 adalah koordinat titik pertama, lat2/lon2 adalah koordinat titik kedua.
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> float:
        """Jarak dalam meter antara dua koordinat GPS."""
        # R adalah jari-jari rata-rata bumi dalam meter.
        R = 6_371_000.0
        # dlat adalah selisih latitude yang dikonversi dari derajat ke radian.
        dlat = math.radians(lat2 - lat1)
        # dlon adalah selisih longitude yang dikonversi dari derajat ke radian.
        dlon = math.radians(lon2 - lon1)
        # a adalah komponen utama rumus Haversine untuk menghitung jarak sudut antar dua koordinat.
        a = (
            # Bagian pertama menghitung kontribusi selisih latitude.
            math.sin(dlat / 2) ** 2
            # Bagian berikutnya menghitung kontribusi selisih longitude dengan koreksi posisi latitude.
            + math.cos(math.radians(lat1))
            * math.cos(math.radians(lat2))
            * math.sin(dlon / 2) ** 2
        )
        # Output berupa jarak meter: radius bumi dikali sudut pusat antara dua titik.
        return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    # Fungsi ini menghitung perintah kecepatan NED agar drone bergerak menuju titik home.
    def _compute_home_velocity(
        # cur_lat/cur_lon adalah posisi saat ini, home_lat/home_lon adalah target pulang.
        self, cur_lat: float, cur_lon: float, home_lat: float, home_lon: float,
        # current_yaw adalah arah drone saat ini, max_speed adalah batas maksimum kecepatan horizontal.
        current_yaw: float, max_speed: float
    ):
        """Hitung kecepatan NED menuju home. Return (vx, vy, dist_m, heading_deg)."""
        # Konversi kasar 1 derajat latitude menjadi meter.
        lat_to_m = 111_320.0
        # Konversi longitude ke meter dikoreksi dengan cos(latitude) karena jarak antar longitude berubah menurut latitude.
        lon_to_m = 111_320.0 * math.cos(math.radians(cur_lat))
        # north_err adalah jarak target home terhadap posisi sekarang pada sumbu utara-selatan.
        north_err = (home_lat - cur_lat) * lat_to_m
        # east_err adalah jarak target home terhadap posisi sekarang pada sumbu timur-barat.
        east_err = (home_lon - cur_lon) * lon_to_m
        # dist adalah jarak horizontal dari posisi sekarang ke home dalam meter.
        dist = math.sqrt(north_err ** 2 + east_err ** 2)

        # Jika jarak sangat kecil, drone dianggap sudah sampai sehingga kecepatan horizontal dibuat nol.
        if dist < 0.1:
            # Mengembalikan vx=0, vy=0, jarak aktual, dan yaw tetap memakai yaw saat ini.
            return 0.0, 0.0, dist, float(current_yaw)

        # speed disesuaikan proporsional terhadap jarak, dibatasi minimum 0.3 dan maksimum max_speed.
        speed = max(0.3, min(max_speed, 0.5 * dist))
        # vx adalah komponen kecepatan ke arah utara dalam frame NED.
        vx = (north_err / dist) * speed
        # vy adalah komponen kecepatan ke arah timur dalam frame NED.
        vy = (east_err / dist) * speed
        # hdg adalah heading/yaw menuju home dalam derajat, dihitung dari error timur dan utara.
        hdg = math.degrees(math.atan2(east_err, north_err))
        
        # Normalisasi heading agar tidak lebih dari 180 derajat.
        while hdg > 180.0: hdg -= 360.0
        # Normalisasi heading agar tidak kurang dari -180 derajat.
        while hdg < -180.0: hdg += 360.0
        
        # Output berupa velocity north, velocity east, jarak ke home, dan heading target.
        return vx, vy, dist, hdg

    # Fungsi ini memeriksa apakah satu data deteksi merepresentasikan kelas "person".
    def _is_person_detection(self, detection) -> bool:
        # Deteksi berbasis object dianggap person jika punya class_id dan nilainya 0, sesuai indeks umum COCO untuk person.
        if hasattr(detection, "class_id") and detection.class_id == 0:
            # True berarti detection ini valid sebagai person.
            return True
        # Deteksi berbasis object juga bisa dikenali melalui class_name bernilai "person".
        if hasattr(detection, "class_name") and str(detection.class_name).lower() == "person":
            # True dikembalikan jika nama kelas cocok setelah dibuat lowercase.
            return True
        # Beberapa pipeline mungkin mengirim detection sebagai dictionary, bukan object.
        if isinstance(detection, dict):
            # Pada format dict, class_id 0 juga dianggap person.
            if detection.get("class_id") == 0:
                # True berarti dict detection adalah person.
                return True
            # Pada format dict, class_name "person" juga dianggap person.
            if str(detection.get("class_name", "")).lower() == "person":
                # True berarti nama kelas dict cocok dengan person.
                return True
        # Jika semua kondisi gagal, detection bukan person.
        return False

    # Fungsi ini mengubah satu detection person menjadi kandidat target dengan koordinat, ukuran, dan skor pendukung.
    def _person_candidate_from_detection(self, detection, frame_w: int, frame_h: int) -> Optional[Dict]:
        # Jika detection bukan person, fungsi berhenti dan tidak menghasilkan kandidat.
        if not self._is_person_detection(detection):
            # None berarti detection diabaikan.
            return None

        # Format pertama: object detection memiliki atribut bbox.
        if hasattr(detection, "bbox"):
            # bbox diasumsikan berisi x1, y1, x2, y2.
            x1, y1, x2, y2 = detection.bbox
            # confidence diambil dari object; default 0.0 jika tidak tersedia.
            conf = getattr(detection, "confidence", 0.0)
        # Format kedua: detection berupa dict dan memiliki key bbox.
        elif isinstance(detection, dict) and "bbox" in detection:
            # bbox diambil dari dictionary.
            x1, y1, x2, y2 = detection["bbox"]
            # confidence diambil dari dictionary; default 0.0 jika tidak ada.
            conf = detection.get("confidence", 0.0)
        # Format ketiga: object detection memiliki atribut koordinat x1, y1, x2, y2 terpisah.
        elif hasattr(detection, "x1"):
            # Koordinat bbox diambil langsung dari atribut object.
            x1, y1, x2, y2 = detection.x1, detection.y1, detection.x2, detection.y2
            # confidence diambil dari object; default 0.0 jika tidak tersedia.
            conf = getattr(detection, "confidence", 0.0)
        # Jika format detection tidak dikenali, kandidat tidak bisa dibuat.
        else:
            # None berarti parsing bbox gagal atau format input tidak didukung.
            return None

        # YOLOConsumer memakai bbox normal 0..1. Beberapa input lama mungkin sudah pixel.
        # normalized menjadi True jika koordinat x2 dan y2 terlihat seperti koordinat normalisasi 0 sampai 1.
        normalized = x2 <= 1.5 and y2 <= 1.5
        # Jika bbox normalized, koordinat perlu dikonversi ke pixel memakai ukuran frame.
        if normalized:
            # Menyimpan koordinat normalized sebagai float.
            x1_n, y1_n, x2_n, y2_n = float(x1), float(y1), float(x2), float(y2)
            # Mengubah koordinat X dari normalisasi menjadi pixel.
            x1_px, x2_px = x1_n * frame_w, x2_n * frame_w
            # Mengubah koordinat Y dari normalisasi menjadi pixel.
            y1_px, y2_px = y1_n * frame_h, y2_n * frame_h
        # Jika bbox sudah dalam pixel, koordinat pixel dipakai langsung lalu dibuat versi normalized.
        else:
            # Menyimpan koordinat pixel sebagai float.
            x1_px, y1_px, x2_px, y2_px = float(x1), float(y1), float(x2), float(y2)
            # Mengubah koordinat X pixel menjadi rasio terhadap lebar frame.
            x1_n, x2_n = x1_px / frame_w, x2_px / frame_w
            # Mengubah koordinat Y pixel menjadi rasio terhadap tinggi frame.
            y1_n, y2_n = y1_px / frame_h, y2_px / frame_h

        # width_px adalah lebar bbox dalam pixel, dibatasi minimal 0 agar tidak negatif.
        width_px = max(0.0, x2_px - x1_px)
        # height_px adalah tinggi bbox dalam pixel, dibatasi minimal 0 agar tidak negatif.
        height_px = max(0.0, y2_px - y1_px)
        # width_n adalah lebar bbox dalam koordinat normalized.
        width_n = max(0.0, x2_n - x1_n)
        # height_n adalah tinggi bbox dalam koordinat normalized.
        height_n = max(0.0, y2_n - y1_n)
        # area_n adalah luas bbox normalized, dipakai sebagai indikator ukuran target pada frame.
        area_n = width_n * height_n

        # Jika bbox tidak memiliki lebar atau tinggi valid, detection dianggap tidak layak.
        if width_px <= 0.0 or height_px <= 0.0:
            # None menandakan kandidat gagal dibuat karena bbox invalid.
            return None

        # cx_n adalah titik tengah bbox pada sumbu X dalam koordinat normalized.
        cx_n = x1_n + width_n / 2.0
        # cy_n adalah titik tengah bbox pada sumbu Y dalam koordinat normalized.
        cy_n = y1_n + height_n / 2.0
        # center_dist adalah jarak pusat bbox ke pusat frame, dinormalisasi agar mudah dibandingkan.
        center_dist = math.sqrt((cx_n - 0.5) ** 2 + (cy_n - 0.5) ** 2) / math.sqrt(0.5)
        # center_score semakin tinggi jika target semakin dekat ke pusat frame.
        center_score = max(0.0, 1.0 - center_dist)
        # area_score semakin tinggi jika bbox semakin besar, dibatasi maksimal 1.0.
        area_score = min(1.0, math.sqrt(max(0.0, area_n)) * 2.0)

        # Dictionary kandidat berisi data mentah, koordinat pixel, koordinat normalized, confidence, dan skor pendukung.
        return {
            # raw menyimpan detection asli agar informasi awal tetap bisa ditelusuri.
            "raw": detection,
            # x adalah koordinat kiri bbox dalam pixel.
            "x": float(x1_px),
            # y adalah koordinat atas bbox dalam pixel.
            "y": float(y1_px),
            # width adalah lebar bbox dalam pixel.
            "width": float(width_px),
            # height adalah tinggi bbox dalam pixel.
            "height": float(height_px),
            # cx adalah titik tengah bbox pada sumbu X dalam pixel.
            "cx": float(x1_px + width_px / 2.0),
            # cy adalah titik tengah bbox pada sumbu Y dalam pixel.
            "cy": float(y1_px + height_px / 2.0),
            # cx_norm adalah titik tengah X dalam koordinat normalized.
            "cx_norm": float(cx_n),
            # cy_norm adalah titik tengah Y dalam koordinat normalized.
            "cy_norm": float(cy_n),
            # area adalah luas bbox normalized.
            "area": float(area_n),
            # confidence adalah tingkat keyakinan model deteksi.
            "confidence": float(conf),
            # center_score adalah skor kedekatan target terhadap pusat frame.
            "center_score": float(center_score),
            # area_score adalah skor ukuran bbox.
            "area_score": float(area_score),
        }

    # Fungsi ini mengubah daftar detection menjadi daftar kandidat person yang valid.
    def _get_person_candidates(self, detections: list, frame_w: int, frame_h: int) -> list:
        # candidates menampung semua kandidat person yang berhasil diparse.
        candidates = []
        # Loop membaca setiap detection; "or []" membuat fungsi aman jika detections bernilai None.
        for detection in detections or []:
            # Membuat kandidat dari satu detection berdasarkan ukuran frame.
            candidate = self._person_candidate_from_detection(detection, frame_w, frame_h)
            # Hanya kandidat valid yang dimasukkan ke list.
            if candidate is not None:
                # Menambahkan kandidat person valid ke daftar kandidat.
                candidates.append(candidate)
        # Output berupa list kandidat person yang siap diberi skor atau dipilih sebagai target.
        return candidates

    # Fungsi ini menghitung jarak posisi pusat dua target dalam koordinat normalized.
    def _target_distance(self, a: Dict, b: Dict) -> float:
        # Jarak Euclidean dihitung dari selisih cx_norm dan cy_norm antara kandidat a dan b.
        return math.sqrt((a["cx_norm"] - b["cx_norm"]) ** 2 + (a["cy_norm"] - b["cy_norm"]) ** 2)

    # Fungsi ini menghitung skor akhir kandidat person untuk menentukan target terbaik.
    def _score_person_candidate(self, candidate: Dict, locked_target: Optional[Dict] = None) -> float:
        # consistency_score memberi bonus jika kandidat dekat dengan target yang sedang dikunci.
        consistency_score = 0.0
        # Jika ada locked_target, kandidat dinilai berdasarkan konsistensinya terhadap target lama.
        if locked_target:
            # dist adalah jarak kandidat terhadap target lock lama.
            dist = self._target_distance(candidate, locked_target)
            # max_dist adalah batas jarak match target; default 0.35 jika atribut belum didefinisikan.
            max_dist = getattr(self, "TARGET_MATCH_MAX_DIST", 0.35)
            # consistency_score makin besar jika jarak kandidat makin dekat dengan locked_target.
            consistency_score = max(0.0, 1.0 - (dist / max_dist))

        # Skor akhir merupakan kombinasi ukuran bbox, confidence deteksi, posisi tengah frame, dan konsistensi target.
        return (
            # area_score mendapat bobot terbesar karena target besar biasanya lebih relevan/dekat.
            0.45 * candidate["area_score"]
            # confidence memberi kontribusi berdasarkan keyakinan model YOLO.
            + 0.25 * candidate["confidence"]
            # center_score membantu memilih target yang berada dekat pusat frame.
            + 0.20 * candidate["center_score"]
            # consistency_score mencegah target berpindah-pindah saat ada beberapa orang.
            + 0.10 * consistency_score
        )

    # Fungsi ini memilih satu target person terbaik dari list deteksi dengan mekanisme scoring dan target lock.
    def _select_person_target(self, detections: list, frame_w: int, frame_h: int, now: Optional[float] = None) -> Optional[Dict]:
        """
        Pilih target person dengan scoring + target lock.
        Ini mencegah drone pindah-pindah orang saat beberapa bbox ukurannya mirip.
        """
        # now berisi waktu saat ini; jika tidak diberikan, gunakan time.time().
        now = time.time() if now is None else now
        # Mengubah detection mentah menjadi kandidat person valid.
        candidates = self._get_person_candidates(detections, frame_w, frame_h)
        # max_age adalah durasi maksimum target lock boleh dipertahankan tanpa terlihat.
        max_age = getattr(self, "TARGET_LOCK_MAX_AGE_S", 1.5)
        # max_dist adalah jarak maksimum agar kandidat dianggap masih target yang sama.
        max_dist = getattr(self, "TARGET_MATCH_MAX_DIST", 0.35)
        # switch_ratio adalah rasio skor minimal agar sistem mau berpindah ke target baru.
        switch_ratio = getattr(self, "TARGET_SWITCH_SCORE_RATIO", 1.35)

        # Jika tidak ada kandidat person pada frame saat ini, target tidak bisa dipilih.
        if not candidates:
            # Jika target lock lama sudah terlalu lama tidak terlihat, lock dihapus.
            if self._target_lock and (now - self._target_lock.get("last_seen", now)) > max_age:
                # Reset target lock agar pemilihan berikutnya tidak terikat target lama.
                self._target_lock = None
            # None berarti tidak ada target aktif untuk frame ini.
            return None

        # locked menyimpan target yang sedang dikunci dari frame sebelumnya.
        locked = self._target_lock
        # Memberi skor pada setiap kandidat person.
        for candidate in candidates:
            # Skor dihitung dengan mempertimbangkan target lock jika ada.
            candidate["score"] = self._score_person_candidate(candidate, locked)

        # best adalah kandidat dengan skor tertinggi pada frame saat ini.
        best = max(candidates, key=lambda c: c["score"])
        # Jika belum ada target lock, kandidat terbaik langsung dijadikan target baru.
        if locked is None:
            # locked_at mencatat kapan target mulai dikunci.
            best["locked_at"] = now
            # last_seen mencatat kapan target terakhir terlihat.
            best["last_seen"] = now
            # Menyimpan salinan kandidat terbaik sebagai target lock internal.
            self._target_lock = best.copy()
            # TEST_G: Log initial target lock.
            logger.info(
                "TEST_G → lock_init cx=%.3f cy=%.3f score=%.3f conf=%.3f area=%.4f",
                best["cx_norm"], best["cy_norm"], best.get("score", 0), best["confidence"], best["area"]
            )
            if hasattr(self, '_test_lock_start_time'):
                self._test_lock_start_time = now
            # Mengembalikan kandidat terbaik sebagai target aktif.
            return best

        # matches berisi kandidat yang jaraknya masih dekat dengan target lock lama.
        matches = [c for c in candidates if self._target_distance(c, locked) <= max_dist]
        # matched memilih kandidat match dengan skor tertinggi; jika tidak ada match, nilainya None.
        matched = max(matches, key=lambda c: c["score"]) if matches else None

        # Jika ada kandidat yang cocok dengan target lock lama, sistem mempertahankan kontinuitas target.
        if matched is not None:
            # locked_at dipertahankan dari target lama agar durasi lock tetap konsisten.
            matched["locked_at"] = locked.get("locked_at", now)
            # last_seen diperbarui karena target terlihat pada frame sekarang.
            matched["last_seen"] = now

            # Ganti target hanya kalau kandidat baru jelas lebih kuat.
            # Kondisi ini mencegah perpindahan target kecuali best jauh lebih baik daripada target yang match.
            if best is not matched and best["score"] > matched["score"] * switch_ratio:
                # TEST_G: Log target switch (lock berubah ke kandidat baru).
                lock_duration = now - locked.get("locked_at", now)
                logger.info(
                    "TEST_G → lock_switch old_score=%.3f new_score=%.3f duration=%.2fs reason=score_superior",
                    matched["score"], best["score"], lock_duration
                )
                if hasattr(self, '_test_lock_switch_count'):
                    self._test_lock_switch_count += 1
                    self._test_lock_start_time = now
                # Target baru mulai dikunci pada waktu sekarang.
                best["locked_at"] = now
                # Target baru juga dianggap terakhir terlihat pada waktu sekarang.
                best["last_seen"] = now
                # Mengganti target lock ke kandidat terbaik baru.
                self._target_lock = best.copy()
                # Mengembalikan target baru sebagai hasil seleksi.
                return best

            # Jika tidak ada alasan kuat untuk pindah target, target lock diperbarui dengan matched.
            self._target_lock = matched.copy()
            # Mengembalikan kandidat matched sebagai target aktif.
            return matched

        # Jika tidak ada kandidat yang cocok tetapi target lama belum kadaluarsa, sistem menunggu agar tidak mudah pindah target.
        if (now - locked.get("last_seen", now)) <= max_age:
            # None berarti pada frame ini belum ada target baru yang diambil karena lock lama masih diberi toleransi.
            return None

        # Jika target lama sudah kadaluarsa, kandidat terbaik dijadikan target lock baru.
        best["locked_at"] = now
        # last_seen target baru diisi waktu sekarang.
        best["last_seen"] = now
        # TEST_G: Log target lock expired and replaced.
        lock_duration = now - locked.get("locked_at", now)
        logger.info(
            "TEST_G → lock_expired old_duration=%.2fs new_score=%.3f",
            lock_duration, best.get("score", 0)
        )
        if hasattr(self, '_test_lock_switch_count'):
            self._test_lock_switch_count += 1
            self._test_lock_start_time = now
        # Menyimpan target baru sebagai target lock.
        self._target_lock = best.copy()
        # Mengembalikan target baru.
        return best

    # Fungsi ini menghapus target lock person yang sedang disimpan.
    def _clear_person_target_lock(self):
        # Setelah direset, seleksi target berikutnya dimulai tanpa preferensi target lama.
        self._target_lock = None

    # Fungsi ini mencari bounding box person terbesar dari daftar deteksi.
    def _get_largest_person_bbox(self, detections: list, frame_w: int, frame_h: int) -> Optional[Dict]:
        """Kembalikan bounding box orang terbesar dari list deteksi (aman dari error object/dict)."""
        # Mengambil semua kandidat person valid dari detection mentah.
        candidates = self._get_person_candidates(detections, frame_w, frame_h)
        # Jika tidak ada kandidat person, fungsi tidak bisa mengembalikan bbox.
        if not candidates:
            # None berarti tidak ada person valid.
            return None
        # Mengembalikan kandidat dengan luas pixel terbesar.
        return max(candidates, key=lambda d: d["width"] * d["height"])

    # Fungsi ini menghitung error posisi titik target terhadap pusat frame kamera.
    def _calculate_frame_error(self, cx: float, cy: float, w: int, h: int) -> Dict:
        """Hitung error piksel dan ternormalisasi dari pusat frame."""
        # err_x positif berarti target berada di kanan pusat frame, negatif berarti di kiri.
        err_x = cx - (w / 2)
        # err_y positif berarti target berada di bawah pusat frame, negatif berarti di atas.
        err_y = cy - (h / 2)
        # Dictionary output memuat error dalam pixel dan dalam bentuk normalisasi -1 sampai 1 relatif pusat frame.
        return {
            # Error horizontal dalam pixel.
            "error_x": err_x,
            # Error vertikal dalam pixel.
            "error_y": err_y,
            # Error horizontal normalized terhadap setengah lebar frame.
            "error_x_normalized": err_x / (w / 2),
            # Error vertikal normalized terhadap setengah tinggi frame.
            "error_y_normalized": err_y / (h / 2),
        }

    # Fungsi ini menghitung perintah velocity horizontal dan yaw target berdasarkan error kamera.
    def _calculate_velocity_command(
        # error berasal dari _calculate_frame_error, current_yaw adalah yaw saat ini, kp_yaw/kp_fwd adalah gain kontrol.
        self, error: Dict, current_yaw: float, kp_yaw: float, kp_fwd: float, max_vel: float
    ) -> tuple:
        """Hitung (vx, vy, yaw_target) dari frame error."""
        # yaw_t dihitung dari yaw saat ini ditambah koreksi proporsional error horizontal.
        yaw_t = current_yaw + (kp_yaw * error["error_x_normalized"])
        # Normalisasi yaw target agar berada pada rentang -180 sampai 180 derajat.
        while yaw_t > 180.0: yaw_t -= 360.0
        # Normalisasi yaw target jika lebih kecil dari -180 derajat.
        while yaw_t < -180.0: yaw_t += 360.0

        # fwd_speed dihitung dari error vertikal frame dengan gain kp_fwd.
        fwd_speed = kp_fwd * error["error_y_normalized"]
        # fwd_speed dibatasi agar tidak melebihi batas maksimum velocity.
        fwd_speed = max(-max_vel, min(max_vel, fwd_speed))

        # Mengubah current_yaw dari derajat ke radian untuk fungsi cos/sin.
        yaw_rad = math.radians(current_yaw)
        # vx adalah komponen kecepatan north berdasarkan arah yaw saat ini.
        vx = fwd_speed * math.cos(yaw_rad)
        # vy adalah komponen kecepatan east berdasarkan arah yaw saat ini.
        vy = fwd_speed * math.sin(yaw_rad)

        # Output tuple berisi velocity north, velocity east, dan yaw target.
        return (vx, vy, yaw_t)

    # Fungsi ini mengambil screenshot dari frame kamera, menggambar overlay deteksi, lalu menyimpannya ke folder foto.
    def _take_scout_screenshot(self, cam, latest_detection, pos: dict, photos_dir: str, frame_w: int, frame_h: int, snapshot_frame=None) -> Optional[str]:
        # Jika OpenCV tidak tersedia, fitur screenshot tidak bisa berjalan.
        if cv2 is None:
            # Mencatat error agar penyebab kegagalan terlihat di log.
            logger.error("Screenshot gagal: OpenCV tidak tersedia")
            # None menjadi output gagal karena file screenshot tidak dibuat.
            return None

        # frame akan menampung gambar kamera yang akan diberi overlay.
        frame = None
        # 1. Prioritaskan snapshot frame yang disimpan saat target centered (lebih akurat posisi BBox-nya).
        if snapshot_frame is not None:
            frame = snapshot_frame.copy()
        # 2. Jika tidak ada snapshot, ambil frame dari sumber kamera.
        elif cam is not None:
            # Format kamera pertama: object punya atribut latest_frame.
            if hasattr(cam, 'latest_frame'):
                # Gunakan .copy() agar frame asli di pipeline tidak terganggu saat kita menggambar overlay
                # Jika latest_frame ada, salinan frame dipakai agar gambar asli pipeline tetap aman.
                frame = cam.latest_frame.copy() if cam.latest_frame is not None else None
            # Format kamera kedua: object punya method read seperti cv2.VideoCapture.
            elif hasattr(cam, 'read'): 
                # read() mengembalikan status keberhasilan dan frame sementara.
                ret, tmp_frame = cam.read()
                # Jika pembacaan berhasil, tmp_frame dipakai sebagai frame screenshot.
                if ret: frame = tmp_frame

        # Jika frame tetap None, berarti sumber kamera tidak memberikan gambar valid.
        if frame is None:
            # Mencatat kegagalan agar mudah didiagnosis.
            logger.error("Screenshot gagal: Frame tidak tersedia")
            # None dikembalikan karena tidak ada file yang bisa dibuat.
            return None

        # Blok try melindungi proses pembuatan folder, overlay, dan penulisan file dari exception.
        try:
            # Membuat folder photos_dir jika belum ada; exist_ok=True mencegah error jika folder sudah ada.
            os.makedirs(photos_dir, exist_ok=True)

            # 2. Ambil list deteksi
            # Mengambil deteksi terbaru jika object latest_detection memiliki atribut detections.
            dets = latest_detection.detections if latest_detection and hasattr(latest_detection, "detections") else []

            # 3. Gambar Bounding Box (Overlay YOLO)
            # Loop ini menggambar setiap bbox deteksi ke frame screenshot.
            for det in dets:
                # Overlay hanya dibuat untuk deteksi yang memiliki atribut bbox.
                if hasattr(det, "bbox"):
                    # Mengambil koordinat bbox dari object deteksi.
                    x1, y1, x2, y2 = det.bbox
                    # Normalisasi koordinat jika masih dalam format 0.0 - 1.0
                    # Jika x2 <= 1.5, bbox diasumsikan normalized sehingga perlu dikali ukuran frame.
                    if x2 <= 1.5:
                        # Konversi koordinat normalized ke pixel untuk titik kiri-atas.
                        x1 = int(x1 * frame_w); y1 = int(y1 * frame_h)
                        # Konversi koordinat normalized ke pixel untuk titik kanan-bawah.
                        x2 = int(x2 * frame_w); y2 = int(y2 * frame_h)
                    # Jika bbox sudah pixel, cukup ubah nilainya ke integer.
                    else:
                        # OpenCV membutuhkan koordinat integer untuk menggambar rectangle.
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        
                    # Menggambar kotak hijau pada area bbox deteksi.
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # Membuat label berisi nama kelas dan confidence deteksi.
                    lbl = f"{getattr(det, 'class_name', '?')} {det.confidence:.2f}"
                    # Menulis label di atas bounding box dengan warna hijau.
                    cv2.putText(frame, lbl, (x1, max(y1 - 8, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # 4. Tambahkan Informasi Telemetri (GPS & Waktu)
            # Menulis koordinat GPS pada bagian kiri atas gambar.
            cv2.putText(frame, f"GPS: {pos.get('lat',0):.6f}, {pos.get('lon',0):.6f}", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            # Menulis waktu saat screenshot dibuat pada bagian kiri atas gambar.
            cv2.putText(frame, datetime.now().strftime("%Y-%m-%d %H:%M:%S"), (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

            # 5. Simpan ke File
            # Nama file dibuat dari prefix scout dan timestamp agar tidak mudah bertabrakan dengan file lain.
            fname = f"scout_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
            # Menulis frame yang sudah diberi overlay ke file JPG di photos_dir.
            cv2.imwrite(os.path.join(photos_dir, fname), frame)
            # Mengembalikan nama file sebagai output sukses.
            return fname
        # Jika terjadi error saat membuat screenshot, exception ditangkap agar program utama tidak crash.
        except Exception as e:
            # Mencatat pesan error screenshot untuk debugging.
            logger.error("Screenshot error: %s", e)
            # None berarti screenshot gagal dibuat.
            return None
