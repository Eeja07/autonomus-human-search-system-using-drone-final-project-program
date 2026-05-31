#!/usr/bin/env python3
"""
YOLO Consumer
=============
Reads frames from GStreamer appsink queue.
Runs YOLO inference.
Puts results into detection_queue for AutonomyController.

NO OpenCV capture. NO independent thread beside own worker.
"""

import logging
import time
import threading
from dataclasses import dataclass, field
from pathlib import Path
from queue import Queue, Empty
from typing import List, Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class Detection:
    """Single YOLO detection result."""
    class_name: str
    confidence: float
    bbox: List[float]        # [x1, y1, x2, y2] normalized 0..1
    cx: float = 0.0          # center x normalized
    cy: float = 0.0          # center y normalized
    area: float = 0.0        # bbox area normalized
    class_id: int = 0        # TEST_B: class id untuk analisis per-kelas


@dataclass
class DetectionFrame:
    """Output from YOLO: one frame worth of detections."""
    detections: List[Detection] = field(default_factory=list)
    timestamp: float = 0.0
    inference_ms: float = 0.0
    fps: float = 0.0
    # TEST_C: timestamp saat frame ditangkap oleh GStreamer (t0) untuk pipeline latency analysis.
    capture_timestamp: float = 0.0
    # TEST_B: dimensi frame untuk kalkulasi bbox ratio di log.
    frame_width: int = 320
    frame_height: int = 320


class YOLOConsumer:
    """
    Reads frames from gst_pipeline.frame_queue.
    Runs YOLOv8 inference.
    Puts DetectionFrame into self.detection_queue.
    """

    DETECTION_QUEUE_SIZE = 2

    def __init__(
        self,
        frame_queue: Queue,
        model_path: str,
        conf_threshold: float = 0.5,
        iou_threshold: float = 0.45,
        target_classes: Optional[List[str]] = None,
    ):
        self.frame_queue = frame_queue
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.target_classes = target_classes or ["person"]

        # Output queue consumed by AutonomyController
        self.detection_queue: Queue = Queue(maxsize=self.DETECTION_QUEUE_SIZE)

        self._model = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Stats
        self._fps = 0.0
        self._frame_times = []
        self.latest_result = None  # <--- TAMBAHKAN BARIS INI
    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        if self._running:
            return
        self._load_model()
        self._running = True
        self._thread = threading.Thread(
            target=self._worker, daemon=True, name="yolo-consumer"
        )
        self._thread.start()
        logger.info("YOLOConsumer started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        logger.info("YOLOConsumer stopped")

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def running(self) -> bool:
        return self._running

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _load_model(self):
        """Load YOLO model (ONNX, PyTorch, or TFLite)."""
        path = Path(self.model_path)
        if not path.exists():
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")

        if path.suffix == ".onnx":
            self._load_onnx(str(path))
        elif path.suffix == ".tflite":
            self._load_tflite(str(path))
        else:
            self._load_pytorch(str(path))
    def _load_tflite(self, path: str):
        try:
            # Gunakan tflite_runtime jika ada, jika tidak fallback ke tensorflow.lite
            # pyrefly: ignore [missing-import]
            import tflite_runtime.interpreter as tflite
        except ImportError:
            # pyrefly: ignore [missing-import]
            import tensorflow.lite as tflite

        self._interpreter = tflite.Interpreter(model_path=path)
        self._interpreter.allocate_tensors()
        
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        
        # YOLOv8 TFLite export biasanya menggunakan format NHWC (Batch, Height, Width, Channels)
        shape = self._input_details[0]['shape'] 
        self._input_h = shape[1]
        self._input_w = shape[2]
        
        self._backend = "tflite"
        self._model = "tflite_loaded"
        logger.info("YOLO TFLite loaded: %s input=%dx%d", path, self._input_w, self._input_h)
    def _load_onnx(self, path: str):
        # pyrefly: ignore [missing-import]
        import onnxruntime as ort

        providers = ["CPUExecutionProvider"]
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 4
        opts.inter_op_num_threads = 1
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        self._session = ort.InferenceSession(path, sess_options=opts, providers=providers)
        self._input_name = self._session.get_inputs()[0].name
        shape = self._session.get_inputs()[0].shape  # [1, 3, h, w]
        self._input_h = shape[2]
        self._input_w = shape[3]
        self._backend = "onnx"
        self._model = "onnx_loaded"
        logger.info("YOLO ONNX loaded: %s input=%dx%d", path, self._input_w, self._input_h)

    def _load_pytorch(self, path: str):
        # pyrefly: ignore [missing-import]
        from ultralytics import YOLO
        self._pt_model = YOLO(path)
        self._backend = "pytorch"
        self._model = "pytorch_loaded"
        logger.info("YOLO PyTorch loaded: %s", path)

    def _worker(self):
        """Main worker: read frame → infer → put result."""
        logger.info("YOLO worker thread running")
        _t_last = time.time()
        # TEST_B: Counter kumulatif untuk analisis akurasi deteksi in-flight.
        self._total_frames_processed = 0
        self._total_detections = 0
        # TEST_C: Akumulasi latency untuk rata-rata per 50 frame.
        self._latency_accum = []

        while self._running:
            # TEST_C: Frame sekarang datang sebagai tuple (frame, t_capture) dari GStreamer.
            # Backward-compatible: jika masih ndarray biasa, t_capture = None.
            try:
                raw_item = self.frame_queue.get(timeout=0.5)
            except Empty:
                continue

            if isinstance(raw_item, tuple) and len(raw_item) == 2:
                frame, t_capture = raw_item
            else:
                frame, t_capture = raw_item, None

            # TEST_C: t1 = frame masuk queue (sudah diparsing), t2 = YOLO mulai inferensi.
            t_queue_read = time.time()
            t0 = time.time()
            try:
                detections = self._run_inference(frame)
            except Exception as e:
                logger.error("YOLO inference error: %s", e)
                continue

            t1 = time.time()
            infer_ms = (t1 - t0) * 1000

            # FPS tracking (rolling window of 10)
            self._frame_times.append(t1)
            if len(self._frame_times) > 10:
                self._frame_times.pop(0)
            if len(self._frame_times) >= 2:
                duration = self._frame_times[-1] - self._frame_times[0]
                if duration > 0:
                    self._fps = (len(self._frame_times) - 1) / duration

            # TEST_B: Update counter kumulatif.
            self._total_frames_processed += 1
            self._total_detections += len(detections)

            # Dapatkan dimensi frame untuk frame_width/frame_height di DetectionFrame.
            fh, fw = frame.shape[:2]

            result = DetectionFrame(
                detections=detections,
                timestamp=t1,
                inference_ms=infer_ms,
                fps=self._fps,
                capture_timestamp=t_capture or t1,
                frame_width=fw,
                frame_height=fh,
            )
            self.latest_result = result

            # TEST_B: Log terstruktur deteksi in-flight untuk analisis akurasi.
            # Format: TEST_B → count, confidence per detection, bbox ratio, FPS aktual.
            if detections:
                for d in detections:
                    bh = d.bbox[3] - d.bbox[1]  # normalized height
                    bw = d.bbox[2] - d.bbox[0]  # normalized width
                    logger.info(
                        "TEST_B → cls=%s conf=%.3f bbox_ratio=%.3f bbox_area=%.4f cx=%.3f cy=%.3f fps=%.1f infer=%.1fms",
                        d.class_name, d.confidence, bh, bw * bh, d.cx, d.cy, self._fps, infer_ms
                    )
            else:
                # TEST_B: Log frame tanpa deteksi (False Negative tracking).
                logger.info("TEST_B → no_detection fps=%.1f infer=%.1fms", self._fps, infer_ms)

            # TEST_C: Log pipeline latency per-stage jika t_capture tersedia.
            if t_capture is not None:
                lat_capture_to_queue = (t_queue_read - t_capture) * 1000
                lat_queue_to_yolo = (t0 - t_queue_read) * 1000
                lat_yolo_infer = infer_ms
                lat_total_to_yolo_out = (t1 - t_capture) * 1000
                self._latency_accum.append({
                    "cap2q": lat_capture_to_queue,
                    "q2yolo": lat_queue_to_yolo,
                    "infer": lat_yolo_infer,
                    "total": lat_total_to_yolo_out,
                })
                # Log setiap 50 frame untuk ringkasan tanpa spam.
                if len(self._latency_accum) >= 50:
                    avg_c2q = sum(e["cap2q"] for e in self._latency_accum) / len(self._latency_accum)
                    avg_q2y = sum(e["q2yolo"] for e in self._latency_accum) / len(self._latency_accum)
                    avg_inf = sum(e["infer"] for e in self._latency_accum) / len(self._latency_accum)
                    avg_tot = sum(e["total"] for e in self._latency_accum) / len(self._latency_accum)
                    min_tot = min(e["total"] for e in self._latency_accum)
                    max_tot = max(e["total"] for e in self._latency_accum)
                    logger.info(
                        "TEST_C_YOLO → avg_capture_to_queue=%.1fms avg_queue_to_yolo=%.1fms "
                        "avg_infer=%.1fms avg_total=%.1fms min=%.1fms max=%.1fms (n=%d)",
                        avg_c2q, avg_q2y, avg_inf, avg_tot, min_tot, max_tot,
                        len(self._latency_accum)
                    )
                    self._latency_accum = []

            # Non-blocking put: drop oldest if full
            try:
                self.detection_queue.put_nowait(result)
            except Exception:
                try:
                    self.detection_queue.get_nowait()
                    self.detection_queue.put_nowait(result)
                except Exception:
                    pass

        # TEST_B: Log ringkasan total saat worker berhenti.
        logger.info(
            "TEST_B_SUMMARY → total_frames=%d total_detections=%d avg_det_per_frame=%.2f",
            self._total_frames_processed, self._total_detections,
            self._total_detections / max(1, self._total_frames_processed)
        )
        logger.info("YOLO worker thread stopped")

    def _run_inference(self, frame: np.ndarray) -> List[Detection]:
        """Run inference on BGR frame, return filtered detections."""
        if self._backend == "onnx":
            return self._infer_onnx(frame)
        elif self._backend == "tflite":
            return self._infer_tflite(frame)
        else:
            return self._infer_pytorch(frame)

    def _infer_onnx(self, frame: np.ndarray) -> List[Detection]:
        """Run ONNX inference."""
        h, w = frame.shape[:2]
        inp = self._preprocess_onnx(frame)

        outputs = self._session.run(None, {self._input_name: inp})
        return self._postprocess_onnx(outputs, w, h)

    def _preprocess_onnx(self, frame: np.ndarray) -> np.ndarray:
        """BGR → RGB → float32 NCHW normalized."""
        # pyrefly: ignore [missing-import]
        import cv2
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (self._input_w, self._input_h))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
        return img

    def _postprocess_onnx(self, outputs, orig_w: int, orig_h: int) -> List[Detection]:
            """Parse YOLOv8 ONNX output and apply NMS."""
            # pyrefly: ignore [missing-import]
            import cv2
            
            # YOLOv8 ONNX: output shape [1, 84, 8400] (4 box + 80 classes)
            pred = outputs[0]  # [1, 84, N]
            pred = pred[0].T   # [N, 84]

            boxes = pred[:, :4]   # cx cy w h (normalized to input size)
            scores = pred[:, 4:]  # [N, 80]

            class_ids = np.argmax(scores, axis=1)
            confidences = np.max(scores, axis=1)

            results = []
            cv_boxes = []
            cv_scores = []
            cv_classes = []
            raw_data = []

            iw, ih = self._input_w, self._input_h

            # 1. Kumpulkan semua tebakan yang melebihi batas confidence
            for i, (box, conf, cls_id) in enumerate(zip(boxes, confidences, class_ids)):
                cls_name = self._class_id_to_name(int(cls_id))
                if conf < self.conf_threshold:
                    continue
                if self.target_classes and cls_name not in self.target_classes:
                    continue

                cx_n, cy_n, bw_n, bh_n = box / np.array([iw, ih, iw, ih])
                
                # Format untuk NMS OpenCV: [x, y, w, h]
                cv_boxes.append([float(cx_n - bw_n/2), float(cy_n - bh_n/2), float(bw_n), float(bh_n)])
                cv_scores.append(float(conf))
                cv_classes.append(cls_name)
                raw_data.append((cx_n, cy_n, bw_n, bh_n))

            # 2. Terapkan Non-Maximum Suppression (NMS) untuk membuang kotak yang menumpuk
            if len(cv_boxes) > 0:
                indices = cv2.dnn.NMSBoxes(cv_boxes, cv_scores, self.conf_threshold, self.iou_threshold)
                
                # 3. Simpan hanya kotak yang lolos seleksi NMS
                for i in indices:
                    idx = i[0] if isinstance(i, (list, tuple, np.ndarray)) else i
                    
                    cx_n, cy_n, bw_n, bh_n = raw_data[idx]
                    x1 = cx_n - bw_n / 2
                    y1 = cy_n - bh_n / 2
                    x2 = cx_n + bw_n / 2
                    y2 = cy_n + bh_n / 2

                    results.append(Detection(
                        class_name=cv_classes[idx],
                        confidence=cv_scores[idx],
                        bbox=[float(x1), float(y1), float(x2), float(y2)],
                        cx=float(cx_n),
                        cy=float(cy_n),
                        area=float(bw_n * bh_n),
                        class_id=int(class_ids[i]) if i < len(class_ids) else 0,
                    ))

            return results
    def _preprocess_tflite(self, frame: np.ndarray) -> np.ndarray:
        """BGR → RGB → float32 NHWC normalized."""
        # pyrefly: ignore [missing-import]
        import cv2
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (self._input_w, self._input_h))
        img = img.astype(np.float32) / 255.0
        # Tidak perlu np.transpose karena TFLite pakai format NHWC (Height, Width, Channels)
        img = np.expand_dims(img, axis=0)
        return img

    def _infer_tflite(self, frame: np.ndarray) -> List[Detection]:
        """Run TFLite inference."""
        h, w = frame.shape[:2]
        inp = self._preprocess_tflite(frame)

        # Set tensor input
        self._interpreter.set_tensor(self._input_details[0]['index'], inp)
        
        # Jalankan inferensi
        self._interpreter.invoke()
        
        # Ambil hasil
        output_data = self._interpreter.get_tensor(self._output_details[0]['index'])
        
        # Gunakan ulang logika postprocess ONNX karena bentuk outputnya sama [1, 84, N]
        return self._postprocess_onnx([output_data], w, h)
    def _infer_pytorch(self, frame: np.ndarray) -> List[Detection]:
        """Run ultralytics YOLO inference."""
        # pyrefly: ignore [missing-import]
        import cv2
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self._pt_model(
            img,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            classes=[0],  # person class
            verbose=False,
        )
        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxyn[0].tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                area = (x2 - x1) * (y2 - y1)
                cls_name = r.names[int(box.cls)]
                if self.target_classes and cls_name not in self.target_classes:
                    continue
                detections.append(Detection(
                    class_name=cls_name,
                    confidence=float(box.conf),
                    bbox=[x1, y1, x2, y2],
                    cx=cx,
                    cy=cy,
                    area=area,
                ))
        return detections

    @staticmethod
    def _class_id_to_name(class_id: int) -> str:
        """COCO class names subset (person=0)."""
        COCO_NAMES = {
            0: "person", 1: "bicycle", 2: "car", 3: "motorcycle",
            4: "airplane", 5: "bus", 6: "train", 7: "truck",
            # ... add more as needed
        }
        return COCO_NAMES.get(class_id, f"class_{class_id}")
