#!/usr/bin/env python3
"""
Monitoring Routes (READ-ONLY)
==============================
Web interface dapat HANYA membaca status.
Tidak ada arm, takeoff, land, start_autonomy.
"""

import time
import cv2
import os
from datetime import datetime

from flask import request, jsonify, Response, send_from_directory

# Definisikan path absolut ke folder photos
# Karena routes.py ada di folder 'web', kita naik satu tingkat ke 'server_flask'
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PHOTOS_PATH = os.path.join(BASE_DIR, "photos")

def register_monitoring_routes(app, get_listener, get_autonomy, get_pipeline=None, get_yolo=None):
    
    # ── STORAGE ROUTES ────────────────────────────────────────────────────────

    @app.route("/api/storage/list", methods=["GET"])
    def list_storage():
        try:
            files_data = []
            
            # 1. Baca folder photos
            if os.path.exists(PHOTOS_PATH):
                for f in os.listdir(PHOTOS_PATH):
                    if f.lower().endswith(('.jpg', '.jpeg', '.png')):
                        files_data.append({
                            "name": f,
                            "type": "photo",
                            "time": os.path.getmtime(os.path.join(PHOTOS_PATH, f)),
                            "size": os.path.getsize(os.path.join(PHOTOS_PATH, f))
                        })
            

            
            # Urutkan berdasarkan waktu (terbaru di atas)
            files_data.sort(key=lambda x: x["time"], reverse=True)
            return jsonify(files_data)
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    @app.route("/api/storage/photos/<filename>")
    def get_photo(filename):
        resp = send_from_directory(PHOTOS_PATH, filename)
        # Paksa header CORS agar lolos dari ORB browser
        resp.headers['Access-Control-Allow-Origin'] = '*'
        return resp


    @app.route("/api/storage/delete/<filename>", methods=["DELETE"])
    def delete_file(filename):
        try:
            deleted = False
            # Cari file di folder photos
            for folder_path in [PHOTOS_PATH]:
                filepath = os.path.join(folder_path, filename)
                if os.path.exists(filepath):
                    os.remove(filepath)
                    deleted = True
                    break
            
            if deleted:
                return jsonify({"success": True})
            else:
                return jsonify({"success": False, "error": "File not found"}), 404
        except Exception as e:
            return jsonify({"success": False, "error": str(e)}), 500

    @app.route("/api/storage/rename/<filename>", methods=["POST"])
    def rename_file(filename):
        try:
            data = request.get_json()
            new_name = data.get("newName")
            if not new_name:
                return jsonify({"success": False, "error": "New name required"}), 400

            renamed = False
            # Cari file di folder photos
            for folder_path in [PHOTOS_PATH]:
                old_filepath = os.path.join(folder_path, filename)
                if os.path.exists(old_filepath):
                    new_filepath = os.path.join(folder_path, new_name)
                    os.rename(old_filepath, new_filepath)
                    renamed = True
                    break

            if renamed:
                return jsonify({"success": True, "filename": new_name})
            else:
                return jsonify({"success": False, "error": "File not found"}), 404
        except Exception as e:
            return jsonify({"success": False, "error": str(e)}), 500

    # ── MONITORING ROUTES ─────────────────────────────────────────────────────

    @app.route("/api/health", methods=["GET"])
    def health():
        return jsonify({
            "status": "ok",
            "system": "primary",       # frontend api.js cek ini
            "mode": "monitoring_only",
            "timestamp": datetime.now().isoformat(),
        })

    @app.route("/api/status", methods=["GET"])
    @app.route("/api/drone/status", methods=["GET"])
    def status():
        listener = get_listener()
        if listener is None:
            return jsonify({"connected": False, "error": "listener not ready"}), 503
        
        raw_state = listener.state
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
        ground_speed = (vx**2 + vy**2)**0.5
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
        
        return jsonify(transformed)

    @app.route("/api/autonomy/status", methods=["GET"])
    def autonomy_status():
        ctrl = get_autonomy()
        listener = get_listener()
        in_offboard = listener.state.get("in_offboard", False) if listener else False
        if ctrl is None:
            return jsonify({"state": "IDLE", "running": False, "in_offboard_mode": in_offboard})
        return jsonify({
            "state": ctrl.current_state,
            "running": ctrl.is_running,
            "in_offboard_mode": in_offboard,
        })

    # ── COMPAT SHIMS ──────────────────────────────────────────────────────────

    @app.route("/api/drone/offboard/status", methods=["GET", "OPTIONS"])
    def offboard_status_compat():
        ctrl = get_autonomy()
        listener = get_listener()
        in_offboard = listener.state.get("in_offboard", False) if listener else False
        return jsonify({
            "active": ctrl.is_running if ctrl else False,
            "state": ctrl.current_state if ctrl else "IDLE",
            "in_offboard_mode": in_offboard,
            "web_control": False,
            "config": {
                "kp_x": 0.5, "kp_y": 0.5,
                "max_velocity": 0.5, "smoothing_alpha": 0.3,
                "frame_width": 416, "frame_height": 416,
            },
            "detection_count": 0,
            "yolo_connected": ctrl is not None,
            "detection_enabled": ctrl.is_running if ctrl else False,
        })

    @app.route("/api/video/status", methods=["GET"])
    def video_status():
        return jsonify({
            "available": True,
            "backend": "gstreamer",
            "stream": "udp:5600",
            "message": "GStreamer UDP H264 stream on port 5600",
        })

    @app.route("/api/video/stream", methods=["GET"])
    def video_stream():
        def generate():
            while True:
                pipeline = get_pipeline() if get_pipeline else None
                if pipeline and hasattr(pipeline, 'latest_frame') and pipeline.latest_frame is not None:
                    
                    ret, buffer = cv2.imencode('.jpg', pipeline.latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                    if ret:
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                               
                # Batasi Flask di 10 FPS (0.1 detik) agar tidak membunuh CPU Raspi
                time.sleep(0.1)
                
        response = Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
        
        origin = request.environ.get('HTTP_ORIGIN', '*')
        response.headers['Access-Control-Allow-Origin'] = origin
        response.headers['Access-Control-Allow-Credentials'] = 'true'
        
        return response

    # ── DISABLED ENDPOINTS ────────────────────────────────────────────────────

    @app.route("/api/drone/offboard/start", methods=["POST"])
    def offboard_start_disabled():
        return jsonify({"success": False, "error": "Web control disabled. Switch RC to OFFBOARD mode."}), 403

    @app.route("/api/drone/offboard/stop", methods=["POST"])
    def offboard_stop_disabled():
        return jsonify({"success": False, "error": "Web control disabled. Switch RC out of OFFBOARD mode."}), 403

    @app.route("/api/drone/arm", methods=["POST"])
    @app.route("/api/drone/takeoff", methods=["POST"])
    @app.route("/api/drone/land", methods=["POST"])
    def flight_control_disabled():
        return jsonify({"success": False, "error": "Flight control via web is disabled. Use RC transmitter."}), 403

    @app.route("/api/yolo/start", methods=["POST"])
    @app.route("/api/yolo/stop", methods=["POST"])
    def yolo_compat():
        return jsonify({"success": True, "message": "YOLO managed internally by GStreamer pipeline."})

    @app.route("/api/video/start", methods=["POST"])
    @app.route("/api/video/stop", methods=["POST"])
    @app.route("/api/video/photo", methods=["POST"])
    def video_stub():
        return jsonify({"success": False, "error": "Use GStreamer UDP stream on port 5600."}), 501

    @app.route("/api/capabilities", methods=["GET"])
    def capabilities():
        return jsonify({
            "monitoring": True,
            "arm_via_web": False,
            "takeoff_via_web": False,
            "land_via_web": False,
            "start_autonomy_via_web": False,
            "autonomy_trigger": "OFFBOARD flight mode via RC/GCS only",
            "video_stream": "GStreamer UDP H264 on port 5600",
        })

    print("✅ Monitoring-only & Storage routes registered")
