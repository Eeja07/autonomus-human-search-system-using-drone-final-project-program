import { useState, useEffect, useRef } from 'react';
import { apiService } from '../services/api';

/**
 * VideoControl
 * ============
 * Menampilkan H264 UDP stream dari GStreamer via HLS/MJPEG proxy.
 *
 * Backend baru tidak punya /api/video/status, /api/video/stream, dll.
 * Video sekarang di-stream via GStreamer UDP:5600.
 *
 * Untuk view di browser: backend perlu proxy UDP→MJPEG atau pakai
 * WebRTC. Sementara ini, tampilkan instruksi stream + deteksi data
 * dari socket yolo:detections.
 */
function VideoControl({ onNotification }) {
  const [detections, setDetections] = useState([]);
  const [fps, setFps] = useState(0);
  const [detectionEnabled, setDetectionEnabled] = useState(false);
  const [serverReady, setServerReady] = useState(false);

  // Check server health saat mount
  useEffect(() => {
    const checkServer = async () => {
      try {
        await fetch(`${apiService.getBaseURL()}/api/health`);
        setServerReady(true);
      } catch {
        setServerReady(false);
      }
    };
    checkServer();
    const interval = setInterval(checkServer, 5000);
    return () => clearInterval(interval);
  }, []);

  // Listen socket yolo:detections
  useEffect(() => {
    const setupListener = () => {
      if (!window.socket) {
        setTimeout(setupListener, 100);
        return;
      }

      const handleDetection = (data) => {
        const persons = (data.detections || []).filter(
          d => (d.class_name || d.class || '').toLowerCase() === 'person'
        );
        setDetections(persons);
        setDetectionEnabled(true);
        const raw = data?.fps_inst ?? data?.fps ?? 0;
        setFps(Number.isFinite(Number(raw)) ? Number(raw) : 0);
      };

      window.socket.on('yolo:detections', handleDetection);

      const listeners = window.socket.listeners('yolo:detections');
      console.log('📡 Registered listeners for yolo:detections:', listeners.length);

      return () => {
        if (window.socket) window.socket.off('yolo:detections', handleDetection);
      };
    };

    const cleanup = setupListener();
    return cleanup;
  }, []);

  const udpPort = 5600;
  const streamHost = apiService.getBaseURL().replace(/:\d+$/, '');

  return (
    <div className="card" style={{ gridColumn: 'span 1' }}>
      <h2>Camera</h2>

      {/* Detection Status */}
      {detectionEnabled && (
        <div style={{
          marginBottom: '1rem',
          padding: '0.5rem',
          borderRadius: '6px',
          border: '1px solid #999',
          fontSize: '0.9rem',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
        }}>
          <span>YOLO Detection</span>
          <div style={{ display: 'flex', gap: '1rem' }}>
            <span>{fps.toFixed(1)} FPS</span>
            <span>{detections.length} Persons</span>
          </div>
        </div>
      )}

      {/* Stream Info Panel -> Diubah jadi Video Player MJPEG */}
      <div style={{
        width: '100%',
        background: '#0a0a0f',
        borderRadius: '8px',
        overflow: 'hidden',
        marginBottom: '1rem',
        position: 'relative',
        border: '2px solid rgba(255,255,255,0.15)',
      }}>

        {serverReady ? (
          <img
            src={`${apiService.getBaseURL()}/api/video/stream`}
            alt="Drone Live Stream"
            crossOrigin="use-credentials" // <--- TAMBAHKAN BARIS INI
            // style={{ width: '100%', height: '100%', objectFit: 'fill' }} // <--- UBAH DI SINI
            style={{ width: '100%', height: 'auto', display: 'block' }} // <--- UBAH DI SINI
          />
        ) : (
          <div style={{ display: 'flex', height: '100%', alignItems: 'center', justifyContent: 'center', color: 'rgba(255,255,255,0.5)' }}>
            Menunggu Server Video...
          </div>
        )}

        {/* Server status dot (Tetap dipertahankan) */}
        <div style={{
          position: 'absolute',
          top: '10px',
          right: '12px',
          display: 'flex',
          alignItems: 'center',
          gap: '0.4rem',
          fontSize: '0.75rem',
          color: serverReady ? '#4caf50' : '#f44336',
          background: 'rgba(0,0,0,0.6)',
          padding: '4px 8px',
          borderRadius: '12px'
        }}>
          <span style={{
            width: 8, height: 8, borderRadius: '50%',
            background: serverReady ? '#4caf50' : '#f44336',
            display: 'inline-block',
          }} />
          {serverReady ? 'Live' : 'Offline'}
        </div>
      </div>

      {/* Detection Results */}
      {detectionEnabled && detections.length > 0 && (
        <div style={{
          marginTop: '0.5rem',
          padding: '0.75rem',
          borderRadius: '6px',
          background: 'rgba(158,158,158,0.1)',
          border: '1px solid rgba(158,158,158,0.3)',
        }}>
          <div style={{
            fontSize: '0.9rem',
            fontWeight: 'bold',
            marginBottom: '0.5rem',
            color: '#999',
          }}>
            {detections.length} Person{detections.length > 1 ? 's' : ''} Detected
          </div>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: '0.5rem', fontSize: '0.85rem' }}>
            {detections.map((det, idx) => (
              <div key={idx} style={{
                padding: '0.25rem 0.75rem',
                background: 'rgba(158,158,158,0.1)',
                border: '1px solid rgba(158,158,158,0.3)',
                borderRadius: '12px',
                display: 'flex',
                alignItems: 'center',
                gap: '0.5rem',
              }}>
                <span>Person #{idx + 1}</span>
                <span style={{ opacity: 0.7 }}>
                  {((det.confidence ?? 0) * 100).toFixed(0)}%
                </span>
              </div>
            ))}
          </div>
        </div>
      )}

      {detectionEnabled && detections.length === 0 && (
        <div style={{
          marginTop: '0.5rem',
          padding: '0.75rem',
          borderRadius: '6px',
          background: 'rgba(158,158,158,0.1)',
          border: '1px solid rgba(158,158,158,0.3)',
          textAlign: 'center',
          color: 'rgba(255,255,255,0.5)',
          fontSize: '0.85rem',
          minHeight: '60px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
        }}>
          Scanning for persons...
        </div>
      )}

      {!detectionEnabled && (
        <div style={{
          marginTop: '0.5rem',
          padding: '0.75rem',
          borderRadius: '6px',
          background: 'rgba(158,158,158,0.05)',
          border: '1px solid rgba(158,158,158,0.15)',
          textAlign: 'center',
          color: 'rgba(255,255,255,0.3)',
          fontSize: '0.82rem',
        }}>
          Menunggu data deteksi dari YOLO...
        </div>
      )}
    </div>
  );
}

export default VideoControl;  