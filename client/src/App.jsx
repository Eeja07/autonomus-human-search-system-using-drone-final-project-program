import { useState, useEffect, useRef } from 'react';
import './App.css';
import { apiService } from './services/api';
import { socketService } from './services/socket';
import DroneStatus from './components/DroneStatus';
import VideoControl from './components/VideoControl';
import StoragePanel from './components/StoragePanel';

function App() {
  const [connected, setConnected] = useState(false);
  const [droneStatus, setDroneStatus] = useState(null);
  const [autonomyStatus, setAutonomyStatus] = useState({
    state: 'IDLE',
    running: false,
  });
  const [notifications, setNotifications] = useState([]);
  const [storageOpen, setStorageOpen] = useState(false);
  const [storageRefresh, setStorageRefresh] = useState(0);
  const notificationIdRef = useRef(0);

  useEffect(() => {
    // Connect socket immediately (synchronous) — avoids duplicate sockets in StrictMode
    const socket = socketService.connect();
    window.socket = socket;
    console.log('✅ window.socket set:', window.socket);

    // Cek koneksi lokal secara async (menggantikan detectMode)
    apiService.checkConnection().then(isConnected => {
      if (isConnected) {
        console.log('API Service ready for local connection');
      }
    });

    const fetchAutonomyStatus = async () => {
      try {
        const response = await apiService.get('/api/autonomy/status');
        setAutonomyStatus({
          state: response.state ?? 'IDLE',
          running: response.running ?? false,
        });
      } catch {
        // Backend may still be starting; keep the last known autonomy state.
      }
    };

    const autonomyPoll = setInterval(fetchAutonomyStatus, 2000);
    fetchAutonomyStatus();

    // Define named callbacks so they can be removed on cleanup
    const handleConnection = (data) => {
      if (data.connected) {
        addNotification('Connected to drone server', 'success');
      } else {
        addNotification('Disconnected from server', 'error');
        setConnected(false);
      }
    };

    const handleDroneStatus = (status) => {
      setDroneStatus(status);
      if (status && status.connected !== undefined) {
        setConnected(status.connected);
      }
    };

    const handleDroneEmergency = (data) => {
      addNotification(data.message, 'warning');
    };

    const handleAutonomyChanged = () => {
      fetchAutonomyStatus();
    };

    // Handler real-time untuk perubahan state scout — update langsung tanpa menunggu polling 2 detik
    const handleScoutStateChanged = (data) => {
      setAutonomyStatus({
        state: `SCOUT_${data.state}`,
        running: true,
      });
    };

    socketService.on('connection', handleConnection);
    socketService.on('drone:status', handleDroneStatus);
    socketService.on('drone:emergency', handleDroneEmergency);
    socketService.on('autonomy:started', handleAutonomyChanged);
    socketService.on('autonomy:tracking', handleAutonomyChanged);
    socketService.on('autonomy:stopped', handleAutonomyChanged);
    socketService.on('autonomy:searching', handleAutonomyChanged);
    socketService.on('autonomy:emergency', handleAutonomyChanged);
    socketService.on('offboard:started', handleAutonomyChanged);
    socketService.on('offboard:auto_rtl', handleAutonomyChanged);
    socketService.on('scout:state_changed', handleScoutStateChanged);
    socketService.on('scout:started', handleAutonomyChanged);
    socketService.on('scout:documented', handleAutonomyChanged);

    return () => {
      clearInterval(autonomyPoll);
      socketService.off('connection', handleConnection);
      socketService.off('drone:status', handleDroneStatus);
      socketService.off('drone:emergency', handleDroneEmergency);
      socketService.off('autonomy:started', handleAutonomyChanged);
      socketService.off('autonomy:tracking', handleAutonomyChanged);
      socketService.off('autonomy:stopped', handleAutonomyChanged);
      socketService.off('autonomy:searching', handleAutonomyChanged);
      socketService.off('autonomy:emergency', handleAutonomyChanged);
      socketService.off('offboard:started', handleAutonomyChanged);
      socketService.off('offboard:auto_rtl', handleAutonomyChanged);
      socketService.off('scout:state_changed', handleScoutStateChanged);
      socketService.off('scout:started', handleAutonomyChanged);
      socketService.off('scout:documented', handleAutonomyChanged);
      socketService.disconnect();
    };
  }, []);

  const addNotification = (message, type = 'info') => {
    // Use incrementing counter instead of Date.now() to ensure unique IDs
    notificationIdRef.current += 1;
    const id = notificationIdRef.current;
    setNotifications(prev => [...prev, { id, message, type }]);
    setTimeout(() => {
      setNotifications(prev => prev.filter(n => n.id !== id));
    }, 5000);
  };

  return (
    <div className="app">
      <header className="app-header">
        <div className="header-left">
          <h1>Holybro S500 Drone Control</h1>
        </div>

        <div className="header-right">
          <button
            className={`storage-toggle-btn ${storageOpen ? 'active' : ''}`}
            onClick={() => setStorageOpen(o => !o)}
            title="Storage – photos & recordings"
          >
            💾 Storage
          </button>

          <div className="connection-status">
            <div className={`status-indicator ${connected ? 'connected' : 'disconnected'}`}>
              <span className="status-text">{connected ? '🟢 Connected' : '🔴 Disconnected'}</span>
            </div>
          </div>

        </div>
      </header>

      <div className="notifications">
        {notifications.map(notif => (
          <div key={notif.id} className={`notification notification-${notif.type}`}>
            {notif.message}
          </div>
        ))}
      </div>

      <div className="main-content">
        {/* Unified Container - All in One Box */}
        <div className="unified-container">
          {/* Top Section: Status & Pre-flight */}
          <div className="section-row">
            <DroneStatus status={droneStatus} connected={connected} autonomyStatus={autonomyStatus} />
            <VideoControl
              onNotification={addNotification}
            />

          </div>
        </div>
      </div>
      <StoragePanel
        isOpen={storageOpen}
        onClose={() => setStorageOpen(false)}
        onNotification={addNotification}
        refreshTrigger={storageRefresh}
      />
    </div>
  );
}

export default App;
