function DroneStatus({ status, connected, autonomyStatus }) {
  if (!status) {
    return (
      <div className="card">
        <h2>Drone Status</h2>
        <p>{connected ? 'Waiting for data...' : 'Not connected, try reconnecting again'}</p>
      </div>
    );
  }

  const getBatteryColor = (battery) => {
    if (battery > 50) return '#4caf50';
    if (battery > 20) return '#ff9800';
    return '#f44336';
  };

  const autonomyState = autonomyStatus?.state || 'IDLE';
  const autonomyRunning = Boolean(autonomyStatus?.running);

  return (
    <div className="card">
      <h2>Drone Status</h2>
      
      <div style={{
        display: 'grid',
        gridTemplateColumns: 'repeat(auto-fit, minmax(120px, 1fr))',
        gap: '0.5rem',
        marginBottom: '0.75rem',
      }}>
        <div style={{ 
          padding: '0.625rem',
          background: status.connected ? 'rgba(153, 153, 153, 0.2)' : 'rgba(244, 67, 54, 0.2)',
          borderRadius: '6px',
          textAlign: 'center',
          border: `2px solid ${status.connected ? '#666666' : '#999999'}`
        }}>
          <div style={{ fontSize: '0.75rem', opacity: 0.8 }}>Connection</div>
          <div style={{ fontSize: 'clamp(0.9rem, 2vw, 1.1rem)', fontWeight: 'bold' }}>
            {status.connected ? '🟢 Online' : '🔴 Offline'}
          </div>
        </div>
        
        <div style={{ 
          padding: '0.625rem',
          background: status.armed ? 'rgba(100, 100, 100, 0.2)' : 'rgba(150, 150, 150, 0.2)',
          borderRadius: '6px',
          textAlign: 'center',
          border: `2px solid ${status.armed ? '#666666' : '#999999'}`
        }}>
          <div style={{ fontSize: '0.75rem', opacity: 0.8 }}>Armed</div>
          <div style={{ fontSize: 'clamp(0.9rem, 2vw, 1.1rem)', fontWeight: 'bold' }}>
            {status.armed ? '● ARMED' : '○ Safe'}
          </div>
        </div>

        <div style={{
          padding: '0.625rem',
          background: 'rgba(153, 153, 153, 0.2)',
          borderRadius: '6px',
          textAlign: 'center',
          border: '2px solid #999999'
        }}>
          <div style={{ fontSize: '0.75rem', opacity: 0.8 }}>Flight Mode</div>
          <div style={{ fontSize: 'clamp(0.9rem, 2vw, 1.1rem)', fontWeight: 'bold', textTransform: 'uppercase' }}>
            {status.mode || 'UNKNOWN'}
          </div>
        </div>

        <div style={{
          padding: '0.625rem',
          background: autonomyRunning ? 'rgba(120, 120, 120, 0.24)' : 'rgba(80, 80, 80, 0.18)',
          borderRadius: '6px',
          textAlign: 'center',
          border: `2px solid ${autonomyRunning ? '#aaaaaa' : '#666666'}`
        }}>
          <div style={{ fontSize: '0.75rem', opacity: 0.8 }}>Autonomy State</div>
          <div style={{ fontSize: 'clamp(0.9rem, 2vw, 1.1rem)', fontWeight: 'bold', textTransform: 'uppercase' }}>
            {autonomyState}
          </div>
        </div>
      </div>

      {/* Battery */}
      <div style={{ marginBottom: '0.75rem' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '0.4rem' }}>
          <span style={{ fontSize: 'clamp(0.8rem, 1.5vw, 0.9rem)' }}>Battery</span>
          <span style={{ fontSize: 'clamp(0.95rem, 2vw, 1.1rem)', fontWeight: 'bold', color: getBatteryColor(status.battery || 0) }}>
            {(status.battery || 0).toFixed(0)}%
          </span>
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
          <div style={{
            flex: 1,
            height: '18px',
            border: '2px solid #aaaaaa',
            borderRadius: '4px',
            background: 'rgba(255,255,255,0.05)',
            overflow: 'hidden',
          }}>
            <div style={{
              width: `${Math.min(100, Math.max(0, status.battery || 0))}%`,
              height: '100%',
              background: getBatteryColor(status.battery || 0),
              transition: 'width 0.5s ease, background 0.5s ease',
              boxShadow: `0 0 6px ${getBatteryColor(status.battery || 0)}88`,
            }} />
          </div>
          {/* Terminal nub */}
          <div style={{
            width: '5px',
            height: '10px',
            background: '#aaaaaa',
            borderRadius: '0 2px 2px 0',
            flexShrink: 0,
          }} />
        </div>
      </div>

      {/* Additional Info Grid */}
      <div className="status-grid">
        <div className="status-item">
          <span className="status-label">Ground Speed</span>
          <span className="status-value">{(status.velocity?.ground_speed ?? 0).toFixed(1)} m/s</span>
        </div>
        <div className="status-item">
          <span className="status-label">Heading</span>
          <span className="status-value">{(status.heading ?? 0).toFixed(0)}°</span>
        </div>
        <div className="status-item">
          <span className="status-label">Voltage</span>
          <span className="status-value">{(status.voltage ?? 0).toFixed(2)} V</span>
        </div>
        <div className="status-item">
          <span className="status-label">Current</span>
          <span className="status-value">{(status.current ?? 0).toFixed(2)} A</span>
        </div>
        <div className="status-item">
          <span className="status-label">GPS Sats</span>
          <span className="status-value">{(status.gps?.satellites ?? 0).toFixed(0)}</span>
        </div>
        <div className="status-item">
          <span className="status-label">Latitude</span>
          <span className="status-value">{(status.gps?.lat ?? 0).toFixed(6)}</span>
        </div>
        <div className="status-item">
          <span className="status-label">Longitude</span>
          <span className="status-value">{(status.gps?.lon ?? 0).toFixed(6)}</span>
        </div>
        <div className="status-item">
          <span className="status-label">Altitude</span>
          <span className="status-value">{(status.gps?.alt ?? 0).toFixed(1)} m</span>
        </div>
      </div>
    </div>
  );
}

export default DroneStatus;
