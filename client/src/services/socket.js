import { io } from 'socket.io-client';
import { apiService } from './api';

class SocketService {
  constructor() {
    this.socket = null;
    this.connected = false;
    this.listeners = new Map();
  }

  connect() {
    if (this.socket) {
      return this.socket;
    }

    const socketURL = apiService.getSocketURL();
    console.log('Connecting to socket:', socketURL);

    this.socket = io(socketURL, {
      transports: ['websocket', 'polling'],
      reconnection: true,
      reconnectionDelay: 1000,
      reconnectionAttempts: 10
    });

    this.socket.on('connect', () => {
      console.log('✅ Socket connected');
      this.connected = true;
      this.emit('connection', { connected: true });
    });

    this.socket.on('disconnect', () => {
      console.log('❌ Socket disconnected');
      this.connected = false;
      this.emit('connection', { connected: false });
    });

    this.socket.on('drone:status', (data) => {
      this.emit('drone:status', data);
    });

    this.socket.on('drone:emergency', (data) => {
      this.emit('drone:emergency', data);
    });

    // YOLO Detection Events
    this.socket.on('yolo:detections', (data) => {
      // console.log('🎯 [SocketService] Received yolo:detections:', data);
      this.emit('yolo:detections', data);
    });

    this.socket.on('yolo:status', (data) => {
      // console.log('🎯 [SocketService] Received yolo:status:', data);
      this.emit('yolo:status', data);
    });

    this.socket.on('yolo:tracking', (data) => {
      // console.log('🎯 [SocketService] Received yolo:tracking:', data);
      this.emit('yolo:tracking', data);
    });
    const autonomyEvents = [
      'autonomy:tracking',
      'autonomy:started',
      'autonomy:stopped',
      'autonomy:searching',
      'autonomy:emergency',
      'watchdog:status',
    ];
    autonomyEvents.forEach((evt) => {
      this.socket.on(evt, (data) => {
        this.emit(evt, data);
      });
    });

    return this.socket;
  }

  disconnect() {
    if (this.socket) {
      this.socket.disconnect();
      this.socket = null;
      this.connected = false;
    }
  }

  on(event, callback) {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, []);
    }
    this.listeners.get(event).push(callback);
  }

  off(event, callback) {
    if (!this.listeners.has(event)) return;
    const callbacks = this.listeners.get(event);
    const index = callbacks.indexOf(callback);
    if (index > -1) {
      callbacks.splice(index, 1);
    }
  }

  emit(event, data) {
    if (!this.listeners.has(event)) return;
    const callbacks = this.listeners.get(event);
    callbacks.forEach(callback => callback(data));
  }

  sendCommand(command, params = {}) {
    if (this.socket && this.connected) {
      this.socket.emit('drone:command', { command, params });
    } else {
      console.error('Socket not connected');
    }
  }

  isConnected() {
    return this.connected;
  }
}

export const socketService = new SocketService();
export const socket = socketService; // Alias for compatibility
export default socketService;
