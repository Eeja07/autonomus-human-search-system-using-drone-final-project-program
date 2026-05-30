/**
 * API Service - Local Mode Only
 * ===================================================
 * * Mode Lokal (Primary):
 * - Direct connection ke Raspberry Pi pada host yang sama dengan dashboard (:3000)
 * - Full flight control commands
 * - Low latency
 * - Mission upload & execution
 */

const API_CONFIG = {
  // Use the same host that serves the dashboard, but switch to Flask port 3000.
  // This works for localhost, Raspberry Pi AP IP, LAN IP, and Tailscale IP.
  baseURL: (() => {
    const hostname = window.location.hostname;
    return `http://${hostname}:3000`;
  })(),
  socketURL: (() => {
    const hostname = window.location.hostname;
    return `http://${hostname}:3000`;
  })()
};

class ApiService {
  constructor() {
    this.connected = false;
    this.checkConnection();
  }

  async checkConnection() {
    console.log('🔍 Checking local connection...');
    
    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 2000);
      
      const response = await fetch(`${API_CONFIG.baseURL}/api/health`, {
        method: 'GET',
        signal: controller.signal
      });
      
      clearTimeout(timeoutId);
      
      if (response.ok) {
        const data = await response.json();
        if (data.system === 'primary') {
          this.connected = true;
          console.log('✅ Connected to LOCAL Primary Control System');
          console.log('🎮 Full flight control available');
          return true;
        }
      }
    } catch (error) {
      console.error('❌ Local connection failed:', error);
    }
    
    this.connected = false;
    return false;
  }

  getBaseURL() {
    return API_CONFIG.baseURL;
  }

  getSocketURL() {
    return API_CONFIG.socketURL;
  }

  async get(endpoint) {
    const url = `${this.getBaseURL()}${endpoint}`;
    const response = await fetch(url, {
      headers: { 'Content-Type': 'application/json' }
    });
    
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    
    return await response.json();
  }

  async post(endpoint, data = {}) {
    const url = `${this.getBaseURL()}${endpoint}`;
    
    try {
      const response = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data),
      });
      
      if (!response.ok) {
        // Try to get error details from response
        let errorMessage = `HTTP ${response.status}: ${response.statusText}`;
        try {
          const errorData = await response.json();
          errorMessage = errorData.error || errorData.message || errorMessage;
        } catch (e) {
          // If JSON parsing fails, use default error message
        }
        
        console.error(`❌ API Error [${endpoint}]:`, errorMessage);
        throw new Error(errorMessage);
      }
      
      return await response.json();
    } catch (error) {
      // Network error or other fetch error
      if (error.message.includes('fetch')) {
        console.error('❌ Network error - is server running?');
        throw new Error('Cannot connect to server. Please check connection.');
      }
      throw error;
    }
  }

  // ==================== DRONE CONTROL API ====================

  async getStatus() {
    return await this.get('/api/drone/status');
  }
  // ==================== STORAGE API ====================

  async getStorageFiles() {
    try {
      return await this.get('/api/storage/list');
    } catch (error) {
      console.error('❌ Failed to fetch storage files:', error);
      return []; 
    }
  }

  getPhotoUrl(filename) {
    return `${this.getBaseURL()}/api/storage/photos/${filename}`;
  }
}

// Create singleton instance
const apiService = new ApiService();

export { apiService };
export const api = apiService; // Alias for compatibility
export default apiService;
