#!/usr/bin/env fish

# ============================================================================
# Drone Control System - Start All Services (Flask Backend)
# ============================================================================
# Script untuk menjalankan semua komponen sistem dengan Flask backend
# Author: TA Mahija
# ============================================================================

# Warna untuk output
set RED '\033[0;31m'
set GREEN '\033[0;32m'
set YELLOW '\033[1;33m'
set BLUE '\033[0;34m'
set CYAN '\033[0;36m'
set NC '\033[0m' # No Color

# Configuration
set PROJECT_DIR "/home/pi/terbangsemhas"
set VENV_DIR "/home/pi/venv"

# Function to check if screen session exists
function session_exists
    screen -list | grep -q $argv[1]
end

# Function to print status
function print_status
    if test $argv[1] = "OK"
        echo -e "$GREEN✓ $argv[2]$NC"
    else if test $argv[1] = "FAIL"
        echo -e "$RED✗ $argv[2]$NC"
    else if test $argv[1] = "INFO"
        echo -e "$BLUE→ $argv[2]$NC"
    else if test $argv[1] = "WARN"
        echo -e "$YELLOW⚠ $argv[2]$NC"
    end
end

# Banner
echo -e "$CYAN"
echo "╔════════════════════════════════════════════════════════╗"
echo "║                                                        ║"
echo "║         🚁 DRONE CONTROL SYSTEM STARTUP 🚁            ║"
echo "║                                                        ║"
echo "║              Raspberry Pi 5 - Flask Backend           ║"
echo "║                                                        ║"
echo "╚════════════════════════════════════════════════════════╝"
echo -e "$NC"

# ============================================================================
# 1. STOP EXISTING SERVICES
# ============================================================================

echo ""
print_status "INFO" "Stopping any existing services..."

if session_exists "frontend"
    screen -S frontend -X quit
    print_status "OK" "Stopped existing frontend"
end

if session_exists "backend_flask"
    screen -S backend_flask -X quit
    print_status "OK" "Stopped existing Flask backend"
end


sleep 2

# ============================================================================
# 2. START FLASK BACKEND
# ============================================================================

echo ""
print_status "INFO" "Starting Flask Backend Server..."

cd $PROJECT_DIR/server_flask

# Activate virtual environment and start Flask
screen -dmS backend_flask bash -c "source $VENV_DIR/bin/activate; python3 app.py"

sleep 3

if session_exists "backend_flask"
    print_status "OK" "Flask backend started in screen session 'backend_flask'"
else
    print_status "FAIL" "Failed to start Flask backend"
end


# ============================================================================
# 3. START FRONTEND
# ============================================================================

echo ""
print_status "INFO" "Starting Frontend Development Server..."

cd $PROJECT_DIR/client

# Check if node_modules exists
if not test -d node_modules
    print_status "WARN" "node_modules not found, running npm install..."
    npm install
end

screen -dmS frontend bash -c "npm run dev"

sleep 3

if session_exists "frontend"
    print_status "OK" "Frontend started in screen session 'frontend'"
else
    print_status "FAIL" "Failed to start frontend"
end

# ============================================================================
# SUMMARY
# ============================================================================

echo ""
echo -e "$CYAN"
echo "╔════════════════════════════════════════════════════════╗"
echo "║                                                        ║"
echo "║                 🎉 SYSTEM STARTED! 🎉                 ║"
echo "║                                                        ║"
echo "╚════════════════════════════════════════════════════════╝"
echo -e "$NC"

echo ""
echo -e "$GREEN"
echo "📡 Services Status:"
echo -e "$NC"

# Check each service
if session_exists "backend_flask"
    echo -e "  $GREEN✓$NC Flask Backend       : http://localhost:3000 (includes YOLO)"
else
    echo -e "  $RED✗$NC Flask Backend       : NOT RUNNING"
end

if session_exists "frontend"
    echo -e "  $GREEN✓$NC Frontend           : http://localhost:5173"
else
    echo -e "  $RED✗$NC Frontend           : NOT RUNNING"
end

echo ""
echo -e "$YELLOW"
echo "📋 Management Commands:"
echo -e "$NC"
echo "  • View logs          : screen -r <service_name>"
echo "  • Detach from screen : Ctrl+A then D"
echo "  • Check status       : ./status.fish"
echo "  • Stop all services  : ./stop_all.fish"
echo ""
echo -e "$BLUE"
echo "🔧 Service Names:"
echo -e "$NC"
echo "  • backend_flask    - Flask Backend Server (includes YOLO)"
echo "  • frontend         - React Frontend (Vite)"
echo ""
echo -e "$GREEN"
echo "🌐 Access Points:"
echo -e "$NC"
echo "  • Frontend Dashboard : http://localhost:5173"
echo "  • Backend API        : http://localhost:3000"
echo "  • Video Stream       : http://localhost:3000/api/video/stream"
echo "  • SocketIO           : ws://localhost:3000/socket.io/"
echo ""

echo -e "$CYAN"
echo "Happy Flying! 🚁"
echo -e "$NC"
