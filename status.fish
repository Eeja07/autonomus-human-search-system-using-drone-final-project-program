#!/usr/bin/env fish

# ============================================================================
# Drone Control System - Status Check (Flask)
# ============================================================================

# Warna
set RED '\033[0;31m'
set GREEN '\033[0;32m'
set YELLOW '\033[1;33m'
set BLUE '\033[0;34m'
set CYAN '\033[0;36m'
set NC '\033[0m'

echo -e "$CYAN"
echo "╔════════════════════════════════════════════════════════╗"
echo "║                                                        ║"
echo "║         📊 DRONE CONTROL SYSTEM STATUS 📊             ║"
echo "║                                                        ║"
echo "╚════════════════════════════════════════════════════════╝"
echo -e "$NC"

# Function to check screen session
function session_exists
    screen -list | grep -q $argv[1]
end

# Function to check port
function port_listening
    lsof -i :$argv[1] > /dev/null 2>&1
end

echo ""
echo -e "$YELLOW Screen Sessions:$NC"

if session_exists "backend_flask"
    echo -e "  $GREEN✓$NC backend_flask    (Flask Backend)"
else
    echo -e "  $RED✗$NC backend_flask    (NOT RUNNING)"
end

# YOLO is now integrated in Flask backend, no separate service needed

if session_exists "frontend"
    echo -e "  $GREEN✓$NC frontend         (React Frontend)"
else
    echo -e "  $RED✗$NC frontend         (NOT RUNNING)"
end

echo ""
echo -e "$YELLOW Listening Ports:$NC"

if port_listening 3000
    echo -e "  $GREEN✓$NC Port 3000        (Flask Backend)"
else
    echo -e "  $RED✗$NC Port 3000        (NOT LISTENING)"
end

if port_listening 5173
    echo -e "  $GREEN✓$NC Port 5173        (Frontend)"
else
    echo -e "  $RED✗$NC Port 5173        (NOT LISTENING)"
end

echo ""
echo -e "$YELLOW Access Points:$NC"
echo "  • Frontend   : http://localhost:5173"
echo "  • Backend API: http://localhost:3000"
echo "  • Video Stream: http://localhost:3000/api/video/stream"
echo "  • WebSocket  : ws://localhost:3000/socket.io/"

echo ""
echo -e "$BLUE Commands:$NC"
echo "  • View logs : screen -r <service_name>"
echo "  • Detach    : Ctrl+A then D"
echo "  • Stop all  : ./stop_all.fish"
echo ""
