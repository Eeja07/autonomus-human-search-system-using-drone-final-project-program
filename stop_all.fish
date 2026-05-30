#!/usr/bin/env fish

# ============================================================================
# Drone Control System - Stop All Services (Flask)
# ============================================================================

# Warna
set RED '\033[0;31m'
set GREEN '\033[0;32m'
set YELLOW '\033[1;33m'
set NC '\033[0m'

echo -e "$YELLOW"
echo "╔════════════════════════════════════════════════════════╗"
echo "║                                                        ║"
echo "║         🛑 DRONE CONTROL SYSTEM SHUTDOWN 🛑           ║"
echo "║                                                        ║"
echo "╚════════════════════════════════════════════════════════╝"
echo -e "$NC"

# Function to stop screen session
function stop_session
    if screen -list | grep -q $argv[1]
        screen -S $argv[1] -X quit
        echo -e "$GREEN✓$NC Stopped: $argv[1]"
    else
        echo -e "$YELLOW⚠$NC Not running: $argv[1]"
    end
end

echo ""
echo -e "$YELLOW Stopping all services...$NC"
echo ""

# Stop all sessions
stop_session "frontend"
stop_session "backend_flask"

echo ""
echo -e "$GREEN✓ All services stopped$NC"
echo ""
