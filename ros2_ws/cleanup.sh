#!/bin/bash
// filepath: cleanup.sh

echo "🧹 Cleaning up unnecessary files..."

# Test directories
rm -rf ros2_ws/src/courier_control/test/
rm -rf ros2_ws/src/courier_nav/test/

# Old/backup files
rm -f ros2_ws/src/courier_nav/courier_nav/mission_controller.py

# Documentation duplicates
rm -f ros2_ws/ARCHITETTURA.md

# Test scripts
rm -f ros2_ws/test_bfs.py
rm -f ros2_ws/start_simulation.sh

# Temporary files
rm -f ros2_ws/=2.2.0
rm -f ros2_ws/frames_2025-12-19_18.11.15.gv

# Old diagrams
rm -f "behavioural tree.mmd"

# Build artifacts (optional - they'll be regenerated)
# rm -rf ros2_ws/build/
# rm -rf ros2_ws/install/
# rm -rf ros2_ws/log/

echo "✅ Cleanup complete!"
echo ""
echo "Removed:"
echo "  • Test files"
echo "  • Old mission controller"
echo "  • Duplicate documentation"
echo "  • Test/debug scripts"
echo "  • Temporary files"