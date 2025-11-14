#!/bin/bash

# Script para lanzar mapping con bcr_bot evitando problemas de sincronización
# Este script asegura que todos los componentes se lancen en el orden correcto

set -e

echo "======================================"
echo "Starting BCR Bot Mapping System"
echo "======================================"

# Colores para output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Directorio del workspace
WS_DIR="/ariac_ws"

# Source ROS 2
echo -e "${YELLOW}[1/5] Sourcing ROS 2 environment...${NC}"
source /opt/ros/jazzy/setup.bash
cd ${WS_DIR}
source install/setup.bash

# Verificar que Gazebo no esté corriendo
echo -e "${YELLOW}[2/5] Checking for existing Gazebo instances...${NC}"
if pgrep -x "gz" > /dev/null || pgrep -x "gzserver" > /dev/null
then
    echo "WARNING: Gazebo is already running. Please kill all Gazebo processes first:"
    echo "  killall -9 gz gzserver gzclient"
    exit 1
fi

# Lanzar Gazebo
echo -e "${YELLOW}[3/5] Launching Gazebo simulation...${NC}"
gz sim ${WS_DIR}/src/indra/worlds/factory.world --verbose &
GZ_PID=$!
echo "Gazebo PID: $GZ_PID"

# Esperar a que Gazebo esté listo
echo "Waiting for Gazebo to initialize (15 seconds)..."
sleep 15

# Verificar que Gazebo sigue corriendo
if ! kill -0 $GZ_PID 2>/dev/null; then
    echo "ERROR: Gazebo failed to start"
    exit 1
fi

echo -e "${GREEN}Gazebo is ready!${NC}"

# Lanzar indra (spawns bcr_bot y bridges)
echo -e "${YELLOW}[4/5] Launching robot and bridges...${NC}"
ros2 launch indra indra.launch.py &
INDRA_PID=$!
echo "Robot launch PID: $INDRA_PID"

# Esperar a que el robot se spawne
echo "Waiting for robot to spawn (10 seconds)..."
sleep 10

# Verificar que los topics estén disponibles
echo "Checking for /bcr_bot/scan topic..."
timeout 10s bash -c 'until ros2 topic list | grep -q "/bcr_bot/scan"; do sleep 1; done' || {
    echo "ERROR: /bcr_bot/scan topic not found"
    kill $GZ_PID $INDRA_PID 2>/dev/null
    exit 1
}

echo -e "${GREEN}Robot spawned successfully!${NC}"

# Lanzar mapping
echo -e "${YELLOW}[5/5] Launching SLAM Toolbox for mapping...${NC}"
ros2 launch bcr_bot mapping.launch.py &
MAPPING_PID=$!
echo "Mapping PID: $MAPPING_PID"

sleep 3

echo ""
echo -e "${GREEN}======================================"
echo "All systems launched successfully!"
echo "======================================${NC}"
echo ""
echo "PIDs:"
echo "  Gazebo: $GZ_PID"
echo "  Robot:  $INDRA_PID"
echo "  Mapping: $MAPPING_PID"
echo ""
echo "To control the robot, open a new terminal and run:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/bcr_bot/cmd_vel"
echo ""
echo "To stop all processes, press Ctrl+C or run:"
echo "  kill $GZ_PID $INDRA_PID $MAPPING_PID"
echo ""

# Esperar a que el usuario detenga el script
wait
