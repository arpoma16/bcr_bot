#!/bin/bash

# Script de diagnóstico para SLAM Toolbox
# Ejecutar mientras el sistema está corriendo

echo "========================================="
echo "SLAM Toolbox Diagnostics"
echo "========================================="
echo ""

echo "=== 1. Verificando frame_id en mensajes de scan ==="
echo "Leyendo el frame_id del topic /bcr_bot/scan..."
FRAME_ID=$(timeout 3s ros2 topic echo /bcr_bot/scan --once 2>/dev/null | grep "frame_id:" | head -1 | awk '{print $2}' | tr -d "'\"")
echo "Frame ID detectado: '$FRAME_ID'"
echo ""

echo "=== 2. Verificando árbol TF ==="
echo "Listando todos los frames TF disponibles:"
timeout 5s ros2 run tf2_ros tf2_monitor 2>&1 | head -20
echo ""

echo "=== 3. Intentando transformación odom -> base_link ==="
timeout 5s ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -10
echo ""

echo "=== 4. Intentando transformación base_link -> $FRAME_ID ==="
timeout 5s ros2 run tf2_ros tf2_echo base_link "$FRAME_ID" 2>&1 | head -10
echo ""

echo "=== 5. Verificando parámetro use_sim_time en nodos ==="
echo "Checking slam_toolbox use_sim_time:"
ros2 param get /slam_toolbox use_sim_time 2>&1
echo ""
echo "Checking rviz2 use_sim_time:"
ros2 param get /rviz2 use_sim_time 2>&1
echo ""

echo "=== 6. Verificando topic /clock ==="
echo "Checking if /clock is being published:"
timeout 3s ros2 topic hz /clock 2>&1 | grep -E "average|WARNING"
echo ""

echo "=== 7. Verificando frecuencia de scan ==="
echo "Checking /bcr_bot/scan frequency:"
timeout 5s ros2 topic hz /bcr_bot/scan 2>&1 | grep -E "average|WARNING"
echo ""

echo "=== 8. Mostrando un mensaje de scan completo ==="
timeout 3s ros2 topic echo /bcr_bot/scan --once 2>&1 | head -30
echo ""

echo "========================================="
echo "Diagnóstico completo"
echo "========================================="
