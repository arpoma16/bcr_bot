#!/bin/bash

# Script para verificar la cadena completa de TF necesaria para SLAM

echo "========================================="
echo "Verificación completa de TF para SLAM"
echo "========================================="
echo ""

# Obtener el frame_id del scan
echo "1. Obteniendo frame_id del sensor..."
SCAN_FRAME=$(timeout 3s ros2 topic echo /bcr_bot/scan header/frame_id --once 2>/dev/null | head -1 | tr -d "'" | tr -d '"' | xargs)

if [ -z "$SCAN_FRAME" ]; then
    echo "ERROR: No se pudo leer el frame_id del scan"
    exit 1
fi

echo "   Frame del sensor: '$SCAN_FRAME'"
echo ""

# Verificar cada transformación en la cadena
echo "2. Verificando cadena de transformaciones..."
echo ""

echo "   a) Transformación: odom -> base_link"
if timeout 3s ros2 run tf2_ros tf2_echo odom base_link > /tmp/tf_odom_base 2>&1; then
    echo "      ✓ OK"
    echo "      Datos:"
    head -10 /tmp/tf_odom_base | grep -E "Translation|Rotation" | sed 's/^/        /'
else
    echo "      ✗ FALLO - Esta transformación debería venir del odometry publisher de Gazebo"
    cat /tmp/tf_odom_base | head -5 | sed 's/^/        /'
fi
echo ""

echo "   b) Transformación: base_link -> $SCAN_FRAME"
if timeout 3s ros2 run tf2_ros tf2_echo base_link "$SCAN_FRAME" > /tmp/tf_base_scan 2>&1; then
    echo "      ✓ OK"
    echo "      Datos:"
    head -10 /tmp/tf_base_scan | grep -E "Translation|Rotation" | sed 's/^/        /'
else
    echo "      ✗ FALLO - Esta transformación debería venir del robot_state_publisher"
    cat /tmp/tf_base_scan | head -5 | sed 's/^/        /'
fi
echo ""

echo "   c) Transformación completa: odom -> $SCAN_FRAME"
if timeout 3s ros2 run tf2_ros tf2_echo odom "$SCAN_FRAME" > /tmp/tf_odom_scan 2>&1; then
    echo "      ✓ OK - La cadena completa funciona"
else
    echo "      ✗ FALLO - No se puede completar la cadena de transformaciones"
    cat /tmp/tf_odom_scan | head -5 | sed 's/^/        /'
fi
echo ""

# Listar todos los frames disponibles
echo "3. Frames TF disponibles:"
timeout 3s ros2 run tf2_ros tf2_monitor --all-frames 2>&1 | head -30 | sed 's/^/   /'
echo ""

# Verificar que use_sim_time esté correcto
echo "4. Verificando use_sim_time:"
echo "   robot_state_publisher:"
ros2 param get /robot_state_publisher use_sim_time 2>&1 | sed 's/^/     /'
echo ""

# Limpiar archivos temporales
rm -f /tmp/tf_*

echo "========================================="
echo "Verificación completa"
echo "========================================="
