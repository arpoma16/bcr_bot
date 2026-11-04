# Nav2 con Namespace para Multi-Robot

## Descripción

La configuración de Nav2 ha sido modificada para usar namespaces, permitiendo múltiples robots simultáneos sin conflictos de topics.

## Cambios Realizados

### 1. Launch File ([nav2.launch.py](../launch/nav2.launch.py))
- Agregado soporte para namespace con `GroupAction` y `PushRosNamespace`
- Namespace por defecto: `bcr_bot`
- Todos los nodos de nav2 se lanzan dentro del namespace

### 2. Parámetros ([nav2_params.yaml](../config/nav2_params.yaml))
- Removidos prefijos `/bcr_bot/` de todos los topics
- Ahora usan topics relativos que se resuelven al namespace
- Topics actualizados:
  - `scan_topic: scan` → `/bcr_bot/scan`
  - `odom_topic: odom` → `/bcr_bot/odom`

### 3. Nodos Personalizados

#### set_initial_pose.py
- Modificado para publicar en `initialpose` (relativo)
- Se convierte en `/bcr_bot/initialpose` con el namespace

#### remapper.py
- Actualizado para trabajar con topics relativos
- Nota: Puede no ser necesario si nav2 está correctamente namespaciado

## Topics Importantes

Con namespace `bcr_bot`, los topics son:

### Entrada (del robot):
- `/bcr_bot/scan` - Datos del lidar
- `/bcr_bot/odom` - Odometría del robot
- `/bcr_bot/joint_states` - Estados de las articulaciones

### Salida (a nav2):
- `/bcr_bot/cmd_vel` - Comandos de velocidad desde nav2
- `/bcr_bot/goal_pose` - Objetivo de navegación
- `/bcr_bot/initialpose` - Pose inicial para AMCL

### Servicios:
- `/bcr_bot/navigate_to_pose` - Navegar a una pose
- `/bcr_bot/navigate_through_poses` - Navegar por múltiples poses
- `/bcr_bot/compute_path_to_pose` - Calcular ruta

### Tópicos de visualización:
- `/bcr_bot/local_costmap/costmap` - Costmap local
- `/bcr_bot/global_costmap/costmap` - Costmap global
- `/bcr_bot/plan` - Ruta planificada
- `/bcr_bot/particle_cloud` - Partículas de AMCL

## Uso

### Lanzar Nav2 con namespace por defecto (bcr_bot):

```bash
ros2 launch bcr_bot nav2.launch.py
```

### Lanzar con namespace personalizado:

```bash
ros2 launch bcr_bot nav2.launch.py namespace:=robot_1
```

### Enviar objetivo de navegación:

```bash
# Con namespace por defecto
ros2 topic pub /bcr_bot/goal_pose geometry_msgs/msg/PoseStamped "..."

# Con namespace personalizado
ros2 topic pub /robot_1/goal_pose geometry_msgs/msg/PoseStamped "..."
```

### Usar desde Python:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class Navigator(Node):
    def __init__(self, namespace='bcr_bot'):
        super().__init__('navigator', namespace=namespace)

        self.goal_pub = self.create_publisher(
            PoseStamped,
            'goal_pose',  # Relativo al namespace
            10
        )

    def send_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        # ... configurar orientación

        self.goal_pub.publish(goal)

# Usar con namespace
navigator = Navigator(namespace='bcr_bot')
```

## Multi-Robot

Para usar múltiples robots:

```bash
# Terminal 1 - Robot 1
ros2 launch bcr_bot nav2.launch.py namespace:=robot_1

# Terminal 2 - Robot 2
ros2 launch bcr_bot nav2.launch.py namespace:=robot_2
```

Cada robot tendrá sus propios topics:
- Robot 1: `/robot_1/goal_pose`, `/robot_1/cmd_vel`, etc.
- Robot 2: `/robot_2/goal_pose`, `/robot_2/cmd_vel`, etc.

## Verificación

Para verificar que todo funciona correctamente:

```bash
# Listar todos los topics del namespace
ros2 topic list | grep /bcr_bot

# Ver servicios disponibles
ros2 service list | grep /bcr_bot

# Verificar que nav2 esté recibiendo datos del lidar
ros2 topic hz /bcr_bot/scan

# Verificar odometría
ros2 topic hz /bcr_bot/odom
```

## Solución de Problemas

### Nav2 no recibe datos del lidar
Verificar que el topic esté publicándose:
```bash
ros2 topic echo /bcr_bot/scan --once
```

### AMCL no se inicializa
Verificar que `set_initial_pose.py` esté corriendo y publicando en el namespace correcto:
```bash
ros2 topic echo /bcr_bot/initialpose
```

### Comandos de velocidad no llegan al robot
Verificar la cadena de topics:
```bash
# Nav2 debería publicar aquí
ros2 topic hz /bcr_bot/cmd_vel

# El robot debería suscribirse aquí
ros2 topic info /bcr_bot/cmd_vel
```

## Archivos Modificados

1. [launch/nav2.launch.py](../launch/nav2.launch.py)
2. [config/nav2_params.yaml](../config/nav2_params.yaml)
3. [scripts/set_initial_pose.py](../scripts/set_initial_pose.py)
4. [scripts/remapper.py](../scripts/remapper.py)

## Referencias

- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS2 Namespaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#remapping)
