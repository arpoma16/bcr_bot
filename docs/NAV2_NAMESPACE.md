# Nav2 con namespace en Indra

Esta es la guía de referencia del flujo de navegación de esta fork. Sustituye
la propuesta histórica de [doc.md](doc.md). El flujo heredado de upstream con
`nav2.launch.py` se conserva por separado en el [README](../README.md#using-nav2-with-bcr_bot).

## Launches y configuración actuales

| Archivo | Responsabilidad |
|---|---|
| [bcr_bot_gz_spawn.launch.py](../launch/bcr_bot_gz_spawn.launch.py) | Crea el robot en Gazebo; opcionalmente inicia Nav2 con su pose de spawn. |
| [bcr_bot_nav2.launch.py](../launch/bcr_bot_nav2.launch.py) | Inicia localización, navegación y nodos de carros para un robot ya creado. |
| [nav2_factory.launch.py](../launch/nav2_factory.launch.py) | Wrapper para el robot de fábrica ya creado por Indra; lo utiliza `run.yml` del workspace. |
| [nav2_factory_params.yaml](../config/nav2_factory_params.yaml) | Parámetros del flujo Indra. |
| [factory_map_2.yaml](../config/factory_map_2.yaml) | Mapa predeterminado. |

El argumento del flujo Indra es `robot_namespace`. Nav2 exige que no esté vacío
y usa `bcr_bot` por defecto. El spawn admite namespace vacío para el flujo
heredado, pero exige uno al activar `launch_nav2` (desactivado por defecto).

El YAML utiliza `<robot_name>` para frames, sustituido por el launch, y
`<robot_namespace>` para tópicos, sustituido por `nav2_bringup`. No se necesita
mantener una copia de parámetros por robot. `config/nav2_params.yaml` pertenece
al flujo heredado y no es la configuración de fábrica.

## Uso

Los comandos requieren un entorno ROS 2 Jazzy con los paquetes instalados y
Gazebo con el mundo ya iniciado. No arrancan el mundo.

Para el robot de fábrica ya creado por Indra:

```bash
ros2 launch bcr_bot nav2_factory.launch.py
```

El wrapper usa namespace `bcr_bot`, pose `(15.0, -24.0, -1.57)` (metros y radianes)
y RViz habilitado. La pose debe coincidir con la del robot en el mapa.

Para iniciar Nav2 sobre otro robot ya creado:

```bash
ros2 launch bcr_bot bcr_bot_nav2.launch.py \
  robot_namespace:=robot_1 \
  position_x:=15.0 position_y:=-24.0 orientation_yaw:=-1.57
```

Este launch admite `map`, `params_file`, `use_sim_time`, `autostart`,
`set_initial_pose`, `initial_pose_delay` y `rviz`, además del namespace y la pose.
Por defecto, inicializa AMCL, usa tiempo de simulación y no abre RViz.
Para cambiar mapa o parámetros, pasar sus rutas mediante `map` y `params_file`
al launch `bcr_bot_nav2.launch.py`.

Para crear el robot e iniciar Nav2 en un solo paso:

```bash
ros2 launch bcr_bot bcr_bot_gz_spawn.launch.py \
  robot_namespace:=robot_1 launch_nav2:=True \
  position_x:=15.0 position_y:=-24.0 orientation_yaw:=-1.57
```

En el spawn, los overrides se llaman `nav2_map` y `nav2_params_file` y se
transmiten a Nav2 junto con la pose de spawn. Para varios robots, repetir con
un namespace distinto y la pose correspondiente a cada uno. Iniciar una sola
instancia de Nav2 por robot: si la inicia el spawn, no lanzar además el wrapper.

## Tópicos, acciones y TF

Para `robot_namespace:=robot_1`:

| Interfaz | Tipo o función |
|---|---|
| `/robot_1/scan` | Láser crudo del robot. |
| `/robot_1/scan_filtered` | Láser filtrado por `cart_carry_adapter.py`, consumido por AMCL y navegación. |
| `/robot_1/odom` | Odometría. |
| `/robot_1/cmd_vel` | Comandos de velocidad hacia el robot. |
| `/robot_1/amcl_pose` | Estimación de pose de AMCL. |
| `/robot_1/set_initial_pose` | Servicio de inicialización de AMCL. |
| `/robot_1/navigate_to_pose` | Acción de navegación a una pose. |
| `/robot_1/navigate_through_poses` | Acción de navegación por varias poses. |
| `/robot_1/compute_path_to_pose` | Acción de cálculo de ruta. |

El frame global es `map`; los frames del robot llevan prefijo, por ejemplo
`robot_1/odom`, `robot_1/base_footprint` y `robot_1/base_link`.
Nav2 consume TF del namespace (`/robot_1/tf` y `/robot_1/tf_static`). El spawn
crea también relays a `/tf` y `/tf_static` para consumidores globales.

Con `set_initial_pose:=True`, [set_initial_pose.py](../scripts/set_initial_pose.py)
usa el servicio `/<ns>/set_initial_pose` y reintenta hasta recibir
`/<ns>/amcl_pose`. No inicializa AMCL publicando en `initialpose`.

Los nodos `cart_carry_adapter.py` y `cart_handler.py` arrancan con Nav2.
Sus interfaces y parámetros se mantienen en el
[README](../README.md#lift-platform-and-carts-this-fork-gz-sim), y el diseño y
los límites de los carros en [LIFT_CARTS.md](LIFT_CARTS.md).

## Comprobaciones en una sesión ya iniciada

Estos diagnósticos son de lectura y deben ejecutarse en el entorno ROS de la
simulación. Sustituir `robot_1` por el namespace utilizado:

```bash
ros2 action list -t
ros2 service list
ros2 topic info /robot_1/scan
ros2 topic info /robot_1/scan_filtered
ros2 topic info /robot_1/odom
ros2 topic echo /robot_1/amcl_pose --once
```

Si AMCL no recibe láser, comprobar tanto el scan crudo como el filtrado y el
nodo `cart_carry_adapter`. Si no se inicializa, comprobar el servicio
`set_initial_pose` y la pose configurada. La existencia de tópicos no prueba
que la navegación funcione: hace falta validación en simulación.
