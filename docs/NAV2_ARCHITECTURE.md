# Sistema de navegación de bcr_bot (Nav2)

Descripción de los componentes de navegación, sus entradas, salidas y funcionamiento.
Diagrama: [nav2_architecture.svg](nav2_architecture.svg). Comandos, namespaces y lanzamiento: [NAV2_NAMESPACE.md](NAV2_NAMESPACE.md).

Fuentes: `config/nav2_factory_params.yaml`, `launch/bcr_bot_nav2.launch.py`, `urdf/bcr_bot.xacro`,
`/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py` y el BT por defecto de `nav2_bt_navigator`.
Todo corre dentro del namespace `/<ns>` (ej. `/bcr_bot`); los frames llevan prefijo (`<ns>/odom`, `<ns>/base_link`).

## 1. Visión general

| | Qué | Detalle |
|---|---|---|
| **Entradas** | Goal | action `navigate_to_pose` (RMF, HMI o RViz) |
| | Mapa | `config/factory_map_2.yaml`, 0.05 m/px, origen (-4.858, -61.360) |
| | LiDAR 2D | `/<ns>/scan`, 361 haces, 360°, 0.55–16 m, 30 Hz |
| | Odometría / TF | `/<ns>/odom` y TF `odom → base_footprint` |
| **Salida** | Velocidad | `/<ns>/cmd_vel` (`geometry_msgs/Twist`: `linear.x`, `angular.z`) |

Robot: tracción diferencial, footprint real 0.90 × 0.64 m (`[[0.45,0.32],[0.45,-0.32],[-0.45,-0.32],[-0.45,0.32]]` + padding 0.05).

### Cadena de salida de velocidad

```
controller_server ─┐
                   ├─► cmd_vel_nav ─► velocity_smoother ─► cmd_vel_smoothed ─► collision_monitor ─► /<ns>/cmd_vel
behavior_server ───┘
```

Las remaps están en `navigation_launch.py`: `controller_server`, `behavior_server` y la entrada de `velocity_smoother` usan `cmd_vel_nav`.
La salida de `collision_monitor` a `/<ns>/cmd_vel` y su entrada `cmd_vel_smoothed` salen del YAML.

### Flujo de una navegación

1. Llega un goal a `bt_navigator` (action `navigate_to_pose`).
2. El Behavior Tree llama a `planner_server` (`ComputePathToPose`) y recibe la ruta global.
3. El BT llama a `controller_server` (`FollowPath`) con esa ruta; este genera velocidades.
4. Las velocidades pasan por `velocity_smoother` y `collision_monitor`, y salen por `cmd_vel`.
5. El BT repite el paso 2 a 1 Hz (replanificación periódica) mientras dura la navegación.

## 2. Componentes

### 2.1 `map_server`
- **Entrada:** `yaml_filename` (se sobrescribe desde el launch con el argumento `map`).
- **Salida:** `/map` (`OccupancyGrid`, transient local).
- **Función:** sirve el mapa estático. Lo consumen la `static_layer` de los costmaps y AMCL.

### 2.2 `cart_carry_adapter` (propio de bcr_bot)
- **Entradas:** `scan` (LiDAR crudo), `lift_state` (`Bool`, plataforma levantada o no).
- **Salidas:** `scan_filtered` (`LaserScan`), footprint de los costmaps, lease de footprint del robot (servicio `cart_straight_footprint`).
- **Función:** con el carro levantado elimina del scan los retornos dentro de la caja del carro, y cambia el footprint de los costmaps al del carro.
  Para tramos rectos guardados concede un lease de 0.6 s con el footprint del robot.
- **Importante:** AMCL, ambos costmaps y `collision_monitor` leen `scan_filtered`, no `scan`.
  El guard del `cart_handler` usa el scan crudo y es independiente.

### 2.3 AMCL
- **Entradas:** `scan_filtered`, `/map`, TF `odom → base_footprint`, `initialpose`.
- **Salidas:** TF `map → odom` (`tf_broadcast: true`); además `amcl_pose` y nube de partículas (comportamiento estándar de AMCL).
- **Función:** localiza el robot en el mapa con un filtro de partículas.
  `laser_model_type: likelihood_field`, `max_beams: 60`, 500–2000 partículas, `DifferentialMotionModel`,
  actualiza al moverse `update_min_d: 0.25` m o `update_min_a: 0.2` rad.
- **Pose inicial:** `set_initial_pose.py` la envía al arrancar (`set_initial_pose:=True`, `initial_pose_delay` 10 s).
- **Riesgo conocido:** el error medido llega a 0.18 m / 0.6°, y la tolerancia de colocación del carro es 0.15 m (ver `../CLAUDE.md`).
- **Quién la consume:** nadie recibe un mensaje directo. Todo lo que trabaja en frame `map` depende de la TF `map → odom`.

### 2.4 `bt_navigator`
- **Entradas:** action `navigate_to_pose` / `navigate_through_poses`, `odom_topic: <ns>/odom`, TF.
- **Salidas:** llamadas de action a `planner_server`, `controller_server` y `behavior_server`; resultado de la navegación con códigos de error (`compute_path_error_code`, `follow_path_error_code`).
- **Función:** ejecuta el Behavior Tree. Usa el BT por defecto `navigate_to_pose_w_replanning_and_recovery.xml`:
  - `PipelineSequence`: replanifica con `RateController hz=1.0` y ejecuta `FollowPath`.
  - Recuperaciones (hasta 6 reintentos): limpiar costmaps, `Spin` 1.57 rad, `Wait` 5 s, `BackUp` 0.30 m a 0.15 m/s.
- **Es el que conecta todo:** el planner, el smoother y el controller no se pasan datos entre sí; los orquesta el BT.

### 2.5 `planner_server`
- **Entradas:** goal y pose de inicio (por `ComputePathToPose` desde el BT, la pose desde TF) y `global_costmap`.
- **Salida:** la ruta (`nav_msgs/Path`) en el resultado de la action, que recibe `bt_navigator`.
- **Función:** planificación global con `nav2_navfn_planner::NavfnPlanner`, `use_astar: true`, `tolerance: 0.5`, `allow_unknown: true`.
  Puede aceptar un destino cercano si el exacto no es alcanzable.

### 2.6 `smoother_server`
- **Entradas / salidas:** ruta de entrada y ruta suavizada, por action `SmoothPath`.
- **Función:** `SimpleSmoother`, `max_its: 1000`, `do_refinement: True`.
- **Atención:** el BT por defecto **no llama a `SmoothPath`**, así que con la configuración actual este nodo está activo
  pero no participa en la navegación normal. Solo intervendría con un BT propio que lo incluya.

### 2.7 `global_costmap` (dentro de `planner_server`)
- **Entradas:** `/map` (`static_layer`), `scan_filtered` (`obstacle_layer`, marca hasta 2.5 m, raytrace 3.0 m), footprint.
- **Salida:** grilla de costos en frame `map`, a 5 Hz (publica a 2 Hz).
- **Capas:** `static_layer`, `obstacle_layer`, `inflation_layer` (radio 0.85 m, factor 1.5).
  `track_unknown_space: true`.

### 2.8 `local_costmap` (dentro de `controller_server`)
- **Entradas:** `scan_filtered` (`voxel_layer`), footprint.
- **Salida:** grilla de costos, ventana móvil 5 × 5 m, 0.05 m, frame `<ns>/odom`, a 10 Hz (publica a 5 Hz).
- **Capas:** `voxel_layer` (16 voxels de 0.05 m) e `inflation_layer` (0.85 m).
- **Nota:** `static_layer` está declarada en el YAML pero no figura en `plugins`, así que no se usa. Ver sección 5.

### 2.9 `controller_server`
- **Entradas:** ruta (`FollowPath`, desde el BT), `local_costmap`, TF y odometría.
- **Salida:** `cmd_vel_nav` (`Twist`).
- **Función:** a 20 Hz, `RotationShimController` envolviendo a `RegulatedPurePursuitController`.
  - El shim rota en sitio cuando el error angular supera 0.785 rad (`angular_dist_threshold`), y luego cede el control a pure pursuit.
  - RPP: `desired_linear_vel: 1.0`, `lookahead_dist: 0.6` (0.3–1.0), sin reversa, desaceleración al aproximar el goal.
  - Detección de colisión sobre la ruta cercana con `max_allowed_time_to_collision_up_to_carrot: 1.2` s.
  - Chequeo de meta: `xy_goal_tolerance 0.25` m, `yaw_goal_tolerance 0.05` rad.
  - Progreso: debe moverse 0.5 m o 0.5 rad cada 15 s o falla.

### 2.10 `behavior_server`
- **Entradas:** actions `spin`, `backup`, `drive_on_heading`, `wait`, `assisted_teleop`; `local_costmap`; TF.
- **Salida:** `cmd_vel_nav`.
- **Función:** maniobras de recuperación del BT y acciones que usa `cart_handler`. A 20 Hz.
  Frames: `local_frame: <ns>/odom`, `global_frame: map`.
- **Ojo:** `BackUp` y `DriveOnHeading` hacen chequeo de colisión; con el carro levantado pueden abortar con `COLLISION_AHEAD`.
  Por eso los tramos rectos del carro los hace `cart_handler` y no Nav2 (ver `../CLAUDE.md`).

### 2.11 `velocity_smoother`
- **Entrada:** `cmd_vel_nav`. **Salida:** `cmd_vel_smoothed`.
- **Función:** limita y suaviza. 20 Hz, `OPEN_LOOP`, v ±1.0 m/s, ω ±2.0 rad/s, aceleración 3.0 m/s² y 4.0 rad/s², `y = 0`.

### 2.12 `collision_monitor`
- **Entradas:** `cmd_vel_smoothed` y `scan_filtered`; footprint publicado de `local_costmap`.
- **Salida:** `/<ns>/cmd_vel` y estado en `collision_monitor_state`.
- **Función:** último filtro, independiente del planner y del BT. Polígono `FootprintApproach`
  (`action_type: approach`, `time_before_collision: 2.0` s, `min_points: 6`): reduce la velocidad hasta frenar antes de la colisión.
  Sigue el footprint publicado, incluido el cambio con el carro.

### 2.13 Nodos de carros (propios de bcr_bot)
- **`lift_controller.py`:** servicios `lift_up` / `lift_down`, tópico `lift_state`.
- **`cart_handler.py`:** action servers `pick_cart`, `place_cart` (`DockRobot`) y `drop_cart` (`UndockRobot`).
  Usa como clientes `navigate_to_pose`, `drive_on_heading`, `spin`, `backup`, `lift_up/down` y `cart_straight_footprint`;
  lee `scan` crudo y `lift_state`. Detalle y errores en el README, sección `Lift platform and carts`.

### 2.14 Presentes pero sin uso en el flujo principal
`waypoint_follower` y `docking_server` están en el stack. El `docking_server` solo tiene una configuración mínima
(`dock1` en (0,0,0)) porque el servidor exige un plugin; el docking de carros lo hace `cart_handler`.

## 3. Qué pasa ante un obstáculo que no está en el mapa

1. El scan, ya filtrado, marca el obstáculo en `obstacle_layer` (global) y `voxel_layer` (local), con inflación de 0.85 m.
   Solo se marca hasta 2.5 m (`obstacle_max_range`).
2. **Reacción inmediata, sin planner:** el controller (colisión a 1.2 s) y el `collision_monitor` (2 s) frenan el robot.
3. **Replanificación:** el BT vuelve a llamar al planner cada 1 s; el nuevo `global_costmap` ya contiene el obstáculo y la ruta lo rodea.
4. Si no hay ruta o el robot se atasca, entran las recuperaciones del BT (sección 2.4). Si fallan todas, la navegación aborta con código de error.

## 4. Limitaciones y puntos abiertos

- El `smoother_server` no interviene con el BT por defecto (sección 2.6).
- Obstáculos detectados a ≤ 2.5 m: a 1.0 m/s dejan poco margen; el `collision_monitor` es la red de seguridad.
- Con el carro levantado el footprint cambia y los chequeos de colisión de Nav2 son estrictos (ver `../CLAUDE.md`).
- El error de AMCL puede superar la tolerancia de colocación del carro.
- Lo marcado como "estándar de AMCL" (`amcl_pose`, partículas) no se comprobó contra el código instalado.

## 5. Por qué el `local_costmap` no usa `static_layer`

**Estado actual** (`config/nav2_factory_params.yaml`): el bloque `static_layer` existe en `local_costmap`,
pero `plugins: ["voxel_layer", "inflation_layer"]` no la nombra. Nav2 solo carga lo que está en `plugins`,
así que el bloque es configuración muerta (no hace nada, ni mal ni bien).

**Origen:** no hay un motivo registrado en el repo ni en el historial de git. `config/nav2_params.yaml`
(upstream, sin tocar) tiene lo mismo, así que es un valor heredado, no una decisión del proyecto.
Que el local costmap vaya sin mapa estático es el patrón habitual de Nav2: el local costmap se pensó
como una vista reactiva de lo que el sensor ve ahora. Esta explicación es de diseño general de Nav2, no de este repo.

**Cómo funciona hoy:** el local costmap solo contiene lo que el LiDAR marca en la ventana de 5 × 5 m,
más la inflación. Las paredes y estructuras fijas del mapa solo las conoce el `global_costmap`.

### Pros de no usarla
- **Independiente de la localización:** el local costmap vive en el frame `<ns>/odom`. Si AMCL deriva
  (error medido de hasta 0.18 m), el controller no se ve afectado: no pega un mapa mal alineado sobre lo que el sensor ve.
- **Menos falsos obstáculos:** un mapa desactualizado (una puerta, una pieza movida desde que se hizo `factory_map_2`)
  no genera obstáculos fantasma en el control local.
- **Cuesta menos:** una capa menos que actualizar y que depender de que `/map` esté publicado.
- **Coherente con el carro levantado:** con el carro, `scan_filtered` quita los retornos del carro. Una capa estática
  no tiene nada que ver con eso y no añade conflictos al footprint cambiante.

### Contras
- **Sin memoria del mapa:** lo que está fuera del alcance del láser (obstacle_max_range 2.5 m) o en un punto ciego
  no existe para el controller. Una pared que el LiDAR no ve bien (vidrio, superficie oscura) no aparece.
- **Un obstáculo "desaparece" al salir del rango:** el voxel_layer limpia con raytracing; si el robot se aleja o rota, el
  controller puede olvidar algo que el mapa sí conocía.
- **Ruta global y control local pueden discrepar:** el planner evita paredes del mapa, el controller solo evita lo que ve;
  si una pared no se ve, el controller puede aprobar una ruta que el global ya consideraba cerrada.
- **Maniobras ajustadas:** en pasillos estrechos o al entrar bajo un carro, el controller dispone de menos información
  que con el mapa como respaldo.

### Si se quisiera activarla
Habría que cambiar a `plugins: ["static_layer", "voxel_layer", "inflation_layer"]`. Antes, valorar:
1. Con `rolling_window: true`, `static_layer` sigue siendo válida (se reposiciona con la ventana), pero depende de la TF `map → odom` de AMCL.
2. El error de AMCL (0.18 m) pasaría a influir en el control local y en las maniobras de colocación del carro, que ya tienen una tolerancia de 0.15 m.
3. Los comportamientos con chequeo de colisión (`BackUp`, `DriveOnHeading`) ya fallan con `COLLISION_AHEAD` con el carro levantado; una capa más podría empeorarlo.
4. Requiere validar en simulación aislada, no solo en tests offline (ver `../CLAUDE.md`).

Recomendación: **dejarla como está**. El sistema ya depende de AMCL para el planner global, y aislar el control local
de esa incertidumbre es una ventaja real en este proyecto. Lo único que conviene es borrar o comentar el bloque
`static_layer` del `local_costmap` para que el YAML no sugiera que está activo.
