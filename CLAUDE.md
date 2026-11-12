# bcr_bot (fork Indra)

Paquete `ament_cmake` del robot AGV diferencial del proyecto Indra. Fork de `blackcoffeerobotics/bcr_bot`
(remote `origin` = `arpoma16/bcr_bot`, rama `ros2`; `upstream` = Black Coffee Robotics, ramas `ros2` y `ros2-jazzy`).
Se usa dentro del workspace `/ariac_ws` (ROS 2 Jazzy + Gazebo Harmonic). Contexto global: `../CLAUDE.md` e `../indra/CLAUDE.md`.

El README mezcla instrucciones de upstream (Gazebo, MuJoCo, Isaac Sim) con la sección de esta fork
(`Lift platform and carts`). Lo propio de Indra es el stack con namespace, el lift, los carros y Nav2 de fábrica.

## Qué es de la fork y qué es de upstream

| Área | Archivos | Estado |
|---|---|---|
| Robot unificado con `robot_namespace` | `urdf/bcr_bot.xacro`, `urdf/gz.xacro`, `urdf/macros.xacro` | Fork. Plataforma lift Kiva en `lift_joint` (prismática, 0 a 0.10 m, PID de fuerza) |
| Spawn en Gazebo | `launch/bcr_bot_gz_spawn.launch.py` | Fork. `robot_namespace`, `launch_nav2`, `stereo_resize`, `nav2_map`, `nav2_params_file` |
| Nav2 con namespace | `launch/bcr_bot_nav2.launch.py`, `config/nav2_factory_params.yaml` | Fork. Nav2 corre dentro de `/<robot_namespace>` (exige namespace no vacío) |
| Wrapper de fábrica | `launch/nav2_factory.launch.py` | Fork. Envoltorio fino sobre `bcr_bot_nav2.launch.py` (mapa `factory_map_2.yaml`, pose de spawn) |
| Lift y carros | `scripts/lift_controller.py`, `cart_carry_adapter.py`, `cart_handler.py`, `cart_legs.py`, `cart_geometry.py` | Fork. Ver sección siguiente |
| Tests offline | `test/` | Fork (pytest, sin ROS) |
| Upstream sin tocar | `launch/gz.launch.py`, `nav2.launch.py`, `nav2_simple.launch.py`, `mapping*.launch.py`, `mujoco.launch.py`, `rviz.launch.py`, `config/nav2_params.yaml`, `mjcf/`, `usd/`, `worlds/` | Legado/upstream; `indra` no los usa en producción. No refactorizar sin pedirlo |

El mundo, el modelo `lift_cart` y las estaciones viven en el paquete `indra` (`models/lift_cart`, `indra/docs/world/CARTS.md`), no aquí. El diseño, las mediciones y los límites de los carros están en `docs/LIFT_CARTS.md` de este repo; los parámetros, en el README.

## Interfaces de lift y carros (todas dentro de `/<ns>`)

- `lift_controller.py`: servicios `lift_up`/`lift_down` (`std_srvs/Trigger`, con rampa), tópico `lift_state` (`Bool`, transient local), `lift_cmd`.
  Lo lanza `bcr_bot_gz_spawn.launch.py`.
- `cart_carry_adapter.py`: con el carro levantado filtra el scan (`scan_filtered`), cambia el footprint de los costmaps al del carro
  y concede un lease de 0.6 s de footprint del robot (`cart_straight_footprint`, `SetBool`) solo para tramos rectos.
- `cart_handler.py`: action servers `pick_cart` y `place_cart` (`nav2_msgs/DockRobot`, con `use_dock_id: false`) y `drop_cart` (`UndockRobot`),
  más `~/reset_fault`. Mide el carro por sus 4 patas en el láser (`cart_legs.py`).
- `cart_geometry.py` es geometría plana pura, **sin ROS**, para poder testearla offline.
- Los dos últimos nodos los lanza `bcr_bot_nav2.launch.py`.

Los parámetros, límites y errores (`DOCK_NOT_VALID`, `FAILED_TO_STAGE`, ...) están documentados en el README (sección `Lift platform and carts`).
No duplicarlos aquí: si cambia el comportamiento, se actualiza el README.

Reglas de seguridad ya cerradas, no aflojarlas sin pedirlo: los límites rectos (defaults `straight_lateral_limit` 0.03 m, `straight_yaw_limit` 2°, `straight_envelope` 0.04 m, medidos como deriva
desde el offset inicial del carro; velocidad máx 0.15 m/s, lookahead 0.75 m) solo se pueden subir por parámetro hasta los techos 0.05 m / 3° / 0.06 m
(antes 0.01 m / 1° / 0.02 m y sin margen); subir los defaults o los techos hay que pedirlo; el guard del scan crudo es independiente del filtro del adapter; una cancelación sin confirmar
deja un fault latcheado que solo limpia `reset_fault`.

## Comandos

```bash
# Tests offline (no hace falta ROS, build ni contenedor), desde la raíz de este repo
python3 -B -m pytest -q -p no:cacheprovider test/

# Spawn con Nav2 de un robot con namespace
ros2 launch bcr_bot bcr_bot_gz_spawn.launch.py robot_namespace:=bcr_bot_1 launch_nav2:=True position_x:=15.0 position_y:=-24.0

# Nav2 de fábrica sobre un robot ya spawneado por indra (run.yml)
ros2 launch bcr_bot nav2_factory.launch.py
```

- Los scripts se instalan con `install(PROGRAMS ...)` en `CMakeLists.txt`. **Un script nuevo hay que añadirlo ahí**, o el launch no lo encuentra.
- `launch/__pycache__` y `scripts/__pycache__` son ruido (no commitear).
- Build y ejecución en el contenedor `indra_dev_container`: ver `../CLAUDE.md` (no compilar sin que el usuario lo pida).

## Convenciones

- Commits en conventional commits con scope: `feat(cart):`, `fix(nav2):`, `refactor(launch):`, `docs:`. Sin atribución de IA.
- Los scripts de ROS corren dentro del namespace del robot: usar nombres de tópico/servicio **relativos** y frames con prefijo del namespace (`<ns>/odom`, `<ns>/base_footprint`).
- Cada script tiene un docstring inicial que explica contrato y parámetros; mantenerlo al día.
- Los tests importan los scripts con ROS mockeado (`test/conftest.py`); lo que pasa offline NO prueba sensores, frenado ni mensajes reales.
  Los tramos de recogida/entrega siguen necesitando validación en la simulación aislada (ver README).

## Cuidado

- `docs/doc.md` está **desactualizado**: habla de `bcr_bot_gz_spawn_ns*.launch.py`, `nav2_params2.yaml` y `bcr_bot_ns.xacro`, que ya no existen
  (el refactor a un único spawn unificado ya se hizo, ver `git log`). No lo tomes como estado actual.
- `docs/NAV2_NAMESPACE.md` describe el `nav2.launch.py` heredado (namespace por defecto `bcr_bot`), no el flujo de `bcr_bot_nav2.launch.py`.
- `config/` tiene varios mapas (`factory_map*.yaml`, `bcr_map*.yaml`) y `factory_map_2_backup.pgm`/`factory_map_aux.pgm`; el que usa Nav2 de fábrica por defecto es `factory_map_2.yaml`.
- No hay `.claude/` ni `.agents/` con contenido útil (carpetas vacías).
- Con el carro levantado, cualquier movimiento con chequeo de colisión falla: RMF→Nav2 devolvió status 6 cuatro veces y un `BackUp` directo de 1,6 m dio ABORTED error 714 `COLLISION_AHEAD`.
  `nav2_msgs/BackUp` instalado no tiene `disable_collision_checks`: por eso los tramos staging↔centro los hace `cart_handler` en línea recta, no Nav2.
- Riesgo abierto: la tolerancia de colocación de 0,15 m puede no cubrir el error de AMCL medido (hasta 0,18 m / 0,6°). Hipótesis (nunca confirmada) del fallo de la prueba R3 en `pick_cart`: la guarda comparaba yaw absoluto contra el eje de `map` y rechazaba límites >0,01 m/1°/0,02 m; se relajó a 0,03 m / 2° / 0,04 m con deriva relativa al inicio del tramo. La prueba R4 debe capturar la línea `straight-motion maxima` (nivel info) para medir la desviación real.
