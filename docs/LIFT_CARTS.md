# Carros con plataforma elevadora (bcr_bot tipo Kiva)

El robot se mete debajo de un carro, lo levanta con la plataforma, lo lleva y lo deja. Este documento recoge el **diseño, las mediciones y los límites**.
La interfaz, los parámetros y las guardas de los tramos rectos están en el [README](../README.md#lift-platform-and-carts-this-fork-gz-sim) (sección `Lift platform and carts`); no se duplican aquí.
Cada script tiene además su docstring inicial como contrato.

Procedencia: escrito el 2026-11-04 en `indra/docs/LIFT_CARTS.md`, movido a este repo el 2026-11-08 y reconciliado ese día con el código (rama `ros2`, `6d9aef1`).
El modelo del carro y las estaciones siguen en el paquete `indra`: ver [`indra/docs/world/CARTS.md`](../../indra/docs/world/CARTS.md).

## Estado

| Qué | Estado |
|---|---|
| Plataforma, `pick_cart` y `drop_cart` (implementación del 2026-11-04) | Verificados en simulación aislada con ground truth de Gazebo (ver [Verificado](#verificado-histórico-2026-11-04)) |
| Tramos rectos guardados, `pick_cart` con salida cargado y `place_cart` (`bcc5014`, `6821503`, 2026-11-08) | **Probados solo offline** (pytest con ROS mockeado). Falta validación en simulación aislada: ver [Pendiente](#pendiente-de-validar) |

## Piezas

| Capa | Archivo | Qué hace |
|---|---|---|
| Robot (URDF) | `urdf/bcr_bot.xacro`, `meshes/bcr_bot_{body,top}.dae` | El mesh se partió en cuerpo y bloque superior (faldón + tapa). La tapa es `lift_link`, con un joint prismático `lift_joint` de 0 a 0,10 m |
| Robot (Gazebo) | `urdf/gz.xacro` | `JointPositionController` (PID de fuerza) en `<ns>/lift_cmd`. Tapa con fricción alta (`lift_friction` 1000) |
| Robot (driver) | `scripts/lift_controller.py` | Servicios `lift_up` / `lift_down` (rampa a 0,05 m/s) y `lift_state` |
| Nav2 | `scripts/cart_carry_adapter.py`, `scripts/cart_legs.py` | Con la tapa arriba: filtra las patas del scan (`scan_filtered`), cambia la huella a la del carro y concede el lease de huella de los tramos rectos |
| Tarea | `scripts/cart_handler.py` | Acciones `pick_cart` / `place_cart` / `drop_cart` y servicio `~/reset_fault` |
| Geometría | `scripts/cart_geometry.py` | Geometría plana pura, **sin ROS**, para testearla offline |
| Tests | `test/test_cart_*.py` | pytest offline (ROS mockeado): geometría, tramos rectos, secuencias pick/place/drop, tolerancias, lease y reset de fallos |
| Mundo | `indra/models/lift_cart/` | Ver [`indra/docs/world/CARTS.md`](../../indra/docs/world/CARTS.md) |

Los nodos los lanza este repo en cada robot:
- `lift_controller` → `bcr_bot_gz_spawn.launch.py`.
- `cart_carry_adapter` y `cart_handler` → `bcr_bot_nav2.launch.py`.

`indra_multirobot.launch.py` ya los tiene a todos. Decisión del 2026-11-04: todo queda en `bcr_bot` por ahora; se puede separar a un paquete de aplicación más adelante (ver [Separación pendiente](#separación-pendiente)).

## Interfaz

Todo dentro de `/<ns>`:
- Plataforma: `lift_up`, `lift_down` (`std_srvs/Trigger`), `lift_state` (`std_msgs/Bool`, transient local; para leerlo con CLI: `--qos-durability transient_local --qos-reliability reliable`) y `lift_cmd` (`std_msgs/Float64`, altura sin rampa; bajo nivel, usar `lift_up`/`lift_down`).
- Carros: `pick_cart` y `place_cart` (`nav2_msgs/action/DockRobot`, `use_dock_id: false`, `dock_pose` en `map`), `drop_cart` (`nav2_msgs/action/UndockRobot`) y `cart_handler/reset_fault` (`std_srvs/Trigger`).
- `scan_filtered` (`sensor_msgs/LaserScan`): lo que usan AMCL, los costmaps y el collision monitor. Es igual al scan crudo salvo con un carro levantado.

La altura de la plataforma también sale de `joint_states` (joint `<ns>/lift_joint`) y de la TF `<ns>/lift_link`. Tabla completa, parámetros y ejemplos: README.

Carro para pruebas:

```bash
ros2 run ros_gz_sim create -file $(ros2 pkg prefix indra)/share/indra/models/lift_cart/model.sdf \
  -name lift_cart -x 15 -y -26 -z 0.01 -Y -1.5708
```

Códigos de error (se reusan los de `DockRobot`/`UndockRobot`):

| Código | Significado aquí |
|---|---|
| 901 `DOCK_NOT_IN_DB` | Se pasó `use_dock_id: true`; no hay base de datos de carros |
| 902 `DOCK_NOT_VALID` | `dock_pose` fuera de `map`; en pick, ya lleva carro; en place/drop, no lo lleva; en place, carro sin medir, a más de 0,25 m / `place_carried_yaw_tolerance` (3°) del robot o con corrección de staging mayor que `place_staging_compensation_limit` |
| 903 `FAILED_TO_STAGE` | Nav2 no llegó delante del carro (pick); en place, el robot no empieza a ≤0,5 m del staging o la alineación lo saca de él |
| 904 `FAILED_TO_DETECT_DOCK` | No se ven las patas del carro cerca de la pose dada |
| 905 `FAILED_TO_CONTROL` | No pudo alinearse, entrar, levantar/bajar o salir; en place, carro fuera de la tolerancia de destino (se niega a bajar); fallo de una guarda de tramo recto |

Una maniobra a la vez: un segundo goal mientras hay uno en curso se rechaza. Cancelar el goal detiene la acción en curso. Una cancelación sin confirmar deja un fault latcheado que solo limpia `reset_fault`.

## Cómo funciona

### `pick_cart`

1. **Staging:** `NavigateToPose` a `staging_distance` (1,8 m) detrás del centro del carro, mirando al carro. Con RMF este paso lo hace RMF (`navigate_to_staging_pose: false`).
2. **Localización:** 10 llamadas a `request_nomotion_update` de AMCL, una cada 0,3 s (`localization_updates`, `localization_period`; 0 lo desactiva).
   AMCL no se actualiza con el robot quieto (`update_min_d` 0,25 m, `update_min_a` 0,2 rad), y al parar Nav2 se midió un error de hasta 0,35 m y 9,5°.
   Con este paso bajó a 0,18 m y 0,6° como máximo (7 paradas, ground truth de Gazebo, 2026-11-04).
   Sin él, el rumbo solo ya corre las patas lejanas ~0,3 m, más que `leg_search_radius`.
3. **Percepción:** las 4 patas se buscan en el scan crudo, cerca de donde deberían estar según la pose dada. Se ajustan al modelo del carro (patas en ±0,58, ±0,48) con un ajuste rígido 2D (`cart_legs.py`). Así se obtiene la pose real del carro respecto del robot, **sin depender de AMCL**.
4. **Alineación:** hasta `max_corrections` (3) correcciones con Spin, DriveOnHeading y, si no hay sitio, BackUp, sobre el eje del carro, hasta que el **lateral previsto en el centro del carro** quede ≤ `lateral_tolerance` (4 cm) con rumbo ≤ `yaw_tolerance` (3°). Se comprueba de nuevo a `pre_entry_distance` (1,3 m), donde hay más puntos por pata.
5. **Entrada:** DriveOnHeading hasta el centro del carro y después `lift_up`, con confirmación del estado de la plataforma.
6. **Al levantar:** `cart_carry_adapter` mide dónde quedó el carro respecto del robot y centra ahí el filtro de las patas y la huella del carro (1,24 × 1,04 m, más 0,05 m de padding).
7. **Salida cargado:** marcha atrás `staging_distance` por el eje de entrada con un **tramo recto guardado** (no Nav2): termina con el carro encima, en el staging. Antes de `bcc5014` el pick terminaba bajo el carro.

### `place_cart`

Requiere la plataforma arriba y que el robot empiece a ≤0,5 m del staging del destino (`centro − staging_distance · [cos yaw, sin yaw]`).
1. Pide al adapter la pose medida del carro respecto del robot; la rechaza si no está medida, a más de 0,25 m o más de `place_carried_yaw_tolerance` (3°, la misma del alineado del pick) de yaw relativo, y compensa el offset al calcular el objetivo.
2. Corrige la alineación en el staging con la huella completa del carro (nunca con el lease: no rota ni corrige lateralmente dentro de una estación). Compara contra AMCL con tolerancias de pocos cm (`place_staging_*_tolerance`), y AMCL solo se actualiza al moverse: recién frenado puede estar 17-19 cm / hasta 8° fuera (medido contra Gazebo; tras 10 actualizaciones en reposo, 4-12 cm / 0,3°). Por eso asienta la localización (`localization_updates`, como el pick) antes de cada medición. El chequeo que vale es el hecho ya orientado al rumbo del carro, porque el giro final desplazó la pose hasta 6 cm (medido en P2): cada ronda asienta y mide; si cumple y aún no está orientado, gira y vuelve a medir; si no cumple, corrige y repite (hasta 6 rondas, `PLACE_ALIGN_ROUNDS`). Con `update_min_d/a` = 0,05 en `nav2_factory_params.yaml` (antes 0,25 m / 0,2 rad) el asentamiento mueve a AMCL solo 1-3 cm (antes 4-13).
3. Avanza `staging_distance` con un tramo recto guardado hasta el centro del destino.
4. Comprueba que el centro estimado del carro cumple `place_tolerance_xy` (0,15 m) y `place_tolerance_yaw` (3°); si no, **se niega a bajar**.
5. `lift_down`, confirma y sale marcha atrás al staging con otro tramo recto (ya con la huella del robot).

No navega al centro de la estación con Nav2; `navigate_to_staging_pose` y `max_staging_time` no se usan en esta acción.

### `drop_cart`

`lift_down` donde está y BackUp `backout_distance` (1,3 m). Es el comportamiento original y no usa lease ni tramo recto.

### Tramos rectos guardados

Existen porque con el carro levantado Nav2 no sirve para entrar y salir de una estación justa: el footprint con carro ya pisa celdas ocupadas y un `BackUp` directo abortó con error 714 `COLLISION_AHEAD`. `nav2_msgs/BackUp` instalado no tiene `disable_collision_checks`.
Por eso los tramos staging ↔ centro con carro los hace `cart_handler` en línea recta, con guardas propias:
- Lease renovable de 0,6 s de la huella del robot (`cart_straight_footprint`), solo para tramos rectos, nunca para rotar, y liberado en toda salida.
- El handler vigila el **scan crudo** de 360° en la franja de 0,75 m más allá del borde delantero del carro, sin inflado lateral: cualquier retorno aborta, también uno estático. Es independiente del filtro del adapter.
- Deriva lateral, de yaw y combinada medidas **desde la pose y el rumbo de salida del tramo** (la línea propia del robot, no el eje del carro: un error de rumbo constante `e` daba `recorrido·sin(e)` de «deriva» lateral aunque el robot fuese recto; medido: 1,1° en 1,8 m = 36 mm con yaw 0,000°). El error de rumbo de salida lo acota el alineado inicial (`lateral_tolerance` / `yaw_tolerance`). Defaults 0,03 m / 2° / 0,04 m, techos configurables 0,05 m / 3° / 0,06 m; velocidad máxima 0,15 m/s; el odómetro final debe coincidir con el desplazamiento ordenado a 3 cm.
- Un retorno por debajo del `range_min` del lidar (0,55 m) no dice nada útil: el scan se da por **inválido** y la acción aborta (`invalid scan or incomplete angular coverage`). Única excepción: el retroceso de `place_cart` sin carro. Al salir por el eje de un carro recién dejado el robot pasa entre sus patas (±0,48 m de lado, ~0,12 m de holgura por lado), a menos de `range_min` del lidar. Ahí se aceptan esos retornos **solo si todos** apuntan a una pata del carro soltado (a ≤ `released_leg_margin`, 0,10 m, de su centro; el carro es una caja de patas de 4 cm y su pose respecto del robot es la medida antes de bajarlo). NaN, cobertura incompleta o un retorno que no apunta a una pata siguen abortando. Medido en simulación: sin esta excepción el retroceso abortaba a los 0,26 m.
- Sin TF de scan/odom fresca, cobertura inválida, inconsistencia del lift, pérdida de huella, deriva o cancelación, la acción se detiene; sin confirmación de restauración de la huella queda un fault latcheado (`reset_fault`).

Detalle completo de parámetros y reglas en el README. Las reglas de seguridad ya cerradas no se aflojan sin pedirlo (`CLAUDE.md` de este repo).

## Por qué está hecho así (medido, no supuesto)

Las mediciones siguientes siguen vigentes (contrastadas con el código el 2026-11-08).

- **AMCL no alcanza para entrar.** El error mapa-mundo es de 7–12 cm, y entre el robot y las patas hay 10–12 cm por lado. Alineando con AMCL, el robot quedó a 7,3 cm del centro y después chocó al navegar y al salir. Con la detección por patas queda a ~2 cm (ground truth de Gazebo).
- **Nav2 da por llegado el robot con 0,25 m de error** (`xy_goal_tolerance`). Por eso se corrige la pose después de navegar.
- **Spin no lograba giros chicos.** Con 10 Hz, 0,4 rad/s mínimo y 3,2 rad/s² se pasaba 2–3°. Ahora el `behavior_server` corre a 20 Hz, con 0,1 rad/s y 1,0 rad/s² (`config/nav2_factory_params.yaml`).
- **Spin, BackUp y DriveOnHeading no funcionaban** con frames con namespace. Jazzy lee `local_frame`, que por defecto vale `odom`; ahora vale `<robot_name>/odom`.
- **Lift en Gazebo/DART:**
  - `use_velocity_commands` no movía el joint.
  - Un damping alto en el joint (20000) más la fricción del contacto se comían la fuerza: solo levantaba ~6 kg.
  - Solución: damping 500 (`lift_damping`) y un `d_gain` alto en el PID.
  - El PID es rígido (el recorrido entero en ~0,25 s); la rampa la pone `lift_controller`.
- **`d_gain` y el paso de física (2026-11-12):** con `d_gain=3000` la plataforma funcionaba con un paso de 1 ms, pero con el
  paso de 4 ms de `factory*.sdf` se quedaba ~1,3 cm por debajo de la consigna (`lift_up` fallaba por timeout con tolerancia de 3 mm)
  y temblaba (banda de 2,8 mm). Es independiente del mundo y de la carga. Un plato de 2 kg con paso de 4 ms necesita
  `d_gain × paso / masa` por debajo de ~1, es decir `d_gain` ≲ 500. Con `d_gain=200` (`lift_d_gain` en `bcr_bot.xacro`, junto a
  `lift_p_gain`, `lift_i_gain` y `lift_i_max`, que son argumentos del xacro) la plataforma sigue la consigna con error ≤0,1 mm y se
  asienta en 0,1–0,2 s. Barrido en aislado (mundo vacío, 4 ms): `d` 500, 200, y `p`/`i` de 20000 y 10000 con `d` 300 y 150 dieron lo mismo.
  Sin integral queda un error fijo de 0,8 mm.
- **`lift_controller` y el log:** el mensaje final usaba una sola llamada con severidad `info` o `warn` según el resultado; rclpy
  lanza `ValueError` si una misma línea cambia de severidad, y el callback del servicio moría sin responder (un `lift_up` fallido
  seguido de un `lift_down` correcto dejaba el cliente esperando). Ahora son dos llamadas.

## Verificado (histórico, 2026-11-04)

`indra_multirobot` headless, ground truth de Gazebo. **Implementación anterior a `bcc5014`**: entonces `pick_cart` terminaba bajo el carro (sin salida cargado) y no existía `place_cart`. No cubre los tramos rectos ni las guardas actuales.

| Prueba | Resultado |
|---|---|
| `pick_cart` desde el spawn de `bcr_bot_1`, carro en (15, −26, −90°) | SUCCEEDED en 26 s; robot a 2,4 cm del centro del carro; carro levantado 5,3 cm |
| `NavigateToPose` 2,4 m con el carro | SUCCEEDED, el carro viaja con el robot |
| `drop_cart` | SUCCEEDED en 16 s; el carro queda apoyado y el robot sale |
| `pick_cart` donde no hay carro | 904, "0 of 4 legs" |
| Segundo goal en curso / cancelar | rechazado / CANCELED con el robot detenido |
| Caja de 20 kg sobre la tapa; carro con avance y giro de 96° (banco aislado) | sube sin hundirse; el carro sigue al robot sin deslizarse |

**Recorrido de 10 cm y PID nuevo (2026-11-12)**, banco aislado (mundo vacío, paso 4 ms, un robot parado con `lift_cart` encima,
`d_gain=200`), sin `pick_cart` ni Nav2:

| Prueba | Resultado |
|---|---|
| `lift_up` | éxito en 2,2 s; plataforma a 0,0993 m, sin temblor |
| Carro levantado | z = 0,0732 m (teórico 7,4 cm); 10 s después igual, sin deslizar |
| `lift_down` | éxito en 2,4 s; plataforma a 0,0000 m (banda 0,1 mm); carro de vuelta en z = 0 |
| Dos `lift_up` a la vez | uno con éxito, el otro `lift busy`; el nodo sigue vivo |

Con un `p` más bajo (20000, 10000) no se ha probado el carro real. Tampoco `pick_cart`/`drop_cart` con el recorrido de 10 cm.

Después de esa fecha, con el carro levantado y en estaciones justas, se observó que Nav2 no puede volver al staging (RMF→Nav2 devolvió status 6 cuatro veces y un `BackUp` de 1,6 m dio 714); en una estación de 4 cm `pick_cart` sí funcionó (`lince_manager/fase3/activo/unified-client-control.md`, L4/R2). De ahí salieron los tramos rectos.

## Pendiente de validar

Lo que el pytest offline **no** establece (README y `CLAUDE.md` de este repo):
- Secuencia completa de `bcc5014` en simulación aislada: pick con salida cargado, `place_cart` (entrada, bajada, salida) y el transporte de `recogida_propulsion_2` a `entrega_avionica_1`.
- Distancia de frenado y latencia de cancelación/respuesta a obstáculos en ambos sentidos; pérdida del lease, restauración de la huella completa y `reset_fault` contra costmaps vivos.
- Cobertura del lidar con el carro presente (sombras de las patas): un scan de 360° no garantiza cobertura sin oclusiones.
- Centro y rumbo reales del carro al colocarlo, y si la localización y las guardas de yaw/odom permiten la secuencia completa en estaciones justas.
- **Riesgo abierto:** la tolerancia de colocación de 0,15 m puede no cubrir el error de AMCL medido (hasta 0,18 m / 0,6°). Hipótesis (nunca confirmada) del fallo de la prueba R3 en `pick_cart`: la guarda comparaba el yaw absoluto contra el eje de `map` y rechazaba límites >0,01 m / 1° / 0,02 m; se relajó a 0,03 m / 2° / 0,04 m con deriva relativa al inicio del tramo. La prueba R4 debe capturar la línea `straight-motion maxima` (nivel info) para medir la desviación real.

## Límites conocidos

- **Un solo modelo de carro.** Las medidas son parámetros de `cart_handler` y `cart_carry_adapter` (`leg_x`, `leg_y`, `cart_half_*`, `filter_half_*`).
- **Sin base de datos de carros.** Quien llama pasa la pose. Cada pata se busca a ≤0,25 m de donde debería estar (`leg_search_radius`, default de `cart_handler.py`; el adapter usa 0,2), así que la pose dada tiene que estar bien a ±0,25 m (menos, si hay error de rumbo).
- **Entrada solo por el +x del carro.** El carro es simétrico, así que da lo mismo dar su orientación o la opuesta.
- **Nada cerca de las patas.** Si hay otro objeto a menos de 0,25 m de donde debería estar una pata, puede confundir la detección.
- **Punto ciego mientras lleva el carro.** El scan pierde todo lo que cae dentro de la caja del carro (+5 cm).
- **Con el carro levantado, Nav2 no sirve para maniobras de estación** (ver Verificado); solo los tramos rectos del handler.
- **Spin más suave:** la configuración de Nav2 también hace más suaves los Spin de recuperación.
- **Sin probar:** dos robots con carros a la vez, pasillos estrechos con la huella del carro, ni el robot real.

## Separación pendiente

Decisión del usuario, 2026-11-04. Se discutió que `cart_handler`, `cart_carry_adapter` y `scan_filtered` son de la aplicación, no del robot. Por ahora quedan en `bcr_bot`. Sigue pendiente: `config/nav2_factory_params.yaml` lee directamente `<robot_namespace>/scan_filtered` y no hay el placeholder `<scan_topic>`. Si se separan:
- **Paquete nuevo:** tiene que ir en otro repo (o en la raíz del workspace). colcon **no encuentra paquetes dentro de otro paquete**, y `bcr_bot` e `indra` son paquetes en la raíz de su repo; esto se verificó con `colcon list`.
- **Ajuste en `bcr_bot`:** agregar un placeholder `<scan_topic>` al YAML de Nav2, con `scan` por defecto, para que el Nav2 de `bcr_bot` no dependa de `scan_filtered`.
