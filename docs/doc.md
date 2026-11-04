Corrección mía

Antes dije que tu nav2_params.yaml tenía el tuning del footprint real. Era falso: no lo verifiqué, lo deduje del mensaje de un commit. El footprint real ([[0.45,0.32],…], inflación 0.85, rotate_to_goal_heading, yaw 0.05, desired_linear_vel 1.0) está en nav2_params2.yaml. nav2_params.yaml todavía tiene robot_radius: 0.22. En el merge no se perdió nada, porque upstream no tocó nav2_params2.yaml. Lo que sí pasa es que el bloque navigators que sumé fue al archivo que no se usa en producción.

Qué se usa realmente

┌───────────────────────────────────────────────────────────────────────────────────────────────────┬────────────────────────────────────────────────────────────┐
│                                              Archivo                                              │                        Quién lo usa                        │
├───────────────────────────────────────────────────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────┤
│ bcr_bot_gz_spawn_ns.launch.py                                                                     │ indra.launch.py, spawn_robots_only                         │
├───────────────────────────────────────────────────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────┤
│ bcr_bot_gz_spawn_ns_nav2.launch.py                                                                │ factory_rmf, indra_multirobot                              │
├───────────────────────────────────────────────────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────┤
│ nav2_factory.launch.py + nav2_params2.yaml                                                        │ run.yml (producción)                                       │
├───────────────────────────────────────────────────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────┤
│ bcr_bot.xacro, gz.xacro, gz.launch.py, bcr_bot_gz_spawn.launch.py, nav2.launch.py, rviz.launch.py │ Nada de indra. Solo el README y los flujos de upstream.    │
├───────────────────────────────────────────────────────────────────────────────────────────────────┼────────────────────────────────────────────────────────────┤
│ macros.xacro                                                                                      │ Ya tiene prefix:='', o sea que esa capa ya está unificada. │
└───────────────────────────────────────────────────────────────────────────────────────────────────┴────────────────────────────────────────────────────────────┘

Los archivos sin namespace son el legado. El stack _ns es el que corre. Por eso la base del refactor tiene que ser lo tuyo y no lo de upstream.

Problemas que encontré

1. Dos topologías de Nav2 distintas. nav2_factory (la de run.yml) usa nodos globales con frames prefijados y un relay /bcr_bot/tf → /tf. El multirobot usa use_namespace y los nodos en /<ns>/…. Para el mismo robot cambian los nombres de acciones, servicios y amcl_pose.
2. Dos tunings y dos mapas. nav2_params contra nav2_params2, y factory_map.yaml contra factory_map_2.yaml.
3. Fragilidad en el YAML. El multirobot hace params_content.replace("bcr_bot", robot_namespace) sobre el texto y escribe un temporal que nunca borra. Es frágil. Debería ser un placeholder explícito.
4. Código muerto o divergente:
   - launch_map_server se lee y no se usa.
   - Dos archivos copy, que son restos.
   - stereo_camera_enabled tiene default False en un launch y True en otros.
   - El spawn _ns republica el stereo crudo, y _ns_nav2 agrega resize a 640 más tf relays.
5. La pose inicial está hardcodeada en nav2_factory (15, -24, -1.57), duplicando la del spawn.
6. rviz_ns lanza un segundo robot_state_publisher con el mismo nombre y namespace que el de la simulación. Lo vi leyendo el código y no lo probé.

Arquitectura destino

- Xacro: bcr_bot.xacro y gz.xacro salen de las versiones _ns, con robot_namespace. Se borran los _ns, y también gazebo/ign que ya borró upstream.
- Spawn: un solo bcr_bot_gz_spawn.launch.py, que es el _ns_nav2 parametrizado con estos argumentos:
  - robot_namespace
  - launch_nav2 (true o false)
  - stereo_resize
  - map
  - params_file
  - set_initial_pose
- Nav2: un solo nav2_params.yaml con placeholder de namespace, y nav2_factory pasa a ser un wrapper fino sobre el spawn.
- Wrappers: hay que actualizar los 5 launches de indra que hoy llaman a los _ns: indra, spawn_robots_only, indra_multirobot, factory_rmf y run.yml. Es otro repo tuyo, así que van en un commit aparte.
- Limpieza: se borran los dos copy y los params y launches que queden sin uso.

Orden y cómo se verifica cada paso

┌─────────────────────┬──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│        Paso         │                                                              Verificación (estática, sin compilar)                                                               │
├─────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ 1. Unificar xacro   │ xacro … robot_namespace:=bcr_bot debe dar el mismo URDF que hoy bcr_bot_ns.xacro. Con bcr_bot_1 solo cambian los nombres.                                        │
├─────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ 2. Unificar         │ Mismo diff, sobre los bloques <gazebo>.                                                                                                                          │
│ gz.xacro            │                                                                                                                                                                  │
├─────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ 3. Spawn único      │ launch_validator.py del skill, de forma estática.                                                                                                                │
├─────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ 4. Params y Nav2    │ Diff de los parámetros resultantes contra nav2_params2 con bcr_bot ya sustituido.                                                                                │
├─────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ 5. Callers de indra │ rg de que no quede ninguna referencia a un _ns.                                                                                                                  │
├─────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ 6. Prueba real      │ 1 robot con el flujo de run.yml y después 2 robots. Necesita tu permiso para usar el contenedor, y antes y después listar los procesos huérfanos, como dice tu   │
│                     │ CLAUDE.md.                                                                                                                                                       │
└─────────────────────┴──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘

Hasta el paso 5 no hace falta compilar ni tocar el contenedor. Eso es verificación estática; que el robot ande en simulación solo lo confirma el paso 6.

Tres decisiones tuyas

1. ¿Qué tuning gana? Recomiendo nav2_params2 como base, porque es el que corre hoy en run.yml. Su diferencia con nav2_params es un criterio tuyo (velocidad, tolerancias, footprint), no un detalle técnico. El otro lo uso solo para traer el bloque navigators.
2. ¿Topología de Nav2? Recomiendo nodos con namespace para todo. Es lo que ya usa el multirobot, y evita mantener dos formas. El costo es que con un solo robot cambian los nombres de las acciones de Nav2 de /navigate_to_pose a /bcr_bot/navigate_to_pose. No miré cómo ActionBridge ni el GCS llaman a Nav2, y eso habría que revisarlo antes de cambiar.
3. ¿Namespace por defecto? Cambio mi recomendación anterior. Propongo bcr_bot por defecto y no vacío, porque en producción siempre hay namespace y así no se rompe ningún topic de indra. El vacío queda soportado, pero como caso secundario.
