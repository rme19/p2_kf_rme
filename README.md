# Filtro de Kalman
Esta práctica consiste en la implementación de filtros de Kalman clásicos en ROS2.

Se han implementado dos versiones del filtro de Kalman:
1. **KalmanFilter**: Este modelo solamente estima la posición y orientación del robot [x, y, θ].
2. **KalmanFilter_2**: Este modelo, además de estimar posición y orientación del robot, también estima las velocidades: [x, y, θ, vx, vy, ω].

Utilizaremos dos modelos matemáticos:
1. Modelo de movimiento (predicción): Formado por una matriz de transición A y una matriz de control B.
2. Modelo de observación (medición): Formado por una matriz C

## KF simplificado
1. **Caso de ruido bajo**
Se configuraron tanto el ruido del proceso como el ruido de la medición a '[0.02, 0.02, 0.01]', que eran los valores por defecto

En la imagen *ruido_bajo1.png* se puede ver que la estimación converge rápidamente a los valores reales, con un error muy pequeño durante todo el proceso.

2. **Caso de ruido alto en la medición**
Se configuró el ruido de medición a '[0.1, 0.1, 0.05]', mientras que al ruido del proceso se le dieron los valores '[0.02, 0.02, 0.01]'.

En la imagen *ruido_alto_obs1.png* se puede ver que la estimación es buena cuando el robot sigue una trayectoria recta hacia un punto, ya que "se fia" mucho de las predicciones, pero, al cambiarle la referencia, el robot tarda un tiempo en que el error entre la estimación y el valor real sea nulo (tiene muy poco en cuenta el valor de los sensores porque considera que es una medida "mala"), apareciendo un considerable error.
 
3. **Caso de ruido alto en el proceso**
Se configuró tanto el ruido de proceso a '[0.1, 0.1, 0.05]', mientras que al ruido del medición se le dieron los valores '[0.02, 0.02, 0.01]'.

En la imagen *ruido_alto_proc1.png* se puede ver que la estimación es mucho más "nerviosa" (no sigue una línea recta perfecta) que en los casos anteriores,debido a que se confía más en las mediciones que en las predicciones. Vemos que cuando le cambiamos el punto objetivo al robot, en las curvas no ocurre lo mismo que en el caso anterior, sino que ahora estaremos siempre sobre la trayectoria real.

## KF puro con velocidad
1. **Caso de ruido bajo**
Se configuraron tanto el ruido del proceso como el ruido de la medición a '[0.02, 0.02, 0.01, 0.02, 0.02, 0.01]', que eran los valores por defecto

En la imagen *ruido_bajo2.png* se puede ver que la estimación converge rápidamente a los valores reales, con un error bastante pequeño durante todo el proceso.

2. **Caso de ruido alto en la medición**
Se configuró el ruido de medición a '[0.1, 0.1, 0.05, 0.1, 0.1, 0.05]', mientras que al ruido del proceso se le dieron los valores '[0.02, 0.02, 0.01, 0.02, 0.02, 0.01]'.

En la imagen *ruido_alto_obs2.png* se puede ver que la estimación tiene casi todo el tiempo un error apreciable (y "constante") y que la estimación permanece paralela a la trayectoria real del robot. Esto se debe a que se tiene muy en cuenta la predicción, pero no se apenas esa predicción con las medidas de los sensores debido a que tienen un mayor ruido.
 
3. **Caso de ruido alto en el proceso**
Se configuró tanto el ruido de proceso a '[0.1, 0.1, 0.05, 0.1, 0.1, 0.05]', mientras que al ruido del medición se le dieron los valores '[0.02, 0.02, 0.01, 0.02, 0.02, 0.01]'.

En la imagen *ruido_alto_proc2.png* se puede ver que la estimación es oscila más en torno a los valores reales que en los casos anteriores,debido a que se considera que la predicción no es tan buena como la medición. En este caso vemos que no ocurre lo mismo que en el caso anterior, ya que aquí siempre estamos en torno a la métrica real y el error.

# Ejecución de los nodos
Para ejecutar los nodos, primero se clonará el repositorio en la carpeta '~/ros2_ws/src' y construiremos el paquete:

`colcon build --packages-select p2_kf_adr`

`source install/setup.zsh`

Ejecutamos la simulación:

`ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true` 

Lanzamos el nodo del filtro de Kalman, que ya incluye la ventana que muestra la trayectoria real del robot y la estimada por el filtro: 

  Para el filtro 1: 

`ros2 run p2_kf_adr kf_estimation`

  Para el filtro 2:

`ros2 run p2_kf_adr kf_estimation_vel`
