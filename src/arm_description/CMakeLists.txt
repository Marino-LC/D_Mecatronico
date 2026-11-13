cmake_minimum_required(VERSION 3.8)
project(arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Instalar los directorios en el espacio compartido del proyecto
install(DIRECTORY
  urdf
  meshes
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Nodos ejecutables (ros2 run arm_description <script>.py)
install(PROGRAMS
  scripts/teach_repeat.py
  scripts/teach_pendant_gui.py
  scripts/serial_bridge.py
  scripts/control_torque.py
  scripts/interfaz_trayectoria.py
  scripts/diseno_pid.py
  scripts/simulacion_offline.py
  DESTINATION lib/${PROJECT_NAME}
)

# Módulos que importan los nodos anteriores (deben quedar en la misma carpeta)
install(FILES
  scripts/modelo_brazo.py
  scripts/nucleo_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
