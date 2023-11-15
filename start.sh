# Setear URI de ROS en caso de correrlo remotamente
ROS_MASTER_URI="http://duckiebot.local:11311"

# Detección de arucos
python arucodet.py &

# Detección de poses de los aruco
python posedet.py &
