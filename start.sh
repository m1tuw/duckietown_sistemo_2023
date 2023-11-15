# Setear URI de ROS en caso de correrlo remotamente
ROS_MASTER_URI="http://duckiebot.local:11311"

# Detección de arucos
python3 arucodet.py &

# Detección de poses de los aruco
python3 posedet.py &
