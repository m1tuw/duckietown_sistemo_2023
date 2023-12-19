# Setear URI de ROS en caso de correrlo remotamente
export ROS_MASTER_URI="http://duckiebot.local:11311"
export ROS_IP="10.42.0.1"

# abrir archivos
python3 testarucodet.py &
python3 movement.py
