# FRONT
rosrun qcar_visnav calib_tool \
  $(rospack find qcar_visnav)/tools/data/front/chessboard_config.yaml -o /tmp/front.yaml

# LEFT
rosrun qcar_visnav calib_tool \
  $(rospack find qcar_visnav)/tools/data/left/chessboard_config.yaml  -o /tmp/left.yaml

# RIGHT
rosrun qcar_visnav calib_tool \
  $(rospack find qcar_visnav)/tools/data/right/chessboard_config.yaml -o /tmp/right.yaml


# What you’ll get:
# RMS error printout + FOV in the console.

# An output file at the path you pass with -o (keep those for the # real cameras later).

# On the real QCar (Ubuntu 18.04 / Melodic) you don’t need to build #this tool at all—just copy the produced YAMLs there and point #your camera drivers at them as the camera_info_url. The color #node only needs the live camera_info topics.

# qcar_visnav/tools/data/front/   <— your images OR a single video
# In each folder, keep a chessboard_config.yaml like