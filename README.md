# Autonomous-Strawberry-Harvester
original workspace name: strawberry_ws

How to Use:

Open terminal and input: cd strawberry_ws (create a workspace if none)

colcon build

source install/setup.bash

ros2 run harvest_bot gui.py

Launch buttons in this order: Launch Hardware > Start Vision > Start Harvest

The Stop Processes button closes everything, including background/ghost processes


Notes:
End Effector connected to tty/USB0 (change accordingly if the user has a different port name in the harvest.py node).
