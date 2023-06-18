#!/bin/bash
gnome-terminal -e "bash -c 'ssh robotics@10.0.0.2; $SHELL'"
sleep 5
gnome-terminal --geometry 104x8-0--500 -e "bash -c 'source install/setup.bash && ros2 run joy joy_node; $SHELL'"
gnome-terminal --geometry 20x20--500+0 -e "bash -c 'source install/setup.bash && ros2 topic echo bno055; $SHELL'"
gnome-terminal --geometry 20x20--500+450 -e "bash -c 'source install/setup.bash && ros2 topic echo bar02; $SHELL'"
gnome-terminal -e "bash -c 'source install/setup.bash && ros2 run rqt_image_view rqt_image_view; $SHELL'"
gnome-terminal -e "bash -c 'source install/setup.bash && ros2 run rqt_image_view rqt_image_view; $SHELL'"