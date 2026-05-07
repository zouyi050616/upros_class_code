gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch upros_description gazebo_upros_base.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch upros_navigation sim_navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch upros_navigation view_nav.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch ccpp_scoring_node ccpp_scoring_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch ccpp_compete_node ccpp_compete_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun upros_transform odom_app_node; exec bash"' \
