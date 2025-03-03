gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch upros_bringup bringup_w4a_t.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch upros_navigation navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch clean_robot area_path.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch upros_navigation view_nav.launch; exec bash"' \
--tab -e 'bash -c "sleep 13; roslaunch clean_robot next_goal.launch; exec bash"' \