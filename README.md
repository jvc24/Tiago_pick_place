

Commands to run assignement 1

roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=iaslab_assignment1 gui:=true verbose:=true
roslaunch tiago_iaslab_simulation navigation.launch
roslaunch tiago_iaslab_simulation apriltag.launch
rosrun tiago_iaslab_simulation apriltag_ids_generator_node
roslaunch ir2425_group_26 assignment.launch


Commands to run assignement 2

roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=iaslab_assignment2 gui:=true verbose:=true
roslaunch tiago_iaslab_simulation navigation.launch
roslaunch tiago_iaslab_simulation apriltag2.launch
rosrun tiago_iaslab_simulation get_straightline_node
rosrun ir2425_group_26_asign_2 nodeA_as2.py
rosrun ir2425_group_26_asign_2 nodeB_as2.py
rosrun ir2425_group_26_asign_2 nodeC_as2.py
