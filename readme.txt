
# Source the workspace

source ~/workspaces/paper_ws/devel/setup.bash

### stablish the ur5e communication 
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.100 [reverse_port:=REVERSE_PORT] kinematics_config:=$(rospack find ur_calibration)/ur5e_calibration.yaml

## load the externalcontrol.urp on the robot's control panel and then run the control by external program on the bottom right of control panel. 


rosparam set /robot_description_kinematics/manipulator/position_only_ik true


## launch the movegroup for trajectory planning
roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch 




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Heating process  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
### run the gui node first 
rosrun pcb_desoldering main_hand_v4.py

## run the heating operation node 
rosrun pcb_desoldering main_v2.py 




######################## to collect data point of pcb components #########################



