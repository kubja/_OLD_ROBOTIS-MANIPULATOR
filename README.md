# ROBOTIS-MANIPULATOR

15/12/10 robotis_manipulator_h_description added
  URDF model uploaded

15/12/10 robotis_manipulator_h_gazebo added 
  Gazebo simulation uploaded

15/12/17 robotis_manipulator_h_moveit
         robotis_manipulator_h_calc    added
         
         
         
For Simulation Gazebo

  1. roslaunch robotis_manipulator_gazebo robotis_world.launch
  
  2. roslaunch robotis_manipulator_moveit moveit_gazebo.launch
  
  3. rosrun robotis_manipulator_calc calc_node _sim:=true
  

For Real Robot Execution
  
  1. roslaunch robotis_manipulator_moveit moveit_demo.launch 
  
  2. roslaunch robotis_controller robotis_manager.launch (ref ROBOTIS-Framework)
  
  3. rosrun robotis_manipulator_calc calc_node
