# ROBOTIS-MANIPULATOR

15/12/10 robotis_manipulator_h_description  added


  URDF model uploaded

15/12/10 robotis_manipulator_h_gazebo   added 


  Gazebo simulation uploaded

15/12/17 robotis_manipulator_h_moveit, robotis_manipulator_h_calc  added


  Moveit setup uploaded
         
         
-------------------------------------------------------------------------------
         
For Simulation Gazebo

  1. roslaunch robotis_manipulator_gazebo robotis_world.launch
  
  2. roslaunch robotis_manipulator_moveit moveit_gazebo.launch
  

    (please add industrial_core before execution 
    
    https://github.com/ros-industrial/industrial_core?files=1)
    
    
  3. rosrun robotis_manipulator_calc calc_node _sim:=true
  

-------------------------------------------------------------------------------

For Real Robot Execution
  
  1. roslaunch robotis_manipulator_moveit moveit_demo.launch 


    (please add industrial_core before execution 
    
    https://github.com/ros-industrial/industrial_core?files=1)


  2. roslaunch robotis_controller robotis_manager.launch (from ROBOTIS-Framework)
  

  (https://github.com/ROBOTIS-GIT/ROBOTIS-Framework/wiki)

  
  3. rosrun robotis_manipulator_calc calc_node
