# Instructions to run the files:

## Dependencies:
  The program requires following libraries to be installaed: rospy, sympy, matlplotlib, math, roboticstoolbox-python
  
## Running publisher-subscriber node:
 
  - Download the package and extract it in your workspace.
  - In a terminal, navigate to your workspace and type following commands:
    $catkin_make
    
    $source devel/setup.bash
    
    $roslaunch ur5urdf project2_ur5.launch
    
  - Next, in a new terminal, run sub.py.
  - Open another terminal, run pub.py.
  
  - You can see the robot moving based on given commands.
  
  ### Output:
    1. By default, robot performs Task-1.
    2. If you want to see Task-2, comment the Task-1 part in pub.py and uncomment Task-2 part.
    3. Rerun the publisher subscriber node.
    
## Forward Kinematics Validation:
  
  - Run ur5_validation.py
  - Run fk_validation.py
  - Compare the results.
    
