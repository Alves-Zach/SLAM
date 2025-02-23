# Nuturtle  Description
URDF files for Nuturtle
* `ros2 launch nuturtle_description load_one.launch.py` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.
![](images/rviz.png)
* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![](images/rqt_graph.svg)
# Launch File Details
* `ros2 launch nuturtle_description load_one.launch.py --show-args`

  ```
  Arguments (pass arguments as '<name>:=<value>'):

    'use_jsp':
        gui (default): use jsp_gui, jsp: use joint_state_publisher, none: no joint states published
        (default: 'gui')

    'use_rviz':
        true (default): start rviz, otherwise don't start rviz
        (default: 'true')

    'color':
        purple (default): the color of the turtlebot (purple, red, green, or blue). Valid choices are: ['purple', 'red', 'green', 'blue']
        (default: 'purple')
    ```
* `ros2 launch nuturtle_description load_all.launch.xml --show-args`  
  ```
  Arguments (pass arguments as '<name>:=<value>'):

    'use_jsp':
        gui (default): use jsp_gui, jsp: use joint_state_publisher, none: no joint states published
        (default: 'gui')

    'use_rviz':
        true (default): start rviz, otherwise don't start rviz
        (default: 'true')

    'color':
        purple (default): the color of the turtlebot (purple, red, green, or blue). Valid choices are: ['purple', 'red', 'green', 'blue']
        (default: 'purple')
    ```