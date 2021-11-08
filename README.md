# Arm-Robot
Mini Project ROS (SFAS)

Please avoid adding files that contain pathfiles specific to your own PC.
you can place a .gitignore file in your repository containing the files you don't want to add to the Github repository.

Program structure:

mini_project_main.py
    contains the general structure and calls the other scripts

    - move_gripper.py 
        contain functions to open and close the gripper file

    - listner.py   
        finds obejct positions

    - (some movement script).py 
        moves the arm to the given position.
