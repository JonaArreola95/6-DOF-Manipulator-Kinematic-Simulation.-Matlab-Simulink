# 6-DOF-Manipulator-Kinematic-Simulation.-Matlab-Simulink
Description:  During my MSc Course, I developed a simulation of the kinematic model of a 6 DOF robot, along with trajectory plans.  Objective:  The main objective was to learn about the kinematics of a manipulator.

![Frames](https://user-images.githubusercontent.com/100036398/155058729-988bf663-af71-45b1-9c87-272b3052b881.PNG)
![Dimensions](https://user-images.githubusercontent.com/100036398/155058730-9c8a60fa-e2d3-49e8-814b-9dc8fcf92500.PNG)


Simulations described in this report require MATLAB and SIMULINK 2021 to be installed in the computer to be run. Robotics System Toolbox is also required.
Please install it before running the programs if the most recent version is not available.

Forward Kinematics:

MATLAB® Code
This process was programmed using MATLAB. The program to visualize it is in saved under the name:
•	Forwards_Kinematics_Program.m (This program calculates the transformation matrixes for the JonaBot, then it evaluates them using a given set of angles to obtain the T06 and the T0tcp Frames.)
in the coding folder. The code was also included in the section A of the Appendix. Similarly, the code was used to generate functions to be used later in other programs. These functions can be found under the names:
•	forwardKinematicsFuncTCP.m (This function calculates the forward kinematics of a given set of angles (Degrees) for the JonaBot DH parameters. The function returns the transformation matrix of frame {tcp} represented in frame {0} (t0tcp).
•	forwardKinematicsFunc.m (This function calculates the forward kinematics for a given set of angles for the JonaBot DH parameters. The function returns a set of 6 Transformation matrixes from frame {0} to frame {tcp} t01, t12, t23, t34, t45, t56, t6tcp)


Inverse Kinematics:

3.2.4 MATLAB® Code
This process was programmed using MATLAB. The program to visualize it is in saved in the coding folder under the name:
•	Inverse_Kinematics_Program.m (This program calculates the inverse kinematics given a certain target for the JonaBot. It provides the option to rotate or change the position of the target. The program then prints out all the possible solutions and displays the robot model in all these different solutions.
The code text was also included in the section B of the Appendix. Similarly, other functions were generated to be used later in other programs and to support the main or other programs. These functions can be found under the names:
•	inverseKinematicsFuncOp.m (This function calculates the inverse kinematics given a certain target for the JonaBot. The program calculates all the possible solutions. It provides the option to choose from the 8 different solutions as the arguments. It returns a vector of 1X6 containing the angle solution of the JonaBot for the provided target).
•	ImportJonaBot.m (This function imports the JonaBot model tree and shows the provided configuration. The program takes a vector containing the joint values and assigns them to the manipulator configuration. Later, it shows the manipulator. It returns a void function.
![Inverse Kinematics](https://user-images.githubusercontent.com/100036398/155058769-1b00bab9-67af-4c4c-b68b-634268369160.PNG)


Trajectory Planning:

MATLAB® Code
This process was programmed using MATLAB. The program to visualize it is in saved in the coding folder under the name:
•	JointSpace_Program_JonaBot.m (This program creates a joint-space trajectory for the JonaBot. The program uses a cubic polynomial function to generate a smooth trajectory for the manipulator joint angles. Also, this program calls a Simulink model to perform the simulation. 
•	TaskSpace_Program_JonaBot.m (This program creates a task space trajectory for the JonaBot. The user can choose between cubic polynomial trajectory or a trapezoidal trajectory. Also, this program calls a Simulink model to perform the simulation.
•	TorqueFedBackControl_Program_JonaBot.m (This program controls the JonaBot through the torque inputs. The program uses a cubic polynomial function to generate a smooth trajectory for the manipulator joint angles delta calculated. This program calls a Simulink model to perform the simulation.)
![Picture3](https://user-images.githubusercontent.com/100036398/15505![Picture1](https://user-images.githubusercontent.com/100036398/155058903-8faaee3d-98fe-4ba3-a18a-3f09d0ee3d18.png)
8892-fd396ec6-cf11-479b-912b-4d457f57d8af.png)



Redundant Manipulator:

MATLAB® Code
This process was programmed using MATLAB. The program to visualize it is in saved under the name:
•	Forwards_Kinematics_Redundant_Program.m (This program calculates the transformation matrixes for the provided redundant manipulator, then it evaluates them using a given set of angles to obtain the T06 and the T0tcp Frames.)
•	Inverse_Kinematics_Redundant_Program.m (This program calculates the inverse kinematics given a certain target for the provided redundant manipulator. It provides the option to rotate or change the position of the target. The program then prints out all the possible solutions and then it displays the robot model in all these different solutions.)
![Capture](https://user-images.githubusercontent.com/100036398/155059106-6a9af569-329c-42ed-9ac2-937f223a8582.PNG)



