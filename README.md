# 2023 Robot
Code for our 2023 competition robot 🤖.

## Controls
We use two joysticks and one ps3 controller to control the robot:

* 2 x **Thrustmaster T16000M** (`leftJoystick` and `rightJoystick`)
* 1 x **ZD-V+ Gaming Controller** (`altJoystick`)

<img src="res/driver1_controls.png" height="400"/>
<img src="res/driver2_controls.png" height="400"/>

## Run Code With Gradle
Gradle allows us to build and deploy our Java code to the robot
1. Right-click on the `build.gradle` file in Visual Studio Code and press `Build Robot Code` to build the code
1. Fix any problems that come up and repeat steps 1 and 2 until the terminal says **BUILD SUCCESSFUL**
    1. A common problem that may come up has to do with formatting, which is shown by the error `Execution failed for task ':spotlessJavaCheck'`. You can fix this problem by running `./gradlew :spotlessApply` in a terminal window
1. Connect to the robot’s wifi
1. Right-click on the `build.gradle` file in Visual Studio Code and press `Deploy Robot Code` to deploy the code