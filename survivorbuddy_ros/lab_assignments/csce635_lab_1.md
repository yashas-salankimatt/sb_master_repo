# Programming Lab 1: Reflexes

## Learning Objectives

- Translate schema theory into practice.
- Familiarity with the `MoveIt` simulator, Robot Operating System (`ROS`), and transferring code from simulator to physical world.

## Things to bring along

- Laptop (with Ubuntu 20.04 installed)
- USB-A to USB-C adapter if you only have USB-C ports on your laptop

## Introduction

In this assignment, you'll program a basic "Startle" behavior schema, test in `MoveIt`, then on a physical robot called `Survivor Buddy (SB)`. "Startle" consists of a perceptual schema and a motor schema.

### Perceptual Schema

- `Goal`: Detect audio alert.
- `Input`: Audio stream.
- `Ouput`: ALERT, a value {0, 1} where 1 is if a loud noise is detected and 0 otherwise
- `Parameter`: Threshold value for the loud noise (you will want to adjust the sensitivity)

> _Note:_ The perceptual schema has to filter the audio to detect that a short, sharp burst (aka narrow peak in the waveform) has occurred.

### Motor Schema

- `Goal`: Generate a trajectory that mimics "Startle" behavior. The exact behavior is left on you as whether you want SB fearful, surprised, or curious.
- `Input`: ALERT signal from the perceptual schema.
- `Output`: Motion involving at least 2 degrees of freedom on SB.

### Extra credit (up to 25%)

- Have SB return to its original pose after `N` seconds in a naturalistic way. `N` would be a parameter like the noise threshold.
- Have the severity of the reaction be proportional to the intensity of the noise, i.e., a very loud noise produces a more extreme movement.
- Turn in clearly different (and believable) motions for all three: fearful, surprised, and curious.

### Things to keep in mind

- **Rapid reflexes**: The reaction needs to be fast, not 3 seconds after you clap.
- **Not breaking the motors or linkages by overextending**: Does your program expect the robot to start from the same location (aka a neutral position) every time? Or does it handle reacting within safe operational range of the links from any starting location?
- **Parameters**: How does the program let the sensitivity of volume of the clap be changed? Is it hardcoded, a command line parameter, or dynamic?

## Procedure

### Step 0: ROS & MoveIt

Make sure you have ROS Noetic, and MoveIt installed and working. You'll also need set up a ROS (catkin) workspace. The catkin workspace is helpful in code organization and distribution. You may do this using the following commands.

```sh
mkdir -p ~/catkin_ws/src
```

Next, clone [this repository](https://github.com/yashas-salankimatt/sb_master_repo.git) with this command

```sh
git clone https://github.com/yashas-salankimatt/sb_master_repo.git --recurse-submodules
```

and add it under `~/catkin_ws/src`. This is a ROS package for interfacing with `SB` that has already been created for you. To build the package, run

```sh
cd ~/catkin_ws/
catkin_make
```

To make sure that the package will always sourced when launching a new terminal, run

```sh
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

### Step 1: Enable sensors

In order to detect a clap/loud noise, you will need to use an audio sensor (mic) for SB to detect a clap/loud noise. To do this, set up the SB head web app by navigating to [this repository](https://github.com/yashas-salankimatt/sb_web/) and following the instructions in `README.md`. This repository should already be cloned into `~/catkin_ws/src` from Step 0.

> Note: You may either launch the web app on your laptop or your phone (currently only works without glitch on Android). The mic on that device would act as the sensor to detect a clap/loud noise.

Once, the sensor is set up, add your implementation of the noise detection logic within `detect_noise()` in the space below `"YOUR CODE HERE"` in `lab_1_reflexes.py`. Note that `detect_noise()` should return a boolean value that is `True` if an `ALERT` is detected and `False` if otherwise.

To test your implementation, run

```sh
python lab_1_reflexes.py
```

and you should see `"Noise detected"` being logged into the terminal when a lound noise is detected.

### Step 2: Execute behavior in MoveIt

Next, you need to program a "startle" behavior when an `ALERT` is detected. Once, the sensor is set up, add your code that executes your desired behavior `execute_behavior()` in the space below `"YOUR CODE HERE"` in `lab_1_reflexes.py`. This is based on the Move Group Interface and for more details, you may refer to [this tutorial](https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html).

> Note: You're free to modify the starter code provided within `execute_behavior()` to create your own custom behaviors.

To start `MoveIt`, in a new terminal window run

```sh
roslaunch sb_moveit_config demo.launch
```

Once you have programmed your desired behavior in `execute_behavior()`, to test your behavior run

```python
python lab_1_reflexes.py
```

and you can see the behavior being executed in `MoveIt`.

> Note: For the safety of the robot, the angle by which each joint can be moved from the rest position has been restricted to stay within [-45, 45] degrees. At the rest position, all the joint angles are 0 degrees.

### Step 3: Test on SB

Power up SB by connecting to a power outlet. Connect the mini-USB to USB-A cable (will be provided) to create a communication interface between your laptop and the Arduino on SB. The micro-USB end goes into SB.

Follow the instructions at [this repository](https://github.com/yashas-salankimatt/sb_web/) to get the sensor data from the phone and the Survivor Buddy hardware hooked up correctly.

Once you confirm that the interfacing has been successful, run your code

```python
python lab_1_reflexes.py
```

and record a video of you clapping/making a loud noise and SB executing the "Startle" behavior.

## Grading

In order for your submission to be eligible for grading

- Show that your code works in `MoveIt`. Show this demo to the TA **before** testing it on SB.
- Record a video (with audio) of your code working on SB and upload it to Google drive/YouTube/Vimeo etc. The video should contain a person clapping or a loud noise and the robot's reaction for at least 2 different settings of noise sensitivity, plus any variations.

## Submission

To submit the assignment, please upload the following files to **Canvas -> Assignments -> Programming lab 1**: your script named `lab_1_reflexes.py`, a `video.txt` file containing a link to the demo video, and a `instructions.txt` file containing instructions on how to run your code if it other than running `python lab_1_reflexes.py` (e.g. if you adapted your code to accept inputs from the command line). Please upload the files in a single zip file named **<your UIN>.zip**.
