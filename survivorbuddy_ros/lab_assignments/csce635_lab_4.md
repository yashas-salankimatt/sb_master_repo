# Programming Lab 4: Multirobot Systems

## Learning Objectives

- Familiarity with multirobot systems.

## Things to bring along to the lab

- Laptop (with Ubuntu 20.04 installed)
- USB-A to USB-C adapter if you only have USB-C ports on your laptop.

## Introduction

The assignment consists of a group of 4 Survivor Buddys perform to a suitable for "work appropriate" song that is about 2-3 minutes long. This is a team project where students have been assigned to different teams randomly. Please see Canvas/Campuswire for the team assignments.

The team decides on what type of MRS (multirobot system) to use, choose the song, and the motions. The effect should be something like [these dancing Spots](https://youtu.be/7atZfX85nd4) where the motions are synchronized and is fun.

## Extra credit (up to 25%)

- Have the SBs perform to your head movements (headbobbing) to the song by using input from the camera feed. However, not all of them should mimic your head motions (try to be creative here).
- Matching appropriate display on the phones.

### Things to keep in mind for the programmer

- Showing the code works on RViz.
- **Not breaking the motors or linkages by overextending**: Does your program expect the robot to start from the same location (aka a neutral position) every time? Or does it handle reacting within safe operational range of the links from any starting location?

## Conceptual and implementation questions

- What type of MRS did you use? Explain in detail based on the text from the book.

## Procedure

### Step 0: Get the updated code

Get the latest updates from the repo (`git pull` recommended). Then `cd` into you catkin workspace and run

```sh
cd ~/catkin_ws/
catkin_make
```

Create a script named `lab_4.py` under `survivorbuddy_ros/src` and add all your code for the lab to it. Feel free to adapt the starter code from any of the previous labs. 

### Step 1: Decide how to place the SBs

A new URDF has been created [here](https://github.com/yashas-salankimatt/sb_master_repo/blob/master/survivorbuddy_ros/urdf/4_survivorbuddy.urdf) in which the SBs stay next to each other by default. However, if you wish to place the SBs in a different configuration, change the `xyz` and `rpy` of the robots. You may do so by looking up the lines marked `<!-- CHANGE THIS LINE -->`. The configuration of the robots in RViz and physically should be the same.

### Step 2: Controlling the SBs in RViz

Next, run

```sh
roslaunch lab4_helper.launch
```

and publish the positions to the `/sb_<robot_num>_cmd_state` ROS topic where `<robot_num>` is the robot number between 0-3.

>__Note:__ A script for motion smoothing on the robots has been added under `survivorbuddy_ros/src/multi_sb_rviz.py`. This file is called in the launch file by default.

### Step 3: Testing connection between your machine and SBs

To test if you're able to control the physical robots, 
1. Run `roslaunch lab4_helper.launch`
2. `cd` to `survivorbuddy_ros/helper_scripts` and run

```sh
python3 multi_robot_test_positions.py
```
This is similar to `sb_test_joint_positions.py` from the previous labs but for all the 4 robots instead of a single robot.

>__Note:__ We will modify the Arduino code on each robot (should be ready by the week starting 04/03) to ensure that each robot subscribes to the right topic e.g., robot 0 subscribes to the correspoding ROS topic `/sb_0_cmd_state`.

### Step 4: Demo on SBs

To run your code on the SBs, `cd` to `survivorbuddy_ros/src` and run

```sh
python3 lab_4.py
```

## Grading

In order for your submission to be eligible for grading

- A PDF file containing answers to the conceptual and implementation questions.
- Record a video (with audio) of your code working on SB and upload it to Google drive/YouTube/Vimeo etc.

## Submission

To submit the assignment, please upload the following files to **Canvas -> Assignments -> Programming lab 4**: your script named `lab_4.py`, the html file(s) (if any) e.g., `sb_head.html` or your custom html script,  a `video.txt` file containing a link to the demo video, a pdf file named `lab_4.pdf` containing answers to the questions, and a `instructions.txt` file containing instructions on how to run your code if it other than running `python3 lab_4.py` (e.g. if you adapted your code to accept inputs from the command line). Please upload the files in a single zip file named **<your UIN>.zip**.
