# Programming Lab 2: Behaviors and IRM

## Learning Objectives

- Building behaviors from other behaviors or perceptual/motor schemas.
- Implicit coordination with schemas with IRM.
- Familiarity with the `OpenCV` library.
- Have fun thinking about `Petit Mal` robot art installation.

## Things to bring along to the lab

- Laptop (with Ubuntu 20.04 installed)
- USB-A to USB-C adapter if you only have USB-C ports on your laptop.

## Introduction

This assignment consists of 2 parts:

1. Programming two independent behaviors; `Engage` and `Cower`, test it in `MoveIt`, and then on the `Survivor Buddy (SB)`.

2. Answering questions related to the concepts used in the assignment and on your implementation.

## Engage behavior

### Attention-seeking

Hear a loud noise/clap and make a movement/noise to attract attention. The programmer may design any behavior (including generating audio) that they seem appropriate to attract attention.

### Mirroring

If a human face is detected, orient to it. If the human face moves close slowly, the robot moves close slowly as well (attraction). If the person moves head to left, `SB` moves its `face` the same amount to its right (mirroring). The behavior should try to match the speed of the human face movement.

> _Note:_ SB's `face` refers to the phone mounted on it. 

### Confused

If the human face goes out of view, then it acts confused. The programmer may design any behavior that seems appropriate for being `confused`.

> _Note:_ If no human face is detected for more than `N` seconds (a parameter), start over.

## Cower behavior

If a human face is too close, **OR** if the approach is too fast, withdraw (repulsion) and protect its `face` (cower).

## Extra credit (up to 25%)

- Display changing patterns on the Android "face" to attract attention.
- Display eyes on the Android "face" that add to the behaviors. Note that just adding eyes that don't blink or widen or other way of dynamically change does **not** count.
- Dancing, singing, or something clever to attract attention once it detects a loud noise.
- SB calling out appropriate things such as "over here!", "nooooo" "come back here".

> _Note:_ For inspiration, check out Simon Penny's `Petit Mal` robot art installation back in 1996 when he had to use ultrasonics instead of cameras [here](https://simonpenny.net/works/petitmal.html).

### Things to keep in mind for the programmer

- **Rapid reflexes**: The reaction needs to be fast, not 3 seconds after you clap.
- **Not breaking the motors or linkages by overextending**: Does your program expect the robot to start from the same location (aka a neutral position) every time? Or does it handle reacting within safe operational range of the links from any starting location?
- **Parameters**: How does the program let the sensitivity of volume of the noise and closeness to SB's face to be changed? Is it hardcoded, a command line parameter, or dynamic?


## Conceptual and implementation questions

- What is the difference between implicit and explicit coordination?

- What were the releasers, perceptual schemas, motor schemas, and
behavioral schemas?

- How did SB produce a reasonable emergent behavior from implicit
coordination?

- Draw the actigrams for the behaviors.

- What behaviors do you think Simon Penny used in `Petit Mal`? 


## Procedure

### Step 1: Enable sensors

In order to detect a clap/loud noise, you will need to use an audio sensor (mic) for SB to detect a clap/loud noise. The set up instructions are the same as in Programming lab 1 which you may refer to [here](https://github.com/yashas-salankimatt/sb_master_repo/blob/master/survivorbuddy_ros/lab_assignments/csce635_lab_1.md). The audio stream gets published on the `/audio` ROS topic.

The camera sensor on your phone will also get activated once you finish the setup for the mic. The image stream is published on the `/camera/image/compressed` ROS topic.


### Step 2: Execute behavior in MoveIt

Use `OpenCV` for face detection and keypoint tracking.

To start `MoveIt`, follow the same instructions from lab 1.

Once you have programmed your desired behavior, to test your code on `MoveIt`, run

```python
python3 lab_2.py
```

and visualize the results on `RViz`.


### Step 3: Test on SB

Follow the instructions from lab 1 to connect to `SB`. Once you confirm that the interfacing has been successful, run your code

```python
python3 lab_2.py
```

and record a video of `SB` performing the "Engage" and "Cower" behaviors.

## Grading

In order for your submission to be eligible for grading

- Show that your code works in `MoveIt`. Show this demo to the TA **before** testing it on SB.
- A PDF file containing answers to the conceptual and implementation questions.
- Record a video (with audio) of your code working on SB and upload it to Google drive/YouTube/Vimeo etc. The video should contain the following things:
1. Clapping or a making a loud noise and the robot reacting to attract attention.
2. Engage behaviors.
4. Cower behavior.

## Submission

To submit the assignment, please upload the following files to **Canvas -> Assignments -> Programming lab 2**: your script named `lab_2.py`, a `video.txt` file containing a link to the demo video, a pdf file named `lab_2.pdf` containing answers to the questions, and a `instructions.txt` file containing instructions on how to run your code if it other than running `python3 lab_2.py` (e.g. if you adapted your code to accept inputs from the command line). Please upload the files in a single zip file named **<your UIN>.zip**.
