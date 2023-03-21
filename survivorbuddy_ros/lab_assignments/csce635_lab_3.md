# Programming Lab 3: Gesture Recognition

## Learning Objectives

- Familiarity with the `OpenCV`/`MediaPipe` gesture recognition library.
- Creating a simple publisher and subscriber in `ROS`.
- Familiarity with `roslibjs` and the `YouTube` Player API.

## Things to bring along to the lab

- Laptop (with Ubuntu 20.04 installed)
- USB-A to USB-C adapter if you only have USB-C ports on your laptop.

## Introduction

Consider a trapped survivor who can't reach the phone's touch interface, but wants to interact with the streaming video ([Tabor the Great](https://youtu.be/sBHCuRNy3Io)).

The assignment consists of detecting hand gestures for:
1. Stop
2. Pause
3. Play/resume
4. Raise volume
5. Lower volume

## Extra credit (up to 25%)

- SB indicates when it is confused which would trigger when it cannot interpret the gesture.
- Train a new hand gesture not part of the default gestures in the `OpenCV`/`MediaPipe` libraries to indicate rewind 10 seconds or fast forward 10 seconds. You may using any machine learning algorithm to do so.

### Things to keep in mind for the programmer

- **Rapid reflexes**: The reaction needs to be fast, not 3 seconds after you clap.
- **Not breaking the motors or linkages by overextending**: Does your program expect the robot to start from the same location (aka a neutral position) every time? Or does it handle reacting within safe operational range of the links from any starting location?

## Conceptual and implementation questions

- How robust is the algorithm? How sensitive is it to the distance between the hand and the camera and the angle of the camera?
- Can you "break" the CV algorithm by shining a flashlight on hands? How sensitive is it to occlusions?
- Thought questions: How would a survivor know what gestures to use? Can the robot train the person?

## Procedure

### Step 1: Enable sensors

Same instructions as lab 2.

### Step 2: Create ROS publisher and subscriber

Create a `ROS` publisher that publishes a message (the message type is up to you to decide) that has information on the recognized gesture. Create a `ROS` subscriber in the web app that subscribes to the publisher you have created. Feel free to make changes to `sb_head.html` and add the subscriber to it. Please take a look at `roslibjs` for interacting with `ROS` from the browser. You may find [these tutorials](http://wiki.ros.org/roslibjs) helpful.

### Step 3: Embed YouTube video

Take a look at the [`YouTube` Player API](https://developers.google.com/youtube/iframe_api_reference) that can be used to embed a `YouTube` video player on a webpage and control the player using `JavaScript`. 
You may embed any `YouTube` video in `sb_head.html` or create a separate page for it. Using the API's JavaScript functions, you can play, pause, or stop a video; adjust the player volume; or retrieve information about the video being played.

### Step 4: Test on the phone attached to SB

Test your implementation on the Pixel phone attached to `SB`. Record a video of the you performing all the gestures you've implemented and the YouTube video being controlled accordingly.

## Grading

In order for your submission to be eligible for grading

- A PDF file containing answers to the conceptual and implementation questions.
- Record a video (with audio) of your code working on SB and upload it to Google drive/YouTube/Vimeo etc.

## Submission

To submit the assignment, please upload the following files to **Canvas -> Assignments -> Programming lab 2**: your script named `lab_3.py`, the html file(s) e.g., `sb_head.html` or your custom html script,  a `video.txt` file containing a link to the demo video, a pdf file named `lab_3.pdf` containing answers to the questions, and a `instructions.txt` file containing instructions on how to run your code if it other than running `python3 lab_3.py` (e.g. if you adapted your code to accept inputs from the command line). Please upload the files in a single zip file named **<your UIN>.zip**.
