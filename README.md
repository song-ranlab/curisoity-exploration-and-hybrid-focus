# Curiosity-Exporation-and-Hybrid-Focus
This repository contains all the code created and outside sources used for my thesis.

The goal of these programs is to create a point of focus for the robot to use as a point of interest to explore.  To do this a human like focus is emulated using topic modeling and object recognition.  The main driving factor for exploration is taken as curiosity, as defined by Girdhar in his previous works.

For additional details and information on these programs please consult my thesis "Curiosity Driven Exploration with Focused Semantic Mapping" by Curran Meek.

# Pre-Requisites

To utilize this code two existing programs are used.  You will need to install:

ROST: Follow the directions provided at https://gitlab.com/warplab/rost-cli

YOLO: Follow the directions at https://pjreddie.com/darknet/yolo/

The main sensor for this work is a RGB camera.  A resolution of 640x480 was used for all processes.  This resolution ensures the perplexity data is segmented in a 15x20 grid over each image.

(Note: OpenCV 3 or above will also need to be installed, as it is utilized for both ROST and the code created here)

# Modified_ROST

This folder contains the code used to modify the sunshine program from ROST.  The exiting sunshine.cpp and broadcaster.hpp will need to be replced with in order to utilize the ROST Interpreter code.  This modification changes how the perplexity data is output to a .json file.  This file is used by the ROST interpeter to access the perplexity data.

# ROS_HybridFocus

This folder contains all the nodes used for implamentation in ROS.  These nodes contain the ROST Interpreter program (rost_interp), the semantic mapping program (obj_markers), and others for implamentation.  Each description next to the folder give a breif explination of the code.  Other progams were used and are referneced in the readme inside the folder. 

*The curious_obj_routine node is not finished but supplied for reference and future work.

Each node source code was created with Python 2.7, each node was used in ROS Melodic and or ROS Kinetic.  These nodes will need to be added/installed to the ROS directory using normal catkin_make commands.  

# HybridFocus_for_Video

This is a standalone program used to process video, or image data.  As described in the Thesis, the program combines data from ROST Sunshine and YOLO object detection preformed seperately.  Once each frame has perplexity data saved, along with any detected objects (with bounding box data) they are then combined to determine the focus point and overlayed onto the image.

To segment the video ffmpeg commands were used to generate an image for each frame.  The program outputs a series of images that can be combined into a video using ffmpeg commands.

In order to properly use this code, first run ROST Sunshine on the video.  This should create a .json file that contains perplexity data for each image, where each line is a new frame.  Ensure these file locations are updated accordingly in the main portion of combinedFocus.py.  Ensure an average perplexity is obtained/updated prior to running, in addition to the number of frames used in the for loop in the main poritoin of the code.  The combinedFocus.py can then be run.  It will output images with drawn overlays for the focus, perplexity, and detected object locations.  The focus region is updated every second to provide more stability, this can be modified in the program.  

The stingray folder gives YOLO configuration data used, while the readme detials the tools used for training the YOLO model.  In addtion, the folder contains the perplexity data generated when the result.mp4 was created.  The orgVideo folder containes the original video used for testing.
