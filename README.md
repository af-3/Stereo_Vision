# Stereo_Vision
Depth perception using 2 webcams

Resulting video recording: https://youtu.be/4i7Eow_L1g8

Code Files:

stereo_video_no_trackbar.cpp | Stereo block matching parameters set manually prior to running the program

stereo_video_trackbar.cpp | Stereo block matching parameters can be adjusted in real time with added trackbars

chessboard_captures.cpp | Captures an image from left and right camera upon pressing the ‘a’ key, used to collect image pairs for calibration

stereo_calib.cpp | Not included in repo as this is unmodified sample code from OpenCV, takes in a set of chessboard image pairs and outputs the intrinsic and extrinsic parameters (these become inputs into the stereo_video files).
