/*
 Lab-3 Embedded Vision-II-Spring-2017
 *Alice Fockele
 *Unmeel Mokashi
 -Captures a single frame from camera and write to a file
 */
//Code referenced from:
/* boneCV.cpp
 *
 * Copyright Derek Molloy, School of Electronic Engineering, Dublin City University
 * www.derekmolloy.ie
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that source code redistributions retain this notice.
 *
 * This software is provided AS IS and it comes with no warranties of any type.
 */
#include<iostream>
#include<opencv2/opencv.hpp>
#include<unistd.h>
using namespace std;
using namespace cv;

int main()
{
    VideoCapture capture(0);
    VideoCapture capture1(1);
    char * filenameL = new char[100];
    char * filenameR = new char[100];
//    VideoCapture capture(2);
//    capture.set(CV_CAP_PROP_FRAME_WIDTH,1920);
//    capture.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
    if(!capture.isOpened()){
        cout << "Failed to connect to the camera." << endl;
    }
//    capture1.set(CV_CAP_PROP_FRAME_WIDTH,1920);
//    capture1.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
    if(!capture1.isOpened()){
        cout << "Failed to connect to the camera." << endl;
    }
//    int frames = 15;

    Mat frame0, frame1;
    namedWindow("left",WINDOW_NORMAL);
    namedWindow("right",WINDOW_NORMAL);
    
    int i = 0;
    while(1)
    {
        capture >> frame0;
        capture1 >> frame1;
//        namedWindow("left",WINDOW_NORMAL);
        //        namedWindow("right",WINDOW_NORMAL);
        Mat left_small;
        pyrDown(frame0, left_small);
        pyrDown(left_small, left_small);
        imshow("left",left_small);
        Mat right_small;
        pyrDown(frame1, right_small);
        pyrDown(right_small, right_small);
        imshow("right",right_small);
        //        imshow("right",frame1);

//        cout << "ready" << i << endl;
//        usleep(500000);
        
        char c = (char)waitKey(10);
        if (c == 'a'){
            capture >> frame0;
            capture1 >> frame1;
            if(frame0.empty() | frame1.empty()){
                cout << "Failed to capture an image" << endl;
                return -1;
            }
            sprintf(filenameL, "left%i.jpg", i);
            sprintf(filenameR, "right%i.jpg", i);
            imwrite(filenameL, frame0);
            imwrite(filenameR, frame1);
            //        usleep(2000000);
            cout << "image captured"<< i << endl;
            i+=1;
        }
    }
    return 0;
    
}