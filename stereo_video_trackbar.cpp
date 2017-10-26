#include <iostream>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

const char *windowDisparity = "Disparity";

int main( int argc, char** argv )
{
    //STEREORECTIFY DATA
    
    string intrinsic_filename = "intrinsics.yml";
    string extrinsic_filename = "extrinsics.yml";
    
    FileStorage fs(intrinsic_filename, FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", intrinsic_filename.c_str());
        return -1;
    }
    
    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;
    
    fs.open(extrinsic_filename, FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", extrinsic_filename.c_str());
        return -1;
    }
    
    Mat R, T, R1, P1, R2, P2, Q;
    fs["R"] >> R;
    fs["T"] >> T;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;
    
    /************************Added for trackbar******************/
    int ndisparities = 208;   /**< Range of disparity */
    int SADWindowSize = 9; /**< Size of the block window. Must be odd */
    int preFilterSize = 5, minDisparity = -7, textureThresh = 699, uniqueRatio = 0, speckWind = 0, speckRange = 100, preFillCap = 63, disp12Diff = 100;
    namedWindow("control", WINDOW_NORMAL);
    createTrackbar("WindowSize", "control", &SADWindowSize, 255, NULL);
    createTrackbar("NumOfDisparities", "control", &ndisparities, 1000, NULL);
    createTrackbar("MinDisparity", "control", &minDisparity, 500, NULL);
    createTrackbar("PreFilSize", "control", &preFilterSize, 100, NULL);
    createTrackbar("Texture", "control", &textureThresh, 2000, NULL);
    createTrackbar("UniquenessRatio", "control", &uniqueRatio, 50, NULL);
    createTrackbar("SpeckleWindow", "control", &speckWind, 5000, NULL);
    createTrackbar("SpeckleRange", "control", &speckRange, 100, NULL);
    createTrackbar("FilterCap", "control", &preFillCap, 100, NULL);
    createTrackbar("setDisp12MaxDiff", "control", &disp12Diff, 100, NULL);
    /***********************Added for trackbar END*****************/
    
    
    VideoCapture capturel(1);
    VideoCapture capturer(0);

    Mat imgLeftIN, imgRightIN, imgLeft, imgRight, disp_out;
    
    //-- Call the constructor for StereoBM

    Ptr<StereoBM> bm = StereoBM::create(ndisparities, SADWindowSize);

    
    while (1) {
        
        /************************Added for trackbar******************/
        int i1,i2,i3,i4,i5,i6,i7,i8,i9, i10,temp1,temp2,temp3,temp4,temp5,temp6,temp7,temp8,temp9,temp10;

        i1 = SADWindowSize;
        temp1 = i1 - 1;
        bm->setBlockSize(temp1);
        if (i1<=5)
        {
            temp1 = 5;
            bm->setBlockSize(temp1);
        }
        if (i1 % 2 != 0 && i1>5)
        {
            temp1 = i1;
            bm->setBlockSize(temp1);
        }
        i2 = ndisparities;
        if (i2 % 16 != 0 && i2>16)
        {
            temp2 = i2 - i2 % 16;
            bm->setNumDisparities(temp2);
        }
        if (i2 % 16 == 0 && i2>16)
        {
            temp2 = i2;
            bm->setNumDisparities(temp2);
        }
        if (i2 <= 16)
        {
            temp2 = 16;
            bm->setNumDisparities(temp2);

        }
        i3 = preFilterSize;
        if (i3 % 2 == 0 && i3 >= 7)
        {
            temp3 = i3 - 1;
            bm->setPreFilterSize(temp3);
        }
        if (i3<7)
        {
            temp3 = 7;
            bm->setPreFilterSize(temp3);

        }
        if (i3 % 2 != 0 && i3 >= 7)
        {
            temp3 = i3;
            bm->setPreFilterSize(temp3);
        }

        i4 = preFillCap;
        if (i4>0)
        {
            temp4 = i4;
            bm->setPreFilterCap(temp4);

        }
        if (i4 == 0)
        {
            temp4 = 1;
            bm->setPreFilterCap(temp4);
        }

        i5 = minDisparity;
        temp5 = -i5;
        bm->setMinDisparity(temp5);

        i6 = textureThresh;
        temp6 = i6;
        bm->setTextureThreshold(temp6);

        i7 = uniqueRatio;
        temp7 = i7;
        bm->setUniquenessRatio(temp7);

        i8 = disp12Diff;
        temp8 = 0.01*((float)i8);
        bm->setDisp12MaxDiff(temp8);

        i9 = speckWind;
        temp9 = i9;
        bm->setSpeckleWindowSize(temp9);

        i10 = speckRange;
        temp10 = i10;
        bm->setSpeckleRange(speckRange);
        /***********************Added for trackbar END*****************/
        
        capturel >> imgLeft;
        capturer >> imgRight;
        
        if(imgLeft.empty() | imgRight.empty()){
            cout << "Failed to capture an image" << endl;
            return -1;
        }
        
        //SHRINK AND FILTER INPUT IMAGE
        
        pyrDown(imgLeft, imgLeft);
        pyrDown(imgRight, imgRight);
        
//        bilateralFilter(imgLeftIN, imgLeft, 15, 150, 150);
//        bilateralFilter(imgRightIN, imgRight, 15, 150, 150);
        
        //DISP MAT AND CONVERT TO GRAYSCALE
        cvtColor(imgLeft, imgLeft, CV_BGR2GRAY);
        cvtColor(imgRight, imgRight, CV_BGR2GRAY);
        Mat imgDisparity16S= Mat( imgLeft.rows, imgLeft.cols, CV_16U );
        Mat imgDisparity8U = Mat( imgLeft.rows, imgLeft.cols, CV_8SC1 );
        
        //-- REMAP
        Size img_size = imgLeft.size();
        
        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
        
        Mat img1r, img2r, img1r_small, img2r_small;
        remap(imgLeft, img1r, map11, map12, INTER_LINEAR);
        remap(imgRight, img2r, map21, map22, INTER_LINEAR);
        
        //SHOW RECTIFIED
        pyrDown(img1r, img1r_small);
        pyrDown(img2r, img2r_small);
        imshow("rect left", img1r_small);
        imshow("rect right", img2r_small);
        
        //COMPUTE DISPARITY
        bm->compute( img1r, img2r, imgDisparity16S );
        
        //-- Check its extreme values
        double minVal; double maxVal;
        minMaxLoc( imgDisparity16S, &minVal, &maxVal );
        //        printf("Min disp: %f Max value: %f \n", minVal, maxVal);
        
        //-- Display it as a CV_8UC1 image
        
          imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
        
        //DISTANCE FROM DISPARITY
//        Mat xyz;
//        reprojectImageTo3D(imgDisparity16S, xyz, Q,false,-1);
        
        // keep only z values
//        vector<Mat> channels(3);
//        split(xyz, channels);
//        disp_out = channels[2];
//        disp_out.convertTo(disp_out, CV_8UC1);
//        applyColorMap(disp_out, disp_out, 2);
//        imshow("Depth Z",disp_out);
        
        applyColorMap(imgDisparity8U, imgDisparity8U, 2);
        namedWindow( windowDisparity, WINDOW_NORMAL );
        imshow( windowDisparity, imgDisparity8U );
        
        char c = (char)waitKey(1);
        if (c == 27){
            break;
        }
        if (c == 'a'){
            imwrite("imgLeft.png", img1r);
            imwrite("imgRight.png", img2r);
            cout << "image captured" << endl;
        }

    }
    
    destroyAllWindows();
    return 0;
}
