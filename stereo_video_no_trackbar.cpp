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
    
    VideoCapture capturel(1);
    VideoCapture capturer(0);
    
    Mat imgLeftIN, imgRightIN, imgLeft, imgRight, disp_out;
    
    //-- 2. Call the constructor for StereoBM
    int ndisparities = 112*2;   /**< Range of disparity */
    int SADWindowSize = 5; /**< Size of the block window. Must be odd */
    
    Ptr<StereoBM> sbm = StereoBM::create( ndisparities, SADWindowSize );
    
    //-- stereoBM Parameters
    //
    sbm->setMinDisparity(0);
    sbm->setTextureThreshold(500);
    sbm->setUniquenessRatio(0);
    sbm->setPreFilterSize(5);
    sbm->setPreFilterCap(61);
    sbm->setSpeckleWindowSize(10);
    sbm->setSpeckleRange(80);
    sbm->setDisp12MaxDiff(20);
    
    while (1) {
        capturel >> imgLeft;
        capturer >> imgRight;
        if(imgLeft.empty() | imgRight.empty()){
            cout << "Failed to capture an image" << endl;
            return -1;
        }
        //SHRINK AND FILTER INPUT IMAGE
        
        pyrDown(imgLeft, imgLeft);
        pyrDown(imgRight, imgRight);
        cout << imgRight.size() << endl;
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
        //        pyrDown(img1r_small, img1r_small);
        //        pyrDown(img2r_small, img2r_small);
        imshow("rect left", img1r_small);
        imshow("rect right", img2r_small);
        
        //COMPUTE DISPARITY
        sbm->compute( img1r, img2r, imgDisparity16S );
        
        //-- Check its extreme values
        double minVal; double maxVal;
        minMaxLoc( imgDisparity16S, &minVal, &maxVal );
        //        printf("Min disp: %f Max value: %f \n", minVal, maxVal);
        
        //-- 4. Display it as a CV_8UC1 image
        //        imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
        normalize(imgDisparity16S,imgDisparity8U,0,255,CV_MINMAX,CV_8U);
        Mat xyz;
        reprojectImageTo3D(imgDisparity16S, xyz, Q,false,-1);
        
        // keep only z values
        vector<Mat> channels(3);
        split(xyz, channels);
        disp_out = channels[2];
        disp_out.convertTo(disp_out, CV_8UC1);
        applyColorMap(disp_out, disp_out, 2);
        //        pyrDown(imgDisparity8U, imgDisparity8U);
        applyColorMap(imgDisparity8U, imgDisparity8U, 2);
        namedWindow( windowDisparity, WINDOW_NORMAL );
        imshow("Depth Z",disp_out);
        imshow( windowDisparity, imgDisparity8U );
        
        char c = (char)waitKey(10);
        if (c == 27){
            break;
        }
    }
    
    destroyAllWindows();
    return 0;
}
