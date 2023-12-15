#include <aruco/aruco.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace std; 

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: inimage" << std::endl;
        return -1;
    }

   
cv::Mat im=cv::imread(argv[1]);
aruco::CameraParameters camera;
camera.readFromXMLFile(argv[2]);
aruco::MarkerMap mmap;
mmap.readFromFile(argv[3]);
aruco::MarkerMapPoseTracker MMTracker;
MMTracker.setParams(camera,mmap);
aruco::MarkerDetector Detector;
Detector.setDictionary("ARUCO_MIP_36h12");
auto markers=Detector.detect(im);//0.05 is the marker size
MMTracker.estimatePose(markers);
if (MMTracker.isValid())
std::cout<<MMTracker.getRvec()<<" "<<MMTracker.getTvec()<<std::endl;
cv::imshow("image",im);
cv::waitKey(0);
}
