#include <iostream>
#include <stdio.h>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include<System.h>
#include<opencv2/opencv.hpp>
#include <BinoCamera.h>
using namespace std;

bool saveImage(std::string name, int imageCnt, cv::Mat mat){
	char path[100];
	sprintf(path, "%s%d.pjm",name.c_str(),imageCnt);
	return cv::imwrite(path, mat);
}
int main(int argc, char **argv) {

	if(argc != 6)
	{
		cerr << endl << "Usage: ./stereo_bino /dev/video* path_to_vocabulary path_to_settings path_to_intrinsics path_to_extrinsics" << endl;
		return 1;
	}
    BinoCameraParameterList paraList;
    paraList.devPath = argv[1];
    paraList.intParameterPath = argv[4];
    paraList.extParameterPath = argv[5];
	cv::Mat left;
	cv::Mat right;
    std::shared_ptr<BinoCamera> camera(new BinoCamera(paraList));
	ORB_SLAM2::System SLAM(argv[2],argv[3],ORB_SLAM2::System::STEREO,true);
	float start_time,t_time;
	start_time = cv::getTickCount()/cv::getTickFrequency();
	for(;;){
		camera->getRectImage(left, right);
		t_time = (cv::getTickCount()/cv::getTickFrequency()) - start_time;
		SLAM.TrackStereo(left,right,t_time);
	}
	SLAM.Shutdown();

	return 0;
}
