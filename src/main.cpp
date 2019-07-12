/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "render/render.h"
#include "highway.h"
#include "matplotlib-cpp/matplotlibcpp.h"

int main(int argc, char** argv)
{

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// set camera position and angle
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);

	Highway highway(viewer);

	//initHighway(viewer);

	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	double egoVelocity = 25;

	while (frame_count < (frame_per_sec*sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		//stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		highway.stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		viewer->spinOnce(1000/frame_per_sec);
		frame_count++;
		time_us = 1000000*frame_count/frame_per_sec;
		
	}
	std::vector<Car> cars = highway.getTraffic();

	std::vector<float> lidarNISThresh(cars[0].ukf.NIS_lidar_->size(),5.991), radarNISThresh(cars[0].ukf.NIS_radar_->size(),7.815);

	for(int i=0; i<cars.size(); i++){

		matplotlibcpp::subplot(2,3,i+1);
		matplotlibcpp::plot(*cars[i].ukf.NIS_lidar_);
		matplotlibcpp::plot(lidarNISThresh,"r--");
		matplotlibcpp::title("NIS Lidar Car "+std::to_string(i+1));
		matplotlibcpp::grid(1);
		matplotlibcpp::subplot(2,3,i+3+1);
		matplotlibcpp::plot(*cars[i].ukf.NIS_radar_);
		matplotlibcpp::plot(radarNISThresh,"r--");
		matplotlibcpp::title("NIS Radar Car "+std::to_string(i+1));
		matplotlibcpp::grid(1);
	}

    matplotlibcpp::show();

}