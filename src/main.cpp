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
	bool visualizeNIS = true;

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

	if(visualizeNIS){
		std::vector<Car> cars = highway.getTraffic();
		float lidarThresh = 5.991;
		float radarThresh = 7.815;
		float NISLength = cars[0].ukf.NIS_lidar_.size();

		std::vector<float> lidarNISThresh(NISLength, lidarThresh), radarNISThresh(NISLength, radarThresh);
		Eigen::MatrixXi aboveThresholdCounts = Eigen::MatrixXi(cars.size(),2);
		aboveThresholdCounts.fill(0);

		matplotlibcpp::figure_size(1600,900);
		matplotlibcpp::suptitle("NIS values over frames for each car and sensor");

		for(int i=0; i<cars.size(); i++){

			matplotlibcpp::subplot(2,3,i+1);
			matplotlibcpp::plot(cars[i].ukf.NIS_lidar_);
			matplotlibcpp::plot(lidarNISThresh,"r--");
			matplotlibcpp::xlabel("frames");
			matplotlibcpp::ylabel("NIS");
			matplotlibcpp::grid(1);
			matplotlibcpp::subplot(2,3,i+3+1);
			matplotlibcpp::plot(cars[i].ukf.NIS_radar_);
			matplotlibcpp::plot(radarNISThresh,"r--");
			matplotlibcpp::xlabel("frames");
			matplotlibcpp::ylabel("NIS");
			matplotlibcpp::grid(1);
			for(int j=0; j<NISLength; j++){
				if(cars[i].ukf.NIS_lidar_[j]>lidarThresh){
					aboveThresholdCounts(i,0)++;
				}
				if(cars[i].ukf.NIS_radar_[j]>radarThresh){
					aboveThresholdCounts(i,1)++;
				}
			}
		}

		//std::cout << aboveThresholdCounts << std::endl;
		Eigen::MatrixXd belowThreshPercentage = Eigen::MatrixXd(cars.size(),2);
		for(int i=0; i<cars.size(); i++){
			belowThreshPercentage(i,0) = (NISLength-aboveThresholdCounts(i,0))/NISLength * 100;
			belowThreshPercentage(i,1) = (NISLength-aboveThresholdCounts(i,1))/NISLength * 100;
			matplotlibcpp::subplot(2,3,i+1);
			matplotlibcpp::title("NIS Lidar Car "+std::to_string(i+1)+",  "+std::to_string(belowThreshPercentage(i,0))+"% below threshold");
			matplotlibcpp::subplot(2,3,i+3+1);
			matplotlibcpp::title("NIS Radar Car "+std::to_string(i+1)+",  "+std::to_string(belowThreshPercentage(i,1))+"% below threshold");
		}

		matplotlibcpp::show();
	}


}