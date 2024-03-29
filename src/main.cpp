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
		float NISLength = max(cars[0].ukf.NIS_lidar_.size(),cars[0].ukf.NIS_radar_.size());
		bool usedRadar = cars[0].ukf.use_radar_;
		bool usedLidar = cars[0].ukf.use_laser_;
		uint8_t numOfSensors;
		std::vector<float> lidarNISThresh(NISLength, lidarThresh);
		std::vector<float> radarNISThresh(cars[0].ukf.NIS_radar_.size(), radarThresh);
		if(usedLidar && usedRadar){
			numOfSensors = 2;
		}
		else{
			numOfSensors = 1;
		}

		Eigen::MatrixXi aboveThresholdCounts = Eigen::MatrixXi(cars.size(),numOfSensors);
		aboveThresholdCounts.fill(0);

		matplotlibcpp::figure_size(1600,900);
		matplotlibcpp::suptitle("NIS values over frames for each car and sensor");

		for(int i=0; i<cars.size(); i++){
			if(usedLidar && usedRadar){
				matplotlibcpp::subplot(numOfSensors,cars.size(),i+1);
				matplotlibcpp::plot(cars[i].ukf.NIS_lidar_);
				matplotlibcpp::plot(lidarNISThresh,"r--");
				matplotlibcpp::xlabel("frames");
				matplotlibcpp::ylabel("NIS");
				matplotlibcpp::grid(1);

				matplotlibcpp::subplot(numOfSensors,cars.size(),i+cars.size()+1);
				matplotlibcpp::plot(cars[i].ukf.NIS_radar_);
				matplotlibcpp::plot(radarNISThresh,"r--");
				matplotlibcpp::xlabel("frames");
				matplotlibcpp::ylabel("NIS");
				matplotlibcpp::grid(1);
			}
			else{
				if(usedLidar){
					matplotlibcpp::subplot(numOfSensors,cars.size(),i+1);
					matplotlibcpp::plot(cars[i].ukf.NIS_lidar_);
					matplotlibcpp::plot(lidarNISThresh,"r--");
					matplotlibcpp::xlabel("frames");
					matplotlibcpp::ylabel("NIS");
					matplotlibcpp::grid(1);
				}
				else if(usedRadar){
					matplotlibcpp::subplot(numOfSensors,cars.size(),i+1);
					matplotlibcpp::plot(cars[i].ukf.NIS_radar_);
					matplotlibcpp::plot(radarNISThresh,"r--");
					matplotlibcpp::xlabel("frames");
					matplotlibcpp::ylabel("NIS");
					matplotlibcpp::grid(1);
				}
			}


			for(int j=0; j<NISLength; j++){
				if(usedLidar && usedRadar){
					if(cars[i].ukf.NIS_lidar_[j]>lidarThresh){
						aboveThresholdCounts(i,0)++;
					}
					if(cars[i].ukf.NIS_radar_[j]>radarThresh){
						aboveThresholdCounts(i,1)++;
					}
				}
				else{
					if(usedLidar){
						if(cars[i].ukf.NIS_lidar_[j]>lidarThresh){
							aboveThresholdCounts(i,0)++;
						}
					}
					else if(usedRadar){
						if(cars[i].ukf.NIS_radar_[j]>radarThresh){
							aboveThresholdCounts(i,0)++;
						}
					}
				}
			}
		}

		//std::cout << aboveThresholdCounts << std::endl;
		Eigen::MatrixXd belowThreshPercentage = Eigen::MatrixXd(cars.size(),numOfSensors);
		for(int i=0; i<cars.size(); i++){
			if(usedRadar && usedLidar){
				belowThreshPercentage(i,0) = (NISLength-aboveThresholdCounts(i,0))/NISLength * 100;
				belowThreshPercentage(i,1) = (NISLength-aboveThresholdCounts(i,1))/NISLength * 100;
				matplotlibcpp::subplot(numOfSensors,cars.size(),i+1);
				matplotlibcpp::title("NIS Lidar Car "+std::to_string(i+1)+",  "+std::to_string(belowThreshPercentage(i,0))+"% below threshold");
				matplotlibcpp::subplot(numOfSensors,cars.size(),i+cars.size()+1);
				matplotlibcpp::title("NIS Radar Car "+std::to_string(i+1)+",  "+std::to_string(belowThreshPercentage(i,1))+"% below threshold");
			}
			else{
				if(usedLidar){
					belowThreshPercentage(i,0) = (NISLength-aboveThresholdCounts(i,0))/NISLength * 100;
					matplotlibcpp::subplot(numOfSensors,cars.size(),i+1);
					matplotlibcpp::title("NIS Lidar Car "+std::to_string(i+1)+",  "+std::to_string(belowThreshPercentage(i,0))+"% below threshold");
				}
				else if(usedRadar){
					belowThreshPercentage(i,0) = (NISLength-aboveThresholdCounts(i,0))/NISLength * 100;
					matplotlibcpp::subplot(numOfSensors,cars.size(),i+1);
					matplotlibcpp::title("NIS Radar Car "+std::to_string(i+1)+",  "+std::to_string(belowThreshPercentage(i,0))+"% below threshold");
				}
			}
		}

		matplotlibcpp::show();
	}


}