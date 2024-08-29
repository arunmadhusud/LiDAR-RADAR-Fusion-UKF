/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "render/render.h"
#include "highway.h"
#include "plot.h"

int main(int argc, char** argv)
{

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

    py::scoped_interpreter guard{}; // Initialize the Python interpreter globally


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

    for (int i = 0; i < highway.traffic.size(); ++i)
    {
        // Get the NIS values for radar and lidar for each car
        const std::vector<double>& NISradar = highway.traffic[i].ukf.NIS_radar_;
        const std::vector<double>& NISlaser = highway.traffic[i].ukf.NIS_laser_;

        // Generate index numbers for the x-axis
        std::vector<int> radar_indices(NISradar.size());
        std::vector<int> lidar_indices(NISlaser.size());

        for (int j = 0; j < radar_indices.size(); ++j)
        {
            radar_indices[j] = j; // Using index as the x-axis for radar
        }
        for (int j = 0; j < lidar_indices.size(); ++j)
        {
            lidar_indices[j] = j; // Using index as the x-axis for lidar
        }

		// Call the plotNIS function for Radar
        plotNIS(NISradar, 7.815 , "Radar for car " + std::to_string(i+1)); // chi-square 3-DOF 95% threshold

        // Call the plotNIS function for Lidar
        plotNIS(NISlaser, 5.991, "Lidar for car " + std::to_string(i+1)); // chi-square 2-DOF 95% threshold
	}

}