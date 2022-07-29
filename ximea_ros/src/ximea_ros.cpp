#include <iostream>
#include <ximea_cv/ximea_cv.hpp>

#include <ximea_cv/ximea_cv.hpp>

using namespace cv;
using namespace std;

XimeaROS::XimeaROS(int argc, char** argv) {
    ros::init(argc, argv, "ximea_camera");
}

/**
 * @brief Initialise Camera Publishers
 * 
 * @param serials List of serial numbers to initialise
 */
void KinectROS::init_camera_pub(std::vector<std::string> serials) {
    // For each device, create node handles and publishers.
    for (std::string serial : serials) {

        nh_camera_info_colors[serial]   = std::make_unique<ros::NodeHandle>();
        nh_camera_info_irs[serial]      = std::make_unique<ros::NodeHandle>();

        nh_camera_colors[serial]        = std::make_unique<ros::NodeHandle>(std::string("color_") + serial);
        nh_camera_irs[serial]           = std::make_unique<ros::NodeHandle>(std::string("ir_") + serial);

        pub_camera_info_colors[serial]  = nh_camera_info_colors[serial]->advertise<sensor_msgs::CameraInfo>(std::string("color_") + serial + std::string("/camera_info"), 1);
        pub_camera_info_irs[serial]     = nh_camera_info_irs[serial]->advertise<sensor_msgs::CameraInfo>(std::string("ir_") + serial + std::string("/camera_info"), 1);

        // For each device, create a camera info manager using the default directory for calibration data.
        manager_colors[serial]          = std::make_unique<camera_info_manager::CameraInfoManager>(*nh_camera_colors[serial], std::string("color_") + serial);
        manager_irs[serial]             = std::make_unique<camera_info_manager::CameraInfoManager>(*nh_camera_irs[serial], std::string("ir_") + serial);

        if (!manager_colors[serial]->loadCameraInfo("")) {
            std::cout << "Failed to get calibration for color camera: " << serial << std::endl;
        }

        if (!manager_irs[serial]->loadCameraInfo("")) {
            std::cout << "Failed to get calibration from ir camera: " << serial << std::endl;
        }

        info_colors[serial] = manager_colors[serial]->getCameraInfo();
        info_irs[serial]    = manager_irs[serial]->getCameraInfo();
    }
}


int main(int argc, char** argv) {
	try {
		// Sample for XIMEA OpenCV
		xiAPIplusCameraOcv cam;

		// Retrieving a handle to the camera device
		cout << "Opening first camera..." << endl;
		cam.OpenFirst();
		
		// Set exposure
		cam.SetExposureTime(5000);
		cam.SetGain(12.0);
		cam.SetDownsampling((XI_DOWNSAMPLING_VALUE)2);
		cam.SetImageDataFormat(XI_MONO8);
		
		cout << "Starting acquisition..." << endl;
		cam.StartAcquisition();
		
		while (true) {
			Mat cv_mat_image = cam.GetNextImageOcvMat();
    		imshow("Image from camera", cv_mat_image);
			waitKey(1);
		}
		
		cam.StopAcquisition();
		cam.Close();
		cout << "Done" << endl;
		waitKey(1000);
	} catch(xiAPIplus_Exception& exp) {

		cout << "Error:" << endl;
		exp.PrintError();
		waitKey(3000);
		return -1;
	}
	return 0;
}
