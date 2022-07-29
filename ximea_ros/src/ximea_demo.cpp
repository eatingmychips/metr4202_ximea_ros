#include <iostream>
#include <ximea_cv/ximea_cv.hpp>

using namespace cv;
using namespace std;

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
