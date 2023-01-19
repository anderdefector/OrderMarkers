#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/core/core.hpp>
using namespace cv;
using namespace std;

int main(){
	Mat image;

	namedWindow("Display Window");
	VideoCapture cap(0);

	if(!cap.isOpened()){
		cout << "cannot open camera";
	}

	while(true){
		cap >> image;
		imshow("Display windows", image);
		waitKey(25);
	}
	return 0;
}
