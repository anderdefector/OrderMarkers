#include "marker/reconoce.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


#include <string>


#include <vector>
#include <tuple>
#include <stdio.h>
#include <unistd.h>
#include <iostream>

using namespace cv;
using namespace std;

typedef tuple <int, float, float, float, float, float, float, float, float, float, float, float, float, int, int, int > carac;

class procesa_robot{
    public:
        vector <carac> puntosMarcador;
        int numero_marcador(int num_comp[2]);
        bool search_marker_in_frame(Mat& IMG1, Mat& IMG, uint8_t um, int tiempo);
        int obtener_caracteristicas(Mat& IMG1, Mat& IMG, uint8_t um, int tiempo);
};
