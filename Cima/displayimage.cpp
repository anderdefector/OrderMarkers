#include <stdio.h>
#include "marker/reconoce.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "homography/homografia.h"
//Para obtener los puntos y el marcador
#include <vector>
#include <tuple>


using namespace cv;
using namespace std;
typedef tuple <int, float, float, float, float, float, float, float, float, float, float, float, float > carac;
vector <carac>  puntosMarcador;

//marker recognizer
bool marker_found;
float points[12];
float Model[12] = {255, 0, 0, 0, 145, 18, 92, 54, 0, 255, 255, 255};
int v[4]; // para marker_recognition, search_marker_in_frame
int state; //Se agrega para el seguimiento
int marker_box[4]; //Se agrega para el seguimiento
Mat Image2; //variable para el reconoce
Mat Image;
uint8_t u;
uint8_t u1;
int num_comp[2]; //number for compare
int num_mark; //número de marador

float MatrixK[16];
float MatrixRt[16];

int bandera;

double NormalModel1[6][2] = {{-1.18225657, -0.8571661},
                             {1.23923279, -0.8571661 }, 
                             {0.19466875, -0.69810435},
                             {-0.30862119, -0.37998085},
                             {1.23923279,  1.39620869},
                             {-1.18225657,  1.39620869}};

double vdP1[4] = {4.88235294, 4.12968983, 3.80392157, 4.43778817};

double NormalModel2[6][2] = {{ 1.19908755,  0.94738732},
                             { -1.19908755,  0.94738732},
                             { -0.35267281,  0.43561539},
                             { 0.35267281,  0.43561539},
                             { -1.19908755, -1.38300271},
                             {1.19908755, -1.38300271}};

double vdP2[4] = {5, 4.16983732, 5.93464052, 4.29112718};


double NormalModel2inv[6][2] = {{-1.19908755, -0.94738732},
                             {  1.19908755, -0.94738732},
                             { 0.35267281, -0.43561539},
                             {-0.35267281, -0.43561539},                             
                             {1.19908755,  1.38300271},
                             {-1.19908755,  1.38300271}};                           

double vdP2inv[4] = {5, 4.16983732, 4.06535948, 4.29112718};


double NormalModel3[6][2] = {{ 1.34510517,  1.02010864},
                             { -1.05071661,  1.02010864},
                             { -0.29908625,  0.7104328},
                             { -0.28969087, -0.14572981},
                             { -1.05071661, -1.30246013},
                             { 1.34510517, -1.30246013}};

double vdP3[4] = {4.38562092, 4.17393317, 5.60784314, 4.30557757};


double NormalModel4[6][2] = {{ -1.04469358,  1.36605263},
                             {-1.04469358, -0.93324388},
                             {-0.63970224, -0.15779486},
                             {0.12425871, -0.70782265},
                             {1.30241535, -0.93324388},
                             {1.30241535,  1.36605263}};

double vdP4[4] = {4.45098039, 4.26056068, 4.05882353, 4.34915634};



string s;


int numero_marcador(int num_comp[2]){
  int num_mark;
  if(num_comp[0] == 3){
    if (num_comp[1] == 1) {
          num_mark = 1;
    }
    if (num_comp[1] == 2) {
          num_mark = 2;
    }
  }else if( num_comp[0] == 2 ){
    if (num_comp[1] == 2) {
          num_mark = 3;
    }
    if (num_comp[1] == 3) {
          num_mark = 4;
    }
  }else{
      num_mark = 5;
  }
  return num_mark;

}

void parametrizar_M( float M[16]){
    double x[6] = {0};
    /**
        0 1 2       3   
        4 5 6       7
        8 9 10      11
        12 13 14    15
    
        x[0] = psi
        x[1] = theta
        x[2] = phi
        x[3] = tx
        x[4] = ty
        x[5] = tz
    **/
    printf("M10 = %f \n", M[10]);
    x[0] = atan2( M[6], M[2]) *  (180.0/3.1416);
    x[1] = acos(M[10]) * (180.0/3.1416);
    x[2] = atan2( M[9] ,-M[8] ) *  (180.0/3.1416);
    x[3] = M[3];
    x[4] = M[7];
    x[5] = M[11];

    printf("%f %f %f %f %f %f \n", x[0], x[1], x[2], x[3], x[4], x[5] );
}


bool search_marker_in_frame(Mat& IMG1, Mat& IMG, uint8_t um) {
  GaussianBlur(IMG, Image, Size(7, 7), 0, 0);  // Smooth filter
  threshold(Image, Image, um, 255, THRESH_BINARY);
  //namedWindow("Original", WINDOW_AUTOSIZE );
  bool marker_found = false;
  uchar *pout;
  int *pin;
  Mat Etiquetas, Estadisticas, Centroides;

  Image = ~Image;

  Image2 = Mat::zeros(480, 640, CV_8UC1);
  /** Los objetos deben ser blancos **/
  int n = connectedComponentsWithStats(Image, Etiquetas, Estadisticas,
                                       Centroides, 8, CV_32S);
  //printf( "n = %d\n", n );

  /** Para todas las componentes conectadas **/
  // for (int i = 1; i < n && !marker_found; i++) { //se lo quité para que
  // busque mas marcadores
  for (int i = 1; i < n; i++) {
  
    // printf("Search in frame\n");
    if (Estadisticas.at<int>(i, CC_STAT_AREA) < 50) continue;

    v[0] = Estadisticas.at<int>(i, CC_STAT_LEFT) - 1;
    if (v[0] < 0) v[0] = 0;
    v[1] = Estadisticas.at<int>(i, CC_STAT_TOP) - 1;
    if (v[1] < 0) v[1] = 0;
    v[2] = v[0] + Estadisticas.at<int>(i, CC_STAT_WIDTH) + 1;
    if (v[2] > Image.cols - 1) v[2] = Image.cols - 1;
    v[3] = v[1] + Estadisticas.at<int>(i, CC_STAT_HEIGHT) + 1;
    if (v[3] > Image.rows - 1) v[3] = Image.rows - 1;

    if (v[0] == 0 || v[1] == 0 || v[2] == Image.cols - 1 ||
        v[3] == Image.rows - 1)
      continue;

    for (int j = v[1]; j <= v[3]; j++) {
      pin = Etiquetas.ptr<int>(j) + v[0];
      pout = Image2.ptr<uchar>(j) + v[0];

      for (int k = v[0]; k <= v[2]; k++) {
        if (*pin == i) {
          *pout = 255;
        }
        pin++;
        pout++;
      }
    }

  

    //Cambiar el 4to parametro por 1 si la imagen ya esta invertida
    
    if (reconoce(Image2, v, points, num_comp, 0) == 0) {
      marker_found = true;
      //printf("Marcador encontrado en frame \n");
      // datos del marcador encontrado
      marker_box[0] = Estadisticas.at<int>(i, CC_STAT_LEFT);    // x
      marker_box[1] = Estadisticas.at<int>(i, CC_STAT_TOP);     // y
      marker_box[2] = Estadisticas.at<int>(i, CC_STAT_WIDTH);   // width
      marker_box[3] = Estadisticas.at<int>(i, CC_STAT_HEIGHT);  // height
      rectangle(IMG1, Point(marker_box[0], marker_box[1]), Point((marker_box[0]+ marker_box[2]), (marker_box[1]+ marker_box[3])) ,Scalar(255, 0, 0), 2, 2);

            // Se identifica cual es el marcador
      num_mark = numero_marcador(num_comp);

      puntosMarcador.push_back(carac(num_mark, points[0], points[1], points[2], points[3], points[4], points[5], points[6], points[7], points[8], points[9], points[10], points[11])  );
      
      s = to_string(num_mark);
      putText(IMG1,s,Point(marker_box[0], marker_box[1]),FONT_HERSHEY_DUPLEX,1,Scalar(0,255,0),2,false);
        // printf("Marcador %d \n", num_mark);
        //FIN reconoce el marcador

    }

    //imshow("Original", IMG1);

    for (int j = v[1]; j <= v[3]; j++) {
      pout = Image2.ptr<uchar>(j) + v[0];
      for (int k = v[0]; k <= v[2]; k++) {
        *pout++ = 0;
      }
    }
  }


  return marker_found;
}

int main(int argc, char** argv )
{
    int deviceID = 0; 
    VideoCapture *cap;
    Mat frame, grayFrame, gray1, gray2;
	
	//Reconocimiento de marcadores
	bool marker_found;
    //float points[12];
	int v[4]; // para marker_recognition, search_marker_in_frame
    int state; //Se agrega para el seguimiento
    int marker_box[4]; //Se agrega para el seguimiento
    Mat Image2; //variable para el reconoce
    Mat Image;
    uint8_t u;
    uint8_t u1;
    //int num_comp[2]; //number for compare
	//int num_mark; //número de marador

	//Variables marcadores
	state = 0;  // Se inicializa el estado para el seguimiento
    u = 110;
    u1 = 155;
    marker_found = false;

   
    Mat image = imread("../img/M1OT/f.jpg", IMREAD_COLOR);
      
  
  
   	cvtColor(image, grayFrame, COLOR_BGR2GRAY);
    //resize( grayFrame, gray1, Size(), 0.5, 0.5, cv::INTER_AREA );
    GaussianBlur(grayFrame, gray1, Size(7, 7), 1.5, 1.5); //Smooth filter
		threshold( grayFrame, gray1, u, 255,  THRESH_BINARY );

		namedWindow("Cámara", WINDOW_AUTOSIZE );
		imshow("Cámara", gray1);
    int k = waitKey(0);  
    marker_found = search_marker_in_frame(image, gray1, u);
    if (marker_found) {
      	state = 1;
    } else {
      	state = 0;
    }

    cout << "Número de marcadores en la imagen : " << puntosMarcador.size()<< endl;

    for(int i=0; i<puntosMarcador.size(); i++){

      points[0] = get<1>(puntosMarcador.at(i));
      points[1] = get<2>(puntosMarcador.at(i));
      points[2] = get<3>(puntosMarcador.at(i));
      points[3] = get<4>(puntosMarcador.at(i));
      points[4] = get<5>(puntosMarcador.at(i));
      points[5] = get<6>(puntosMarcador.at(i));
      points[6] = get<7>(puntosMarcador.at(i));
      points[7] = get<8>(puntosMarcador.at(i));
      points[8] = get<9>(puntosMarcador.at(i));
      points[9] = get<10>(puntosMarcador.at(i));
      points[10] = get<11>(puntosMarcador.at(i));
      points[11] = get<12>(puntosMarcador.at(i));

      switch( get<0>(puntosMarcador.at(i)) ){
        case 1:
          bandera = homografia(NormalModel1, vdP1, points, MatrixK, MatrixRt, (double)image.rows, (double)image.cols);
          printf("Modelo 1 \n");
        break;
        case 2:
        printf("Modelo 2 \n");
        bandera = homografia(NormalModel2, vdP2, points, MatrixK, MatrixRt, (double)image.rows, (double)image.cols);
        
        printf("Modelo 2  invertido \n");
        //bandera = homografia(NormalModel2inv, vdP2inv, points, MatrixK, MatrixRt, (double)image.rows, (double)image.cols);

      break;
      case 3:
        bandera = homografia(NormalModel3, vdP3, points, MatrixK, MatrixRt, (double)image.rows, (double)image.cols);
        printf("Modelo 3 \n");
      break;
      case 4:
        bandera = homografia(NormalModel4, vdP4, points, MatrixK, MatrixRt, (double)image.rows, (double)image.cols);
        printf("Modelo 4 \n");
      break;
      default:
      break;
    }

    parametrizar_M( MatrixRt);

    

    printf(" \nRt: \n");
    for(int i =0; i <16; i++){
      if((i)%4 == 0){
        printf("\n");
      }
      printf("%f ",MatrixRt[i]);
    } 
      

    }

    
    //Se cambia la coordenada Y
    
  
    for(int i=0; i<12; i++){
      if(i%2 == 0){
        printf("\n");
      }
      printf("%f ",points[i]);

    }
    printf("\n");
       
    namedWindow("Cámara", WINDOW_AUTOSIZE );
    imshow("Cámara", image);
    k = waitKey(0);

    
		



    return 0;
}