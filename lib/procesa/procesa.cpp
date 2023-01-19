#include "procesa/procesa.h"

int procesa::numero_marcador(int num_comp[2]){
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

bool procesa::search_marker_in_frame(Mat& IMG1, Mat& IMG, uint8_t um, int tiempo){
    Mat Image;
  float points[12];
  int num_comp[2]; //number for compare
  int marker_box[4]; //Se agrega para el seguimiento
  int num_mark; //número de marador
  int v[4]; // para marker_recognition, search_marker_in_frame
  bool marker_found = false;
  uchar *pout;
  int *pin;
  
  Mat Etiquetas, Estadisticas, Centroides;
  String s;

  GaussianBlur(IMG, Image, Size(7, 7), 0, 0);  // Smooth filter
  threshold(Image, Image, um, 255, THRESH_BINARY);
     

  Image = ~Image;

  Mat Image2 = Mat::zeros(480, 640, CV_8UC1);
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
      num_mark = procesa_robot::numero_marcador(num_comp);
    
      procesa::puntosMarcador.push_back(carac(num_mark, points[0], points[1], points[2], points[3], points[4], points[5], points[6], points[7], points[8], points[9], points[10], points[11], marker_box[0], marker_box[2], tiempo ));
      
      s = to_string(num_mark);
      putText(IMG1,s,Point(marker_box[0], marker_box[1]),FONT_HERSHEY_DUPLEX,1,Scalar(0,255,0),2,false);
        // printf("Marcador %d \n", num_mark);
        //FIN reconoce el marcador

    }


    for (int j = v[1]; j <= v[3]; j++) {
      pout = Image2.ptr<uchar>(j) + v[0];
      for (int k = v[0]; k <= v[2]; k++) {
        *pout++ = 0;
      }
    }
  }


  return marker_found;
}

int procesa::obtener_caracteristicas(Mat& IMG1, Mat& IMG, uint8_t um, int tiempo){
    bool marker_found;
    marker_found = search_marker_in_frame(IMG1, IMG, um, tiempo);
    if (marker_found) {
      	return procesa::puntosMarcador.size(); //El número de marcadores encontrados.
    } else {
      	return 0;
    }
}
