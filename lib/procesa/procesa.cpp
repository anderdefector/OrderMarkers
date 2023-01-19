#include "procesa/procesa.h"

struct Caracteristicas{
  int num_foto;           //Número de fotografía en la que se encontro el marcador en la foto.
  int ID_marcador;        //Número del marcador encontrado (1, 2, 3, 4 )
  int num_marcador_foto;  //En caso de que existan más marcadores con el mismo ID en la misma foto se les asigna un contador conforme fueron encontrados
  float puntos[12];       //Puntos del marcador encontrado
  int x_centroide;        //Centroide del marcador para saber posición
  float MatrixRt[16];     //Matriz de rotación y traslación
};

list<Caracteristicas> lista_caracteristicas;
list<Caracteristicas> lista_caracteristicas_leida;

typedef tuple <int, float, float, float, float, float, float, float, float, float, float, float, float, int, int, int > carac;
vector <carac>  puntosMarcador;
vector <carac>  puntosMarcadorRecibidos;

//Robot

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


bool search_marker_in_frame(Mat& IMG1, Mat& IMG, uint8_t um, int tiempo) {

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
      num_mark = numero_marcador(num_comp);

      puntosMarcador.push_back(carac(num_mark, points[0], points[1], points[2], points[3], points[4], points[5], points[6], points[7], points[8], points[9], points[10], points[11], marker_box[0], marker_box[2], tiempo ));
      
      s = to_string(num_mark);
      putText(IMG1,s,Point(marker_box[0], marker_box[1]),FONT_HERSHEY_DUPLEX,1,Scalar(0,255,0),2,false);
        // printf("Marcador %d \n", num_mark);
        //FIN reconoce el marcador

    }

    //imshow("Original", IMG1);#include "homography/homografia.h"

    for (int j = v[1]; j <= v[3]; j++) {
      pout = Image2.ptr<uchar>(j) + v[0];
      for (int k = v[0]; k <= v[2]; k++) {
        *pout++ = 0;
      }
    }
  }


  return marker_found;
}


/*
  Se usa para saber si existen marcadores y cuantos se econtraron.
*/

int obtener_caracteristicas(Mat& IMG1, Mat& IMG, uint8_t um, int tiempo){
    bool marker_found;
    marker_found = search_marker_in_frame(IMG1, IMG, um, tiempo);
    if (marker_found) {
      	return puntosMarcador.size(); //El número de marcadores encontrados.
    } else {
      	return 0;
    }
}

void cambia_DatoAumentado(unsigned char buffer[55], int i, int dato){
 
    buffer[i] = dato /255 ;
    buffer[i+1] = dato % 255;

  //printf("%d, %d ", dato, (dato % 255));
}

void cambia_coordenadasAumentadas(unsigned char buffer[55], int i, float coordenada){
  float tmp;
  int ParteEntera;
  int DivEnt, ResEnt, DivFlt, ResFlt;
  int ParteFlotante;

  ParteEntera = (int) coordenada;
  tmp = coordenada - (float) ParteEntera;
  //printf("%f \n", tmp);
  tmp = tmp  * 1000;

  ParteFlotante = (int) tmp;

  //printf("Ent: %d Float: %d\n", ParteEntera, ParteFlotante);

  DivEnt = ParteEntera / 255;
  ResEnt = ParteEntera % 255;
  
  DivFlt = ParteFlotante / 255;
  ResFlt = ParteFlotante % 255;

  buffer[i] = DivEnt;
  buffer[i+1] = ResEnt;
  buffer[i+2] = DivFlt;
  buffer[i+3] = ResFlt;

  //printf("Ent Div: %d Res: %d, Float div: %d Res: %d \n", buffer[i], buffer[i+1], buffer[i+2], buffer[i+3]);

}

void enviar_caracteristicasTrans(int num_marcadores, int sock){
  int indice_marcador = 0;  
  int div, resto;
  int i = 0;
  int valread;
  unsigned char buffer_enviado[55] = {0} ;
  char buffer_recibido[2] = {0};
  while(i < num_marcadores){
      printf("Esperando a la computadora \n");
      valread = read(sock, buffer_recibido, 2); //indicación de la computadora lista para recibir 
      if(buffer_recibido[0] == 1){
          buffer_enviado[0] = get<0>(puntosMarcador.at(i)); //ID del marcador
          cambia_coordenadasAumentadas(buffer_enviado, 1, get<1>(puntosMarcador.at(i)));
          cambia_coordenadasAumentadas(buffer_enviado, 5, get<2>(puntosMarcador.at(i)));
          cambia_coordenadasAumentadas(buffer_enviado, 9, get<3>(puntosMarcador.at(i)));
          cambia_coordenadasAumentadas(buffer_enviado, 13, get<4>(puntosMarcador.at(i)));
          cambia_coordenadasAumentadas(buffer_enviado, 17, get<5>(puntosMarcador.at(i)));
          cambia_coordenadasAumentadas(buffer_enviado, 21, get<6>(puntosMarcador.at(i)));
          cambia_coordenadasAumentadas(buffer_enviado, 25, get<7>(puntosMarcador.at(i)));
          cambia_coordenadasAumentadas(buffer_enviado, 29, get<8>(puntosMarcador.at(i)));
          cambia_coordenadasAumentadas(buffer_enviado, 33, get<9>(puntosMarcador.at(i)));
          cambia_coordenadasAumentadas(buffer_enviado, 37, get<10>(puntosMarcador.at(i)));
          cambia_coordenadasAumentadas(buffer_enviado, 41, get<11>(puntosMarcador.at(i)));
          cambia_coordenadasAumentadas(buffer_enviado, 45, get<12>(puntosMarcador.at(i)));
          // Fin puntos del marcador
          cambia_DatoAumentado(buffer_enviado, 49, get<13>(puntosMarcador.at(i))); // Inicio coordenada X
          cambia_DatoAumentado(buffer_enviado, 51, get<14>(puntosMarcador.at(i))); // Ancho
          cambia_DatoAumentado(buffer_enviado, 53, get<15>(puntosMarcador.at(i))); //Estampa de tiempo
          send(sock , buffer_enviado , 55 , 0 ); //Enviar dato a la computadora
          i++;   
      } 
  }
  puntosMarcador.clear(); // Se limpia el vector al terminar de enviar
  printf("Puntos enviados\n");
}

//Computadora

int cambia_DatoRecibido(int DivEnt, int ResEnt){
  return (DivEnt * 255) + ResEnt;
}

float cambia_coordenadasRecibidas(int DivEnt, int ResEnt, int DivFlt, int ResFlt){
  float Ent, Flt, coordenada;
  int tmp, tmp2;
  tmp = (DivEnt * 255) + ResEnt;
  //printf(" tmp = %d \n", tmp);
  Ent = (float) tmp;
  //printf(" Ent = %f \n", Ent);
  tmp2 = (DivFlt * 255) + ResFlt;
  //printf(" tmp2 = %d \n", tmp2);
  Flt = ((float) tmp2) / 1000; 
  //printf(" Flt = %f \n", Flt);

  coordenada = Ent + Flt;
  //printf("%f \n", coo);
  return coordenada;
}

void recibir_caracteristicas(int num_marcadores, int sock){
    int bandera;
    int valread;
    int i = 0;
    float MatrixK[16];
    char buffer_enviado[2] = {0};
    //char buffer_recibido[17] = {0};
    unsigned char buffer_recibido[55] = {0};
    double filas = 480.0;    //Alto de la imagen cámara Raspberry
    double columnas = 640.0; //Ancho de la imagen cámara Raspberry
    int indice_marcador = 0;
    int num_M1 = 0;
    int num_M2 = 0;
    int num_M3 = 0;
    int num_M4 = 0;
    int num_M = 0;

    Caracteristicas c_recibidas;

    //Modelos normalizados de marcadores de 10 x 10 
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

    //Modelos normalizados de marcadores de 10 x 10


    while(indice_marcador < num_marcadores){
      buffer_enviado[0] = 1;
      send(sock , buffer_enviado , 2 , 0 ); //Enviar dato al robot para recibir puntos
      valread = read(sock, buffer_recibido, 55); // Lee el dato del robot

      c_recibidas.num_foto = cambia_DatoRecibido(buffer_recibido[53], buffer_recibido[54]);
      c_recibidas.ID_marcador = buffer_recibido[0];
      for(int j=0; j<12; j++){
        c_recibidas.puntos[j] = cambia_coordenadasRecibidas(buffer_recibido[4*j+1], buffer_recibido[4*j+2], buffer_recibido[4*j+3], buffer_recibido[4*j+4]);
      }

      c_recibidas.x_centroide = cambia_DatoRecibido(buffer_recibido[49], buffer_recibido[50]) + (cambia_DatoRecibido(buffer_recibido[51], buffer_recibido[52]) / 2 );

      switch( c_recibidas.ID_marcador ){
        case 1:
          bandera = homografia(NormalModel1, vdP1, c_recibidas.puntos, MatrixK, c_recibidas.MatrixRt, filas, columnas);
          printf("Modelo 1 \n");
          num_M = num_M1; //Cuenta cuantos marcadores del mismo modelo hay en la misma foto
          num_M1++;
        break;
        case 2:
          printf("Modelo 2 \n");
          bandera = homografia(NormalModel2, vdP2, c_recibidas.puntos, MatrixK, c_recibidas.MatrixRt, filas, columnas);
          num_M = num_M2; //Cuenta cuantos marcadores del mismo modelo hay en la misma foto
          num_M2++;
        break;
        case 3:
          bandera = homografia(NormalModel3, vdP3, c_recibidas.puntos, MatrixK, c_recibidas.MatrixRt, filas, columnas);
          printf("Modelo 3 \n");
          num_M = num_M3; //Cuenta cuantos marcadores del mismo modelo hay en la misma foto
          num_M3++;
        break;
        case 4:
          bandera = homografia(NormalModel4, vdP4, c_recibidas.puntos, MatrixK, c_recibidas.MatrixRt, filas, columnas);
          printf("Modelo 4 \n");
          num_M = num_M4; //Cuenta cuantos marcadores del mismo modelo hay en la misma foto
          num_M4++;
        break;
        default:
          printf("No existe ese marcador.");
        break;  
    }
      c_recibidas.num_marcador_foto = num_M; //Cuenta cuantos marcadores del mismo modelo hay en la misma foto

      lista_caracteristicas.push_back(c_recibidas);
      indice_marcador++;
    }
}


void imprimir_lista(){
    printf("Datos en lista \n");
    for(Caracteristicas item : lista_caracteristicas ){
      std::cout<< "\nNum Foto: " << item.num_foto<< " ID Marcador: "<<item.ID_marcador << " Num M Foto: "<< item.num_marcador_foto << " X: " <<item.x_centroide << std::endl;
      printf("Puntos: \n");
      for(int i=0; i<12; i++ ){
        printf("%f  ", item.puntos[i]);
      }
      printf(" \nRt: \n");
      for(int i =0; i <16; i++){
        if((i)%4 == 0){
          printf("\n");
        }
        printf("%f ",item.MatrixRt[i]);
      }
    }
}

void imprimir_lista_leida(){
    printf("Datos en lista leida \n");
    for(Caracteristicas item : lista_caracteristicas_leida ){
      std::cout<< "\nNum Foto: " << item.num_foto<< " ID Marcador: "<<item.ID_marcador << " Num M Foto: "<< item.num_marcador_foto << " X: " <<item.x_centroide << std::endl;
      printf("Puntos: \n");
      for(int i=0; i<12; i++ ){
        printf("%f  ", item.puntos[i]);
      }
      printf(" \nRt: \n");
      for(int i =0; i <16; i++){
        if((i)%4 == 0){
          printf("\n");
        }
        printf("%f ",item.MatrixRt[i]);
      }
    }
}

void guardar_lista(){
    fstream f;
    f.open("../Lista.dat", ios::out | ios::binary);
    if(f.is_open()){
      for(Caracteristicas item : lista_caracteristicas ){
          f.write(reinterpret_cast<char*>(&item),sizeof(Caracteristicas));
      }
      f.close();
    }else{
      printf("Error al crear el archivo\n");
    }
}

void abrir_lista(){
    fstream f;
    Caracteristicas tmp;
    f.open("../Lista.dat", ios::in | ios::binary);
    if(f.is_open()){
      while(f.read(reinterpret_cast<char*>(&tmp),sizeof(tmp))){
        lista_caracteristicas_leida.push_back(tmp);
      }
      f.close();
    }else{
      printf("Error al crear el archivo\n");
    }

}




//Opcionales


void obtener_homografias(Mat& IMG1, Mat& IMG, uint8_t um){ 
    float MatrixK[16];
    float MatrixRt[16];
    float points[12];
    bool marker_found;
    int state;
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

    

    marker_found = search_marker_in_frame(IMG1, IMG, um, 0);
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
          bandera = homografia(NormalModel1, vdP1, points, MatrixK, MatrixRt, (double)IMG1.rows, (double)IMG1.cols);
          printf("Modelo 1 \n");
        break;
        case 2:
          printf("Modelo 2 \n");
          bandera = homografia(NormalModel2, vdP2, points, MatrixK, MatrixRt, (double)IMG1.rows, (double)IMG1.cols);
        break;
        case 3:
          bandera = homografia(NormalModel3, vdP3, points, MatrixK, MatrixRt, (double)IMG1.rows, (double)IMG1.cols);
          printf("Modelo 3 \n");
        break;
        case 4:
          bandera = homografia(NormalModel4, vdP4, points, MatrixK, MatrixRt, (double)IMG1.rows, (double)IMG1.cols);
          printf("Modelo 4 \n");
        break;
        default:
          printf("No existe ese marcador.");
        break;
    }

      printf(" \nRt: \n");
      for(int i =0; i <16; i++){
        if((i)%4 == 0){
          printf("\n");
        }
        printf("%f ",MatrixRt[i]);
      }
    
      for(int i=0; i<12; i++){
        if(i%2 == 0){
          printf("\n");
        }
        printf("%f ",points[i]);

      } 
      
       printf("\n");

    }
    puntosMarcador.clear();


}


