//#define  _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <stdio.h>
#include <math.h>
#define _USE_MATH_DEFINES			//definição para cts matemáticas (ex.: pi)
#include <sstream>
#include <string>
#include <tchar.h>
//#include <time.h>
#include <windows.h>		//sleep

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <opencv\highgui.h>
#include <opencv\cv.h>

#include <opencv2/nonfree/features2d.hpp>	//SurfDescriptorExtractor
#include <opencv2/legacy/legacy.hpp>		//BruteForceMatcher

#include <phidget21.h>						//biblioteca para imu


using namespace cv;				//usado nas trackbars
using namespace std;

double spatialData[3];			//aux imu
double spatialData2[3];			//aux imu
double magfield[3];				//aux imu

bool imu_conectado = false;



/*
struct timeval stop, start, difference;	///< aux to mensure the time
struct timeval accdifference;			///< aux to mensure the time

int time_dif(struct timeval *rdiff, struct timeval *xend, struct timeval *ystr)
{
// Runs the carry for the future "-" by updating ystr.
if (xend->tv_usec < ystr->tv_usec) {
int nsec = (ystr->tv_usec - xend->tv_usec) / 1000000 + 1;
ystr->tv_usec -= 1000000 * nsec;
ystr->tv_sec += nsec;
}
if (xend->tv_usec - ystr->tv_usec > 1000000) {
int nsec = (xend->tv_usec - ystr->tv_usec) / 1000000;
ystr->tv_usec += 1000000 * nsec;
ystr->tv_sec -= nsec;
}
// Runs the time remaining to wait tv_usec is "+".
rdiff->tv_sec = xend->tv_sec - ystr->tv_sec;
rdiff->tv_usec = xend->tv_usec - ystr->tv_usec;

// Return 1 if result is negative.
return xend->tv_sec < ystr->tv_sec;
}*/


//**********************************************************************
//*                            IMU functions                           *
//**********************************************************************
//callback that will run if the Spatial is attached to the computer
int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d attached!", serialNo);
	imu_conectado = true;
	return 0;
}

//callback that will run if the Spatial is detached from the computer
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d detached! \n", serialNo);
	imu_conectado = false;
	return 0;
}

//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown)
{
	printf("Error handled. %d - %s \n", ErrorCode, unknown);
	imu_conectado = false;
	return 0;
}

//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event
int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
	int i;
	static int t0 = 0, t1 = 0;		//corrigir static só para t0
	int dt;

	printf("Number of Data Packets in this event: %d\n", count);
	for (i = 0; i < count; i++)
	{
		t1 = (data[i]->timestamp.seconds) * 1000000 + data[i]->timestamp.microseconds;
		dt = t1 - t0;
		spatialData[0] += data[i]->angularRate[0] * dt;	//horizontal/vertical line control
		spatialData[1] += data[i]->angularRate[1] * dt;
		spatialData[2] += data[i]->angularRate[2] * dt;

		printf("\nData Set: %d", i);
		printf("\nAcceleration>\tx: %f  y: %f  z: %f", data[i]->acceleration[0], data[i]->acceleration[1], data[i]->acceleration[2]);
		printf("\nAcce Angular>\tx: %f  y: %f  z: %f", data[i]->angularRate[0], data[i]->angularRate[1], data[i]->angularRate[2]);
		//printf("\nCp Magnético>\tx: %6f  y: %6f  z: %6f", data[i]->magneticField[0], data[i]->magneticField[1], data[i]->magneticField[2]);
		printf("\nTimestamp>\tseconds: %d mileseconds: 0%6d", data[i]->timestamp.seconds, (data[i]->timestamp.microseconds) * 1000);
		if( dt != 0 )printf("\nAmostragens por segundo: %f", 1000/dt);
	
		t0 = t1; // update for next iteration

	}

	printf("\n---------------------------------------------\n");
	return 0;
}


//Display the properties of the attached phidget to the screen.  
//We will be displaying the name, serial number, version of the attached device, the number of accelerometer, gyro, and compass Axes, and the current data rate
// of the attached Spatial.
int display_properties(CPhidgetHandle phid)
{
	int serialNo, version;
	const char *ptr;
	int numAccelAxes, numGyroAxes, numCompassAxes, dataRateMax, dataRateMin;

	CPhidget_getDeviceType(phid, &ptr);
	CPhidget_getSerialNumber(phid, &serialNo);
	CPhidget_getDeviceVersion(phid, &version);
	CPhidgetSpatial_getAccelerationAxisCount((CPhidgetSpatialHandle)phid, &numAccelAxes);
	CPhidgetSpatial_getGyroAxisCount((CPhidgetSpatialHandle)phid, &numGyroAxes);
	CPhidgetSpatial_getCompassAxisCount((CPhidgetSpatialHandle)phid, &numCompassAxes);
	CPhidgetSpatial_getDataRateMax((CPhidgetSpatialHandle)phid, &dataRateMax);
	CPhidgetSpatial_getDataRateMin((CPhidgetSpatialHandle)phid, &dataRateMin);

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("Number of Accel Axes: %i\n", numAccelAxes);
	printf("Number of Gyro Axes: %i\n", numGyroAxes);
	printf("Number of Compass Axes: %i\n", numCompassAxes);
	printf("datarate> Max: %d  Min: %d\n", dataRateMax, dataRateMin);

	return 0;
}

//Declare a spatial handle
CPhidgetSpatialHandle spatial = 0;

int initSpatial()
{
	int result;
	const char *err;

	spatialData[0] = 0;
	spatialData[1] = 0;
	spatialData[2] = 0;

	//create the spatial object
	CPhidgetSpatial_create(&spatial);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)spatial, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)spatial, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);

	//Registers a callback that will run according to the set data rate that will return the spatial data changes
	//Requires the handle for the Spatial, the callback handler function that will be called, 
	//and an arbitrary pointer that will be supplied to the callback function (may be NULL)
	CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);

	//open the spatial object for device connections
	CPhidget_open((CPhidgetHandle)spatial, -1);		//should be NULL Insted -1

	//get the program to wait for a spatial device to be attached
	printf("Waiting for spatial to be attached.... \n");

	if ((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 1000)))
	//if ((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	//Display the properties of the attached spatial device
	display_properties((CPhidgetHandle)spatial);

	//read spatial event data
	printf("Reading.....\n");

	//Set the data rate for the spatial events
	CPhidgetSpatial_setDataRate(spatial, 33);		//datarate em 33 [ms]

	return 0;
}

void closeSpatial()
{
	printf("Closing spatial device...\n");
	CPhidget_close((CPhidgetHandle)spatial);
	CPhidget_delete((CPhidgetHandle)spatial);
}
//**********************************************************************

//**********************************************************************
//*                       Desenhos da Interface                        *
//**********************************************************************
void drawSpatial(IplImage *frame)
{
	CvPoint pt1;
	CvPoint pt2;
	CvScalar red = CV_RGB(250, 0, 0);
	CvScalar black = CV_RGB(0, 0, 0);
	CvScalar white = CV_RGB(255, 255, 255);
	CvScalar blue = CV_RGB(0, 0, 255);

	CvFont font;
	double hScale = .5;
	double vScale = .5;
	int lineWidth = 1;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);
	char *aux_text = (char*)malloc(sizeof(char));

	//---------------------------------nível---------------------------------
	int len = 250;//changed it from 480-Qaiser	// len of line in pixels
	int w = frame->width;
	int h = frame->height;
	int centrox = .5*w;
	int centroy = .5*h;

	double theta;
	if (!imu_conectado)
	{
		char key;
		key = cvWaitKey(1);				// espera 10 ms para ler tecla
		if (char(key) == ',')
		{
			spatialData[1] = spatialData[1] - 250000.0;
		}
		if (char(key) == '.')
		{
			spatialData[1] = spatialData[1] + 250000.0;
		}
	}

	theta = -spatialData[1] * (M_PI / 180.0 / 1000000.0) + M_PI / 2;

	int x1 = .5*(w - len*sin(theta)),
		y1 = .5*(h + len*cos(theta)),

		x2 = .5*(w + len*sin(theta)),
		y2 = .5*(h - len*cos(theta));

	pt1 = cvPoint(x1, y1);
	pt2 = cvPoint(x2, y2);
	cvLine(frame, pt1, pt2, red, 5, 4, 0);	//cvLine(img, pt1, pt2, color, thickness, line_type=8, shift=0 )
	//cvCircle(frame, cvPoint(centrox, centroy), len / 2, white, 1, 4, 0);
	
	int aux_printcos = (int)(1000 * cos(theta));

	pt1 = cvPoint(.7*w, 25);
	//sprintf(aux_text, "cos theta_line: %d", aux_printcos);
	sprintf(aux_text, "angulo: %d", aux_printcos);

	cvPutText(frame, aux_text, pt1, &font, white);


	//--------------------------------bússola--------------------------------
	int l_faixa = 24;				//largura dos marcadores
	int inicio = centrox - .5*centrox;
	int dimen = centrox;

	pt1 = cvPoint(inicio, .95*h);
	pt2 = cvPoint(centrox + .5*centrox, .95*h);
	cvLine(frame, pt1, pt2, white, 1, 4, 0);	//linha base

	if (!imu_conectado)
	{
		char key;
		key = cvWaitKey(1);				// espera 10 ms para ler tecla
		if (char(key) == 'q')			//key_left
		{
			magfield[0] = magfield[0] - 250000.0;
		}
		if (char(key) == 'w')			//key_right
		{
			magfield[0] = magfield[0] + 250000.0;
		}
	}

	theta = -magfield[0] * (M_PI / 180.0 / 1000000.0) + M_PI / 2;

	int i;							//marcadores
	for (i = 0; i <= 8; i = i + 1)
	{					//desenha os 8 marcadores centrais transversais
		pt1 = cvPoint(inicio + i*dimen / 8, .95*h - .5*l_faixa);
		pt2 = cvPoint(inicio + i*dimen / 8, .95*h + .5*l_faixa);
		cvLine(frame, pt1, pt2, white, 1, 4, 0);	//linhas ortogonais
		cvCircle(frame, cvPoint(inicio + i*dimen / 8, .95*h), 1, white, -1, 4, 0);
	}

	pt1 = cvPoint(.7*w, 50);
	sprintf(aux_text, "aux_cp mag: %d", magfield[0]);
	cvPutText(frame, aux_text, pt1, &font, white);

	/*	pt1 = cvPoint(.7*w, 25);
	sprintf(aux_text, "cos theta_line: %0.4g", sin(theta));
	cvPutText(frame, aux_text, pt1, &font, white);
	*/

	aux_printcos = (int)(1000 * cos(theta));
	pt1 = cvPoint(.7*w, 75);
	sprintf(aux_text, "cos theta mag: %d", aux_printcos);
	cvPutText(frame, aux_text, pt1, &font, white);

	//--------------------------------up/down--------------------------------
	//38 = [SETA ACIMA];
	//40 = [SETA ABAIXO];

	l_faixa = 24;				//largura dos marcadores
	inicio = centroy - .5*centroy;
	dimen = centroy;

	pt1 = cvPoint(.95*w, inicio);
	pt2 = cvPoint(.95*w, centroy + .5*centroy);
	cvLine(frame, pt1, pt2, white, 1, 4, 0);	//linha base

	for (i = 0; i <= 8; i = i + 1)
	{					//desenha os 8 marcadores centrais transversais
		pt1 = cvPoint(.95*w - .5*l_faixa, inicio + i*dimen / 8);
		pt2 = cvPoint(.95*w + .5*l_faixa, inicio + i*dimen / 8);
		cvLine(frame, pt1, pt2, white, 1, 4, 0);	//linhas ortogonais
		cvCircle(frame, cvPoint(.95*w, inicio + i*dimen / 8), 1, white, -1, 4, 0);
	}

	//--------------------------------vibração-------------------------------
	CvPoint pts[4];
	pts[0] = cvPoint(1+.025*w, 1+.95*h);	//referencia anti-horária
	pts[1] = cvPoint(1+.025*w, 1+.975*h-1);
	pts[2] = cvPoint(.225*w-1, .975*h-1);
	pts[3] = cvPoint(.225*w-1, 1+.95*h);

	cvFillConvexPoly(frame, pts, 4, blue, 4, 0);		//preenchimento faixa da esq
	
	pt1 = cvPoint(.025*w, .95*h);
	pt2 = cvPoint(.225*w, .975*h);
	cvRectangle(frame, pt1, pt2, white, 1, 4, 0);	//borda faixa

}

void drawSpatial2(IplImage *frame)
{
	CvScalar green = CV_RGB(0, 255, 0);
	CvScalar red = CV_RGB(255, 0, 0);
	CvScalar black = CV_RGB(0, 0, 0);
	CvPoint pt1;
	CvPoint pt2;

	CvFont font;
	double hScale = .5;
	double vScale = .5;
	int    lineWidth = 1;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);

	//char aux_text[255];
	char *aux_text = (char*)malloc(sizeof(char));
	int largura = frame->width;
	int altura = frame->height;
	int centrox = largura / 2;
	int centroy = altura / 2;

	//--------------------------interface estática---------------------------
	CvScalar cor_circulo = CV_RGB(0, 255, 0);
	pt1 = cvPoint(centrox, centroy);

	int raio_e;											//circulo maior
	int raio_i;											//circulo menor
	if (largura > altura){ raio_e = centroy; }
	else{ raio_e = centrox; };							//raio do circulo maior
	raio_e = raio_e*.75;
	raio_i = 0.4*raio_e;								//raio do circulo menor

	//cvCircle(frame, pt1, raio_e, cor_circulo, 1, 4, 0);	//desenha circulo maior
	//cvCircle(frame, pt1, raio_i, cor_circulo, 1, 4, 0);	//desenha circulo menor

	CvScalar cor_faixa = CV_RGB(0, 255, 0);
	int l_faixa = 6;									//largura das 3 faixas
	CvPoint pts[4];

	pt1 = cvPoint(centrox - raio_e, centroy - .5*l_faixa);
	pt2 = cvPoint(centrox - raio_i, centroy + .5*l_faixa);
	//cvRectangle(frame, pt1, pt2, cor_faixa, 1, 4, 0);	//borda faixa da esq
	pts[0] = pt1;
	pts[1] = cvPoint(centrox - raio_e, centroy + .5*l_faixa);
	pts[2] = pt2;
	pts[3] = cvPoint(centrox - raio_i, centroy - .5*l_faixa);
	cvFillConvexPoly(frame, pts, 4, cor_faixa, 4, 0);		//preenchimento faixa da esq

	pt1 = cvPoint(centrox + raio_i, centroy - .5*l_faixa);
	pt2 = cvPoint(centrox + raio_e, centroy + .5*l_faixa);
	//cvRectangle(frame, pt1, pt2, cor_faixa, 1, 4, 0);	//faixa direita
	pts[0] = pt1;
	pts[1] = cvPoint(centrox + raio_i, centroy + .5*l_faixa);
	pts[2] = pt2;
	pts[3] = cvPoint(centrox + raio_e, centroy - .5*l_faixa);
	cvFillConvexPoly(frame, pts, 4, cor_faixa, 4, 0);		//preenchimento faixa da esq

	pt1 = cvPoint(centrox - raio_e, centroy);
	pt2 = cvPoint(centrox + raio_e, centroy);
	cvLine(frame, pt1, pt2, green, 1, 4, 0);				//linha horizontal

	pt1 = cvPoint(centrox - .5*l_faixa, centroy + raio_i);
	pt2 = cvPoint(centrox + .5*l_faixa, centroy + raio_e);
	//cvRectangle(frame, pt1, pt2, cor_faixa, 1, 4, 0);	//faixa inf central
	pts[0] = pt1;
	pts[1] = cvPoint(centrox + .5*l_faixa, centroy + raio_i);
	pts[2] = pt2;
	pts[3] = cvPoint(centrox - .5*l_faixa, centroy + raio_e);
	cvFillConvexPoly(frame, pts, 4, cor_faixa, 4, 0);		//preenchimento faixa da esq

	pt1 = cvPoint(centrox, centroy - raio_e);
	pt2 = cvPoint(centrox, centroy + raio_e);
	cvLine(frame, pt1, pt2, green, 1, 4, 0);			//linha vertical central

	int inicio;
	int dimen = 2 * raio_i;
	int i;
	for (i = 0; i <= 8; i = i + 1)
	{					//desenha os 8 marcadores centrais transversais
		inicio = centroy - raio_i;
		pt1 = cvPoint(centrox - .5*l_faixa, inicio + i*dimen / 8);
		pt2 = cvPoint(centrox + .5*l_faixa, inicio + i*dimen / 8);
		cvLine(frame, pt1, pt2, cor_faixa, 1, 4, 0);	//linhas ortogonais
		cvCircle(frame, cvPoint(centrox, inicio + i*dimen / 8), 1, red, -1, 4, 0);

		inicio = centrox - raio_i;
		pt1 = cvPoint(inicio + i*dimen / 8, centroy - .5*l_faixa);
		pt2 = cvPoint(inicio + i*dimen / 8, centroy + .5*l_faixa);
		cvLine(frame, pt1, pt2, cor_faixa, 1, 4, 0);	//linhas ortogonais
		cvCircle(frame, cvPoint(inicio + i*dimen / 8, centroy), 1, red, -1, 4, 0);
	}

	pt1 = cvPoint(0, 25);
	sprintf(aux_text, "max_x: %d", largura);
	cvPutText(frame, aux_text, pt1, &font, black);

	pt1 = cvPoint(0, 50);
	sprintf(aux_text, "max_y: %d", altura);
	cvPutText(frame, aux_text, pt1, &font, black);

	pt1 = cvPoint(0, 75);
	sprintf(aux_text, "raio externo: %d", raio_e);
	cvPutText(frame, aux_text, pt1, &font, black);


	pt1 = cvPoint(0, 75);
	//sprintf(aux_text, "inclinação: %d", inclinacao);
	//cvPutText(frame, aux_text, pt1, &font, black);

	pt1 = cvPoint(0, 100);
	//sprintf(aux_text, "theta: %d", theta);
	//cvPutText(frame, aux_text, pt1, &font, black);



	//cvLine(frame, pt1, pt2, red, 1, 4, 0);	//cvLine(img, pt1, pt2, color, thickness, line_type=8, shift=0 )

	/*
	x00 = (int)frame->width / 2;
	pt1 = cvPoint(x00, (int)2 * y0);
	pt2 = cvPoint(x00, (int)7 * y0);
	cvLine(frame, pt1, pt2, green, 1, 8);
	*/


	//-------------------------indicador de vibração-------------------------
	float sense = 1;
	float vibration = (sense*abs(spatialData2[0]) + sense*abs(spatialData2[1]) + sense*abs(spatialData2[2]));
	if (vibration > 45){ vibration = 45; }
	//cvCircle(frame, cvPoint(largura / 2, centroy), 30 + (int)vibration, red, .1, 4, 0);
	//substituir por barra

	//---------------------------barras horizontais--------------------------
	/*
	int vertical = spatialData[0] / 1000000;	//ax Y fonte do movimento das barras horizontais
	float size = largura / 7;
	int x00 = (int) 0*size;
	int x01 = (int) 2*size;
	int x10 = (int) 5*size;
	int x11 = (int) 7*size;

	float y0 = altura / 9;
	for (int n = 2; n <= 7; n++)
	{	//left column
	pt1 = cvPoint(x00, (int)n*y0 + vertical);
	pt2 = cvPoint(x01, (int)n*y0 + vertical);
	cvLine(frame, pt1, pt2, green, 1, 8);
	//right column
	pt1 = cvPoint(x10, (int)n*y0 + vertical);
	pt2 = cvPoint(x11, (int)n*y0 + vertical);
	cvLine(frame, pt1, pt2, green, 1, 8);
	}
	*/

}


//**********************************************************************
//*                       Desenhos Rastreamento                        *
//**********************************************************************
void drawSpatial3(IplImage *frame, int x, int y)
{
	int len = 400, len2 = len / 2; // len of line in pixels
	//int w = frame->width, w2 = w / 2;
	//int h = frame->height, h2 = h / 2;
	double theta = -spatialData[1] * (M_PI / 180.0 / 1000000.0) + M_PI / 2;
	int x1 = x - len2*sin(theta),
		y1 = y + len2*cos(theta),
		x2 = x + len2*sin(theta),
		y2 = y - len2*cos(theta);

	CvPoint pt1 = cvPoint(x1, y1);
	CvPoint pt2 = cvPoint(x2, y2);
	CvScalar red = CV_RGB(250, 0, 0);
	cvLine(frame, pt1, pt2, red, 2, 8);

	float y0 = frame->height / 9;
	int x00 = x;
	pt1 = cvPoint(x00, (int)2 * y0);
	pt2 = cvPoint(x00, (int)7 * y0);
	cvLine(frame, pt1, pt2, CV_RGB(250, 0, 0), 1, 8);
}
//**********************************************************************

//**********************************************************************
//*                      Slid bar para uso geral                       *
//**********************************************************************
//using namespace cv;

void func_slide(int v, void*)
{//This function gets called whenever a
	// trackbar position is changed
	//printf("\nNovo valor: %d", v);
}

int H_MIN = 170;
int H_MAX = 180;
int S_MIN = 160;
int S_MAX = 256;
int V_MIN = 60;
int V_MAX = 256;
int A_MIN = 1000;

void geraSlidbars(){

	/*static int H_MIN = 0;
	static int H_MAX = 255;
	static int S_MIN = 0;
	static int S_MAX = 255;
	static int V_MIN = 0;
	static int V_MAX = 255;*/

	const string nomeJanela = "controle HSV";	//nome da janela com slid bars
	namedWindow(nomeJanela, 0);					//janela para slide bars
	//nome do slide, janela inserida, variável modific, malor máximo, func a ser chamada.
	createTrackbar("H_MIN", nomeJanela, &H_MIN, H_MAX, func_slide);
	createTrackbar("H_MAX", nomeJanela, &H_MAX, H_MAX, func_slide);
	createTrackbar("S_MIN", nomeJanela, &S_MIN, S_MAX, func_slide);
	createTrackbar("S_MAX", nomeJanela, &S_MAX, S_MAX, func_slide);
	createTrackbar("V_MIN", nomeJanela, &V_MIN, V_MAX, func_slide);
	createTrackbar("V_MAX", nomeJanela, &V_MAX, V_MAX, func_slide);
	createTrackbar("A_MIN", nomeJanela, &A_MIN, 15300, func_slide);
}
//**********************************************************************

//**********************************************************************
//*                      Funções de reconhecimento                     *
//**********************************************************************
IplImage *imgTracking;

/*
void morphOps(Mat &thresh){
//create structuring element that will be used to "dilate" and "erode" image.
//the element chosen here is a 3px by 3px rectangle
Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
//dilate with larger element so make sure object is nicely visible
Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

erode(thresh, thresh, erodeElement);
erode(thresh, thresh, erodeElement);

dilate(thresh, thresh, dilateElement);
dilate(thresh, thresh, dilateElement);
}*/

//This function threshold the HSV image and create a binary image
IplImage *GetThresholdedImage(IplImage *imgHSV){
	IplImage *imgThresh = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);			//1U, 8U, 16U, 32F 
	cvInRangeS(imgHSV, cvScalar(H_MIN, S_MIN, V_MIN), cvScalar(H_MAX, S_MAX, V_MAX), imgThresh);	//filtra a imagem para as cores selecionadas

	//cvInRangeS(imgHSV, cvScalar(170, 160, 60), cvScalar(180, 2556, 256), imgThresh);	//filtra a imagem para as cores selecionadas
	return imgThresh;																	//retorna imagem filtrada
}

void trackObject(IplImage *imgThresh){
	CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
	cvMoments(imgThresh, moments, 1);
	double moment10 = cvGetSpatialMoment(moments, 1, 0);
	double moment01 = cvGetSpatialMoment(moments, 0, 1);
	double area = cvGetCentralMoment(moments, 0, 0);

	// if the area<1000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
	if (area>A_MIN){
	//if (area>1000){
		// calculate the position of the ball
		int posX = moment10 / area;
		int posY = moment01 / area;

		if (posX >= 0 && posY >= 0)
		{
			cvCircle(imgTracking, cvPoint(posX, posY), 60, cvScalar(255, 0, 0), 3, 4, 0);
			//drawSpatial3(imgTracking, posX, posY);
		}
	}
	free(moments);
}
//**********************************************************************
void draw_level_curves(IplImage *frame)
{
	//cvCircle(frame, cvPoint(200, 200), 50, CV_RGB(0, 255, 0), 10, 8, 0);
	int thickness = 2;
	int lineType = 8;
	int w = frame->width;
	int h = frame->height;
	double angle1 = 0;
	double angle2 = 180;
	
	cvEllipse(frame,
		/*Point(w / 2.0, w / 2.0)*/Point(.75*w,.5*h),
		/*Size(w / 4.0, w / 16.0)*/Size(30,30),
		angle1,
		90,
		270,
		Scalar(255, 0, 0),
		thickness,
		lineType);
	
	cvEllipse(frame,
		/*Point(w / 2.0, w / 2.0)*/Point(.25*w, .5*h),
		/*Size(w / 4.0, w / 16.0)*/Size(30, 30),
		angle2,
		90,
		270,
		Scalar(255, 0, 0),
		thickness,
		lineType);
}




int main(int argc, char* argv[])
{
	bool aux_traking = false;
	//bool aux_traking = true;
	char key;

	//_tsetlocale(LC_ALL, _T("portuguese_brazil"));
	SYSTEMTIME	t_inicio, t_final;				// Guardar os horários de início e fim de execução

	initSpatial();								// get access to accelerometer

	cvNamedWindow("output_console", CV_WINDOW_NORMAL);	//CV_WINDOW_OPENGL: suporte a opengl
	CvCapture *capture = cvCaptureFromCAM(CV_CAP_DSHOW);	//captura com camera externa
	if (!capture)
	{
		CvCapture *capture = cvCaptureFromCAM(CV_CAP_ANY);	// Capture using any camera connected
	}

	IplImage *frame = cvQueryFrame(capture);	// buffer para o display

	/*IplImage *auxf_estatico = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	cvZero(auxf_estatico);						//limpa buffer do traking
	drawSpatial2(auxf_estatico);*/

	// inicia o ponteiro para traking
	if (aux_traking){
		cvNamedWindow("output_threshed", CV_WINDOW_NORMAL);	// cria as janelas
		//cvNamedWindow("output_traked", CV_WINDOW_NORMAL);
		geraSlidbars();							//gera as slidbars
		imgTracking = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	}

	//-------------------teste imagens pré-prontas-------------------
	IplImage *myImage;
	myImage = cvLoadImage("arquivos/inimigomov.png", 1);
	//cvNamedWindow("imagem_carregada", CV_WINDOW_NORMAL);
	cvNamedWindow("imagem_carregada", CV_WINDOW_AUTOSIZE);

	//IplImage *mask = cvCreateImage(cvGetSize(myImage), 8, 3);
	//cvInRangeS( mask, cvScalar(125.0, 0.0, 0.0), cvScalar(255.0, 127.0, 127.0), mask );
	//cvNot(mask, mask);
	//IplImage *myImageWithTransparency = cvCreateImage(cvGetSize(myImage), IPL_DEPTH_8U, 3); //You may need to initialize it before
	//cvCopy(myImage, myImageWithTransparency, mask);
	//cvShowImage("transparente", myImageWithTransparency);

	/*IplImage *auxmyImage = ;
	cvSub(frame, frame2, frame);					//soma frame da detecçao com imagem: src, src2, destino
	cvNamedWindow("transparente", CV_WINDOW_NORMAL);
	cvShowImage("transparente", myImageWithTransparency);*/
	
	//---------------------------------------------------------------

	while (1) {
		//gettimeofday(&start, NULL);						//Get the initial time before start to process the command.
		GetLocalTime(&t_inicio);					// obtem o tempo de início de ConvolveC

		cvShowImage("imagem_carregada", myImage);	// imagem final

		frame = cvQueryFrame(capture);		// adquire nova imagem da webcam

		//--------------------tracker--------------------
		if (aux_traking){
			IplImage *aux_frame = cvCloneImage(frame);	//copia o frame
			cvSmooth(aux_frame, aux_frame, CV_GAUSSIAN, 3, 3); //smooth the original image using Gaussian kernel

			IplImage *imgHSV = cvCreateImage(cvGetSize(aux_frame), IPL_DEPTH_8U, 3);	//cria uma imagem em imgHSV
			cvCvtColor(aux_frame, imgHSV, CV_BGR2HSV);	//muda de RGB para VSH(fonte, destino, codg)

			IplImage *imgThresh = GetThresholdedImage(imgHSV);	//aplica mascara de cor
			cvSmooth(imgThresh, imgThresh, CV_GAUSSIAN, 3, 3);	//filtra imagem usando Gaussiana kernel
			trackObject(imgThresh);								//rastrea objeto pela cor
			//cvAdd(aux_frame, imgTracking, aux_frame);			//soma frame da detecçao com imagem: src, src2, destino
			cvAdd(aux_frame, imgTracking, frame);

			//cvShowImage("output_traked", aux_frame);
			cvShowImage("output_threshed", imgThresh);

			//Clean up used images
			cvReleaseImage(&aux_frame);
			cvReleaseImage(&imgHSV);
			cvReleaseImage(&imgThresh);
			cvZero(imgTracking);			//limpa buffer do traking
		}

		//--------------------console--------------------
		drawSpatial2(frame);				// interface estática
		drawSpatial(frame);					// interface móvel
		//cvAdd(frame, auxf_estatico, frame);	//teste para não ficar desenhando
		draw_level_curves(frame);
		//IplImage *frame2 = cvQueryFrame(capture);	//teste para rastrear movimento
		//cvSub(frame, frame2, frame);				//soma frame da detecçao com imagem: src, src2, destino

		cvShowImage("output_console", frame);	// imagem final

		key = cvWaitKey(5);					// espera em ms para ler tecla

		//-------------------controles-------------------
		if (char(key) == 27) {				//se for 'ESC' fim do programa
			break;
		}

		if (char(key) == 'a') {				//reinicia variaveis
			spatialData[0] = 0;
			spatialData[1] = 0;
			spatialData[2] = 0;

			magfield[0] = 0;
			magfield[1] = 0;
			magfield[2] = 0;
		}

		key = cvWaitKey(5);					// espera em ms para ler tecla
		if (char(key) == 's') {				//grava momento do tiro
			char name_screenshot[100];
			sprintf(name_screenshot, "shot_at_%d-%02d-%02d-%02d-%02d-%02d.png", t_inicio.wYear, t_inicio.wMonth, t_inicio.wDay, t_inicio.wHour, t_inicio.wMinute, t_inicio.wSecond);
			cvSaveImage(name_screenshot, frame);			//salva screenshot
		}

		//gettimeofday(&stop, NULL);
		GetLocalTime(&t_final);				// obtem o tempo de fim de ConvolveC
		//time_dif(&difference, &stop, &start);
		int t_total = (t_final.wSecond - t_inicio.wSecond) * 1000 + (t_final.wMilliseconds - t_inicio.wMilliseconds);

		if (t_total != 0){
			//printf("\nFrames/s: %d", (1000 / t_total));
		}

	}

	closeSpatial();						// close accelerometer
	cvReleaseCapture(&capture);			// Release capture.
	//cvDestroyWindow("Camera_Output");	// Destroy Window
	cvDestroyAllWindows();				// Destroy Window
	//cvReleaseImage(&frame);
	cvReleaseImage(&imgTracking);

	return 0;
}