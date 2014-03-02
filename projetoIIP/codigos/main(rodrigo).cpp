#define  _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <stdio.h>
#include <math.h>
#define _USE_MATH_DEFINES		//allow ct pi

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <phidget21.h>

using namespace std;
char key;
double spatialData[3];
double spatialData2[3];
int lvibx;
int lviby;

float vibx = 0,viby = 0;

float ax,ay,az;
float radius=90;


//callback that will run if the Spatial is attached to the computer
int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d attached!", serialNo);

	return 0;
}

//callback that will run if the Spatial is detached from the computer
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d detached! \n", serialNo);

	return 0;
}

//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown)
{
	printf("Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}

//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event


int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
	int i;
	static int t0 = 0, t1 = 0;
	int dt;

	printf("Number of Data Packets in this event: %d\n", count);
	for (i = 0; i < count; i++)
	{
		t1 = (data[i]->timestamp.seconds) * 1000000 + data[i]->timestamp.microseconds;
		dt = t1 - t0;
		spatialData[0] += data[i]->angularRate[0] * dt;
		spatialData[1] += data[i]->angularRate[1] * dt;
		spatialData[2] += data[i]->angularRate[2] * dt;

		ax=data[i]->acceleration[0];
		ay=data[i]->acceleration[1];
		az=data[i]->acceleration[2];

		printf("=== Data Set: %d ===\n", i);
		printf("Acceleration> x: %6f  y: %6f  x: %6f\n", data[i]->acceleration[0], data[i]->acceleration[1], data[i]->acceleration[2]);
		printf("Angular Rate> x: %6f  y: %6f  x: %6f\n", data[i]->angularRate[0], data[i]->angularRate[1], data[i]->angularRate[2]);
		printf("Magnetic Field> x: %6f  y: %6f  x: %6f\n", data[i]->magneticField[0], data[i]->magneticField[1], data[i]->magneticField[2]);
		printf("Timestamp> seconds: %d -- microseconds: %d\n", data[i]->timestamp.seconds, data[i]->timestamp.microseconds);

		t0 = t1; // update for next iteration
	}

	printf("---------------------------------------------\n");
	//system("pause");
	return 0;
}


//Display the properties of the attached phidget to the screen.  
//We will be displaying the name, serial number, version of the attached device, the number of accelerometer, gyro, and compass Axes, and the current data rate
// of the attached Spatial.
int display_properties(CPhidgetHandle phid)
{
	int serialNo, version;
	const char* ptr;
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
	CPhidget_open((CPhidgetHandle)spatial, -1);

	//get the program to wait for a spatial device to be attached
	printf("Waiting for spatial to be attached.... \n");
	if ((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 10000)))
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
	CPhidgetSpatial_setDataRate(spatial, 33);

	return 0;
}

void closeSpatial()
{
	printf("Closing spatial device...\n");
	CPhidget_close((CPhidgetHandle)spatial);
	CPhidget_delete((CPhidgetHandle)spatial);
}

void drawSpatial(IplImage *frame)
{
	int len = 100, len2 = len / 2; // len of line in pixels
	int w = frame->width, w2 = w / 2;
	int h = frame->height, h2 = h / 2;

	double theta = -spatialData[1] * (M_PI / 180.0 / 1000000.0);

	int x1 = w2 - len2*sin(theta),
		y1 = h2 + len2*cos(theta),
		x2 = w2 + len2*sin(theta),
		y2 = h2 - len2*cos(theta);

	CvPoint pt1 = cvPoint(x1, y1);
	CvPoint pt2 = cvPoint(x2, y2);
	CvScalar red = CV_RGB(250, 0, 0);
	cvLine(frame, pt1, pt2, red, 10, 8);
}

void drawSpatial2(IplImage *frame)
{
	CvScalar green = CV_RGB(0, 250, 0);
	int vertical = spatialData[0] / 1000000;	//ax Y

	int sensi = 1000000;

	
	//int vibration = abs(vibx - lvibx + viby - lviby);
	int vibration = abs(vibx) + abs(viby);

	if (vibration >= 20){ vibration = 20; }


	float size = frame->width / 7;
	int x00 = (int)2 * size;
	int x01 = (int)3 * size;
	int x10 = (int)4 * size;
	int x11 = (int)5 * size;

	float y0 = frame->height / 9;

	for (int n = 2; n <= 7; n++)
	{	//left column
		CvPoint pt1 = cvPoint(x00, (int)n*y0 + vertical);
		CvPoint pt2 = cvPoint(x01, (int)n*y0 + vertical);
		cvLine(frame, pt1, pt2, green, 1, 8);
		//right column
		pt1 = cvPoint(x10, (int)n*y0 + vertical);
		pt2 = cvPoint(x11, (int)n*y0 + vertical);
		cvLine(frame, pt1, pt2, green, 1, 8);
	}
		
	radius = radius - 12;

	if(radius < 90 ) radius = 90 ;

	radius = radius + 3*(ax+ay+az); 
	if(radius > 180) radius = 180;

	cvCircle(frame, cvPoint(frame->width / 2, frame->height / 2), (int)radius, green, 1, CV_AA, 0);
	//cvCircle(frame, cvPoint(150, 150), 90, CV_RGB(0, 0, 255), 1, CV_AA, 0);
	/* first argument is the pointer to IplImage, second is the center of the circle
	3rd is the radius, 4th is the color, 5th is thickness (if <0 it will be filled),
	6 is the linetype, last is the shift (number of fractional bits in the point
	coordinates).
	Primeiro argumento é o ponteiro para uma estrutura IplImage, segundo é o centro
	do circulo, terceiro é o raio, 4° é a cor, os tres ultimos são iguais ao cvLine
	*/

	char text[255];
	//sprintf(text, "Score %d \n comprimento: %d \n altura: %d", (int)vertical, (int)7 * x00 / 2, (int)size*9);
	sprintf(text, "variavel vibr X: %d vibr Y: %d", vibx, viby);
	//system("pause");

	CvFont font;
	double hScale = 1.0;
	double vScale = 1.0;
	int    lineWidth = 1;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);
	cvPutText(frame, text, cvPoint(0, 200), &font, green);

	lvibx = vibx;
	lviby = viby;

}

int main()
{
	// get access to accelerometer
	initSpatial();

	// Create window
	cvNamedWindow("Camera_Output", 1);

	// Capture using any camera connected
	CvCapture *capture = cvCaptureFromCAM(CV_CAP_ANY);

	//Create infinte loop for live streaming
	while (1) {
		// Create image frames from capture
		IplImage *frame = cvQueryFrame(capture);
		
		// draw carpenter's level
		drawSpatial(frame);

		drawSpatial2(frame);
		// Show image frames on created window
		cvShowImage("Camera_Output", frame);

		// Capture Keyboard stroke
		key = cvWaitKey(10);

		// If you hit ESC key loop will break.
		if (char(key) == 27) {
			break;
		}
		if (char(key) == 'a') {
			spatialData[0] =
				spatialData[1] =
				spatialData[2] = 0.0;
		}
	}

	// Release capture.
	cvReleaseCapture(&capture);

	// Destroy Window
	cvDestroyWindow("Camera_Output");

	// close accelerometer
	closeSpatial();

	return 0;
}
