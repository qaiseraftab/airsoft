/* Fabiola Maffra - 0711279 */

#include<opencv\cv.h>
#include<opencv\highgui.h>
#include<opencv\cvaux.h>
#include <stdio.h>

int threshold = 50;

/* soma dois frames para depois calcular a media*/
void avrgTrain(IplImage* frame, IplImage* avrg)
{
	int i, j;
	CvScalar BGR_avrg, BGR;

	for(i = 0; i < frame->height; i++)
	{
		for(j = 0; j < frame->width; j++)
		{
			BGR = cvGet2D(frame,i,j);
			BGR_avrg = cvGet2D(avrg,i,j);
			BGR_avrg.val[0] = BGR_avrg.val[0] + BGR.val[0]; 
			BGR_avrg.val[1] = BGR_avrg.val[1] + BGR.val[1];
			BGR_avrg.val[2] = BGR_avrg.val[2] + BGR.val[2];
			cvSet2D(avrg,i,j,BGR_avrg);
		}
	}
}

/* distingue partes dinamicas e estaticas */
void blackWhite(IplImage* avrg, IplImage* frame, IplImage* newFrame, int t, CvScalar black, CvScalar white)
{
	int i, j;
	CvScalar BGR, BGR_avrg;

	for(i = 0; i < frame->height; i++)
	{
		for(j = 0; j < frame->width; j++)
		{
			BGR = cvGet2D(frame,i,j);
			BGR_avrg = cvGet2D(avrg,i,j);

			if( (abs(BGR.val[0] - BGR_avrg.val[0])<=t)&&(abs(BGR.val[1] - BGR_avrg.val[1])<=t)&&(abs(BGR.val[2] - BGR_avrg.val[2])<=t) )
			{
				cvSet2D(newFrame,i,j,black);
			}
			else
			{
				cvSet2D(newFrame,i,j,white);
			}
		}
	}
}

int main( void )
{
	CvCapture* capture = NULL;
	IplImage* frame = NULL;
	int nFrames;
	IplImage* avrg = NULL;
	IplImage* avrg2 = NULL;
	IplImage* newFrame = NULL;
	IplImage* median = NULL;
	IplImage* imgClosed = NULL;
	IplImage* imgTemp = NULL;
	int w, h, i, nc, nl, step, numContours, j;
	CvSeq* first_contour;
	IplConvKernel* elem;
	IplImage* img;
	CvMemStorage* contour_storage;
	CvScalar s, black, white;
	contour_storage = cvCreateMemStorage(0);

	nFrames = 0;

	black.val[0] = black.val[1] = black.val[2] = 0x00;
	white.val[0] = white.val[1] = white.val[2] = 0xFF;

	cvNamedWindow("Input", CV_WINDOW_AUTOSIZE);
	cvCreateTrackbar( "Threshold", "Input", &threshold, 255, NULL );
	cvNamedWindow("Output", CV_WINDOW_AUTOSIZE);

	capture = cvCaptureFromCAM(CV_CAP_ANY);	// Capture using any camera connected	

	if( !capture )
	{
		printf("Could not initialize capturing...\n");
		return 0;
	}

	frame = cvQueryFrame(capture);
	if(frame)
	{
		avrg = cvCreateImage( cvGetSize(frame), IPL_DEPTH_32F, 3 );
		newFrame = cvCloneImage(frame);
		avrg2 = cvCloneImage(frame);
		median = cvCloneImage(frame);
		imgClosed = cvCloneImage(frame);
		imgTemp = cvCloneImage(frame);
		img = cvCreateImage( cvGetSize(frame), 8, 1 );
		nFrames++;
	}

	for(i = 0; i < frame->height; i++)
	{
		for(j = 0; j < frame->width; j++)
		{
			cvSet2D(avrg,i,j,cvGet2D(frame,i,j));
		}
	}

	// Treinamento
	while(nFrames <= 30)
	{
		frame = cvQueryFrame(capture);
		if(!frame) break;
		nFrames++;

		if(cvWaitKey(1) >= 0) break;
		avrgTrain(frame, avrg);
	}

	for(i = 0; i < frame->height; i++)
	{
		for(j = 0; j < frame->width; j++)
		{
			s = cvGet2D(avrg,i,j);
			s.val[0] = s.val[0]*0.034f;
			s.val[1] = s.val[1]*0.034f;
			s.val[2] = s.val[2]*0.034f;
			cvSet2D(avrg2,i,j,s);
		}
	}

	elem = cvCreateStructuringElementEx(9, 9, 4, 4, CV_SHAPE_ELLIPSE);

	while(1)
	{
		frame = cvQueryFrame(capture);
		if(!frame) break;

		blackWhite(avrg2, frame, newFrame, threshold, black, white);

		cvSmooth( newFrame, median, CV_MEDIAN, 3, 3, 0, 0 );
		cvMorphologyEx(median, imgClosed, imgTemp, elem, CV_MOP_CLOSE, 1);
	
		/* Pega os contornos ativos */
		cvCvtColor( imgClosed, img, CV_BGR2GRAY );

		numContours = cvFindContours( img, contour_storage, &first_contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1);

		for (i = 0 ; i<numContours ; first_contour = first_contour->h_next, i++) 
		{
			cvDrawContours(imgClosed,first_contour,CV_RGB(255,0,0),CV_RGB(255,0,0),2,1,8,cvPoint(0,0));
		}

		cvShowImage("Input", frame);
		cvShowImage("Output", imgClosed);

		if(cvWaitKey(1) >= 0) break;
	}

	cvWaitKey(10000);

	cvReleaseCapture(&capture);

	/* Libera as janelas */
	cvDestroyWindow("Input");
	cvDestroyWindow("Output");

	/* Libera as imagens */
	cvReleaseImage(&avrg);
	cvReleaseImage(&median);
	cvReleaseImage(&newFrame);
	cvReleaseImage(&imgClosed);
	cvReleaseImage(&imgTemp);

	return 0;
}