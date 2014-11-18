#include <opencv2/opencv.hpp>
#include <opencv/cxmisc.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.h>
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>

#include <iostream>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>
//#include "mcv.h"
//#include "CvexTime.h"
//#include "CvexStereoCameraCalibration.h"
//#include "CvexCameraCalibration.h"


#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define CHESS_SIZE cvSize(7,4)

enum
{
	VIEW_LEFT=0,
	VIEW_RIGHT,
	VIEW_ANAGLYPH,
	VIEW_INTERLACE,
	VIEW_SIDEBYSIDE,
	VIEW_DEPTHMAP,//max value for trackbar
};

void RectificationRGB (IplImage* left, IplImage* right, IplImage*
		render, int shift, bool RGB, float &valor_resta_rojo, float
		&valor_resta_azul)
{
	cvNamedWindow ("Rectificacion RGB", CV_WINDOW_AUTOSIZE);
	int mode = CVEX_CONNECT_HORIZON;
	IplImage* a1=cvCreateImage(cvGetSize(left),8,1);
	IplImage* a2=cvCreateImage(cvGetSize(left),8,1);
	IplImage* a3=cvCreateImage(cvGetSize(left),8,1);
	IplImage* left2 = cvCloneImage(left);
	IplImage* right2 = cvCloneImage(right);
	IplImage* left3 = cvCreateImage(cvGetSize(left),8,3);
	IplImage* right3 = cvCreateImage(cvGetSize(left),8,3);
	int valor_rojo = 40;
	int valor_medio_rojo = valor_rojo/2;
	int valor_azul = 40;
	int valor_medio_azul = valor_azul/2;
	float val_prov1;
	float val_prov2;
	float val_prov3;
	IplImage* render2 = cvCloneImage(render);
	cvCreateTrackbar("Componente Roja","Rectificacion RGB",&valor_medio_rojo,valor_rojo,NULL);
	cvCreateTrackbar("Componente Cian","Rectificacion RGB",&valor_medio_azul,valor_azul,NULL);
	IplImage* swap = cvCreateImage(cvGetSize(left),8,left->nChannels);
	IplImage* temp = cvCreateImage(cvGetSize(left),8,left->nChannels);
	IplImage* render_syde = cvCloneImage(render);
	cvSplit(left,NULL,NULL,a1,NULL);
	cvSplit(right,a2,a3,NULL,NULL);
	cvMerge(a2,a3,NULL,NULL,right3);
	cvMerge(NULL,NULL,a1,NULL,left3);
	cvexWarpShift(left3,swap,-shift,0);
	cvexWarpShift(right3,temp,+shift,0);
	IplImage* connect = cvexConnect(swap,temp,mode);
	cvResize(connect,render_syde);
	int key = 0;
	while (key!=1048603) //TECLA ESC
	{
		if(key==CVEX_KEY_ARROW_UP && RGB == true)
		{
			valor_medio_rojo--;
			cvSetTrackbarPos("Componente Roja","Rectificacion RGB",valor_medio_rojo);
		}
		if(key==CVEX_KEY_ARROW_DOWN && RGB == true)
		{
			valor_medio_rojo++;
			cvSetTrackbarPos("Componente Roja","Rectificacion RGB",valor_medio_rojo);
		}
		if(key==CVEX_KEY_ARROW_LEFT && RGB == true)
		{
			valor_medio_azul--;
			cvSetTrackbarPos("Componente Cian","Rectificacion RGB",valor_medio_azul);
		}
		if(key==CVEX_KEY_ARROW_RIGHT && RGB == true)
		{
			valor_medio_azul++;
			cvSetTrackbarPos("Componente Cian","Rectificacion RGB",valor_medio_azul);
		}
		valor_resta_rojo = (valor_medio_rojo -
				(valor_rojo/2))*40;
		valor_resta_azul = (valor_medio_azul -
				(valor_azul/2))*40;
		//printf("%f\n%f\n",valor_resta_azul, valor_resta_rojo);
		int i, j;
		CvScalar s1, s2, s3, s4;
		for( i= 0; i < render->height; i++ )
		{
			for(j = 0; j < render->width; j++ )
			{
				s1 = cvGet2D(left, i, j);
				s2 = cvGet2D(left2, i, j);
				s3 = cvGet2D(right, i, j);
				s4 = cvGet2D(right2, i, j);
				val_prov2 = s2.val[2];
				val_prov1 = s4.val[0];
				val_prov3 = s4.val[1];
				s3.val[0] = val_prov1 + valor_resta_azul;
				s3.val[1] = val_prov3 + valor_resta_azul;
				s1.val[2] = val_prov2 + valor_resta_rojo;
				cvSet2D(left, i, j, s1);
				cvSet2D(right, i, j, s3);
			}
		}
		IplImage* g1 = cvCreateImage(cvGetSize(left),8,1);
		IplImage* g2 = cvCreateImage(cvGetSize(left),8,1);
		IplImage* swap2 = cvCreateImage(cvGetSize(left),8,1);
		cvCvtColor(left,swap2,CV_BGR2GRAY);
		cvexWarpShift(swap2,g1,-shift,0);
		cvCvtColor(right,swap2,CV_BGR2GRAY);
		cvexWarpShift(swap2,g2,shift,0);
		cvMerge(g2,g2,g1,NULL,render);
		cvShowImage ("Rectificacion RGB", render_syde);
		cvShowImage ("WEBCAM 3D. SELECCION DE MODO", render);
		key = cvWaitKey (0);
	}
	cvDestroyWindow("Rectificacion RGB");
}
void modifica_pixels (IplImage* left, IplImage* right, IplImage*
		left_out, IplImage* right_out, float valor_resta_rojo, float
		valor_resta_azul)
{
	CvScalar s1, s2;
	int i, j;
	float val_prov2;
	float val_prov1;
	for( i =0; i < left->height; i++ )
	{
		for(j = 0; j < left->width; j++ )
		{
			s1 = cvGet2D(left, i, j);
			s2 = cvGet2D(right, i, j);
			val_prov2 = s1.val[0];
			val_prov1 = s2.val[2];
			s2.val[2] = val_prov1 + valor_resta_rojo;
			s1.val[0] = val_prov2 + valor_resta_azul;
			cvSet2D(left_out, i, j, s1);
			cvSet2D(right_out, i, j, s2);
		}
	}
}
void cvexMakeStereoImageInterlace(IplImage* left, IplImage* right, IplImage* dest, int shift)
{
	cvexWarpShift(left,dest,-shift,0);
	IplImage* swap = cvCreateImage(cvGetSize(right),8,3);
	cvexWarpShift(right,swap,+shift,0);
#pragma omp parallel for
	for(int j=0;j<left->height;j++)
	{
		if(j%2==1)
		{
			for(int i=0;i<left->width;i++)
			{
				dest->imageData[j*left->widthStep+i*3+0]=swap->imageData[j*left->widthStep+i*3+0];
				dest->imageData[j*left->widthStep+i*3+1]=swap->imageData[j*left->widthStep+i*3+1];
				dest->imageData[j*left->widthStep+i*3+2]=swap->imageData[j*left->widthStep+i*3+2];
			}
		}
	}
	cvReleaseImage(&swap);
}
void Move_image (IplImage* image_left, IplImage* image_right,int *_v, int *_h, bool calibracionRGB, float valor_resta_rojo, float valor_resta_azul, int modeAnaglyph, int dis)
{
	int v_max = image_left->height/5;
	int h_max = image_left->width;
	cvNamedWindow ("Rectificacion Manual", CV_WINDOW_AUTOSIZE);
	int v=*_v+v_max/2;
	cvCreateTrackbar("v","Rectificacion Manual",&v,v_max,NULL);
	int h = *_h+h_max/2;
	cvCreateTrackbar("h","Rectificacion Manual",&h,h_max,NULL);
	IplImage* render = cvCloneImage(image_left);
	IplImage* render_entrelazada = cvCloneImage(image_left);
	IplImage* render_anaglyph = cvCloneImage(image_left);
	int key = 0;
	while (key !=1048603) //TECLA ESC
	{
		cvexWarpShift(image_right,render,h-h_max/2,v-v_max/2);
		cvexMakeStereoImageInterlace(image_left,render,render_entrelazada,0);
		cvShowImage ("Rectificacion Manual",
				render_entrelazada);
		CvScalar s1, s2;
		int i, j;
		float val_prov2;
		float val_prov1;
		for( i =0; i < image_left->height; i++ )
		{
			for(j = 0; j < image_left->width; j++ )
			{
				s1 = cvGet2D(image_left, i, j);
				s2 = cvGet2D(render, i, j);
				val_prov2 = s1.val[0];
				val_prov1 = s2.val[2];
				s2.val[2] = val_prov1 + valor_resta_rojo;
				s1.val[0] = val_prov2 + valor_resta_azul;
				cvSet2D(image_left, i, j, s1);
				cvSet2D(render, i, j, s2);
			}
		}
		IplImage* g1 =
				cvCreateImage(cvGetSize(image_left),8,1);
		IplImage* g2 =
				cvCreateImage(cvGetSize(image_left),8,1);
		IplImage* swap2 =
				cvCreateImage(cvGetSize(image_left),8,1);
		cvCvtColor(image_left,swap2,CV_BGR2GRAY);
		cvexWarpShift(swap2,g1,0,0);
		cvCvtColor(render,swap2,CV_BGR2GRAY);
		cvexWarpShift(swap2,g2,0,0);
		cvMerge(g2,g2,g1,NULL,render_anaglyph);
		cvShowImage ("WEBCAM 3D. SELECCION DE MODO",
				render_anaglyph);
		key = cvWaitKey (0);
	}
	*_v=v-v_max/2;
	*_h=h-h_max/2;
	cvDestroyWindow("Rectificacion Manual");
	cvReleaseImage(&render);
}
void ShiftRectificationParameter(IplImage* image_left, IplImage* image_right,int *_v, int *_h, bool calibracionRGB, float valor_resta_rojo, float valor_resta_azul, int modeAnaglyph, int dis)
{
	int v_max = image_left->height/5;
	int h_max = image_left->width;
	cvNamedWindow ("Rectificacion Manual", CV_WINDOW_AUTOSIZE);
	int v=*_v+v_max/2;
	cvCreateTrackbar("v","Rectificacion Manual",&v,v_max,NULL);
	int h = *_h+h_max/2;
	cvCreateTrackbar("h","Rectificacion Manual",&h,h_max,NULL);
	IplImage* render = cvCloneImage(image_left);
	IplImage* render_entrelazada = cvCloneImage(image_left);
	IplImage* render_anaglyph = cvCloneImage(image_left);
	int key = 0;
	while (key !=1048603) //TECLA ESC
	{
		cvexWarpShift(image_right,render,h-h_max/2,v-v_max/2);
		cvexMakeStereoImageInterlace(image_left,render,render_entrelazada,0);//render->width/2);
		cvShowImage ("Rectificacion Manual",
				render_entrelazada);
		CvScalar s1, s2;
		int i, j;
		float val_prov2;
		float val_prov1;
		for( i =0; i < image_left->height; i++ )
		{
			for(j = 0; j < image_left->width; j++ )
			{
				s1 = cvGet2D(image_left, i, j);
				s2 = cvGet2D(render, i, j);
				val_prov2 = s1.val[0];
				val_prov1 = s2.val[2];
				s2.val[2] = val_prov1 + valor_resta_rojo;
				s1.val[0] = val_prov2 + valor_resta_azul;
				cvSet2D(image_left, i, j, s1);
				cvSet2D(render, i, j, s2);
			}
		}
		IplImage* g1 =
				cvCreateImage(cvGetSize(image_left),8,1);
		IplImage* g2 =
				cvCreateImage(cvGetSize(image_left),8,1);
		IplImage* swap2 =
				cvCreateImage(cvGetSize(image_left),8,1);
		cvCvtColor(image_left,swap2,CV_BGR2GRAY);
		cvexWarpShift(swap2,g1,0,0);
		cvCvtColor(render,swap2,CV_BGR2GRAY);
		cvexWarpShift(swap2,g2,0,0);
		cvMerge(g2,g2,g1,NULL,render_anaglyph);
		cvShowImage ("WEBCAM 3D. SELECCION DE MODO",
				render_anaglyph);
		key = cvWaitKey (0);
	}
	*_v=v-v_max/2;
	*_h=h-h_max/2;
	cvDestroyWindow("Rectificacion Manual");
	cvReleaseImage(&render);
}

void depth_estimation(IplImage* image_left, IplImage* image_right, IplImage* dest, int maxDisparity, int occ_penalty=15, int match_reward=3, int good_match=6,int middle_match=8, int bad_match=15)
{
	IplImage* depth = cvCreateImage(cvGetSize(image_left),8,1);
	IplImage* l = cvCreateImage(cvGetSize(image_left),8,1);
	IplImage* r = cvCreateImage(cvGetSize(image_left),8,1);
	cvCvtColor(image_left,l,CV_BGR2GRAY);
	cvCvtColor(image_right,r,CV_BGR2GRAY);
	cvFindStereoCorrespondence( l, r, CV_DISPARITY_BIRCHFIELD,
			depth, maxDisparity, occ_penalty, match_reward,good_match,
			middle_match, bad_match );
	cvCvtColor(depth,dest,CV_GRAY2BGR);
	cvScale(dest,dest,255/maxDisparity);
	cvReleaseImage(&depth);
	cvReleaseImage(&l);
	cvReleaseImage(&r);
}

void cvexMakeStereoImageSidebySide(IplImage* left, IplImage* right, IplImage* dest, int shift, int mode = CVEX_CONNECT_HORIZON)
{
	IplImage* swap = cvCreateImage(cvGetSize(left),8,left->nChannels);
	IplImage* temp = cvCreateImage(cvGetSize(left),8,left->nChannels);
	cvexWarpShift(left,swap,-shift,0);
	cvexWarpShift(right,temp,+shift,0);
	IplImage* connect = cvexConnect(swap,temp,mode);
	cvResize(connect,dest);
	cvReleaseImage(&connect);
	cvReleaseImage(&swap);
	cvReleaseImage(&temp);
}

void cvexSidebySideToAnaglyph(IplImage* dest, IplImage* left, IplImage* right, int shift)
{
	IplImage* swap;
	IplImage* temp;
	cvNamedWindow("IZQ DESPUES DE SIDEBYSIDE",
			CV_WINDOW_AUTOSIZE);
	cvNamedWindow("DRCH DESPUES DE SIDEBYSIDE",
			CV_WINDOW_AUTOSIZE);
	swap =cvCreateImage(cvSize(dest->width,MAX(dest->height,dest->height)),8,3);
	cvexWarpShift(dest,swap,-shift,0);
	temp =cvCreateImage(cvSize(dest->width,MAX(dest->height,dest->height)),8,3);
	cvexWarpShift(dest,temp,+shift,0);
	cvSetImageROI( swap, cvRect( 0, 0, dest->width/2, dest->height ) );
	cvSetImageROI( temp, cvRect( dest->width/2, 0, dest->width,
			dest->height ) );
	cvResize(swap,right);
	cvResize(temp,left);
	cvResetImageROI( swap );
	cvResetImageROI( temp );
	cvShowImage ("IZQ DESPUES DE SIDEBYSIDE", left);
	cvShowImage ("DRCH DESPUES DE SIDEBYSIDE", right);
}

void tiempo_real (IplImage* image_left, IplImage* image_right, IplImage* render, bool isRectification, bool &calibracionRGB, float valor_resta_rojo, float valor_resta_azul, int key, int mode, int	modeAnaglyph)
{
	int dis = image_left->width/2;
	CvexStereoCameraCalibration
	calib(cvGetSize(render),CHESS_SIZE,30);
	switch(mode)
	{
	case VIEW_LEFT:
		cvCopy(image_left,render);
		break;
	case VIEW_RIGHT:
		cvCopy(image_right,render);
		break;
	case VIEW_INTERLACE:
		cvexMakeStereoImageInterlace(image_left,image_right,render,dis-render->width/2);
		break;
	case VIEW_SIDEBYSIDE:
		cvexMakeStereoImageSidebySide(image_left,image_right,render,dis-render->width/2);
		break;
	case VIEW_DEPTHMAP:
		depth_estimation(image_left,image_right,render,100);
		break;
	default:
	case VIEW_ANAGLYPH:
		if (calibracionRGB == false)
		{
			calibracionRGB = true;
			RectificationRGB(image_left, image_right, render, dis-render->width/2, calibracionRGB, valor_resta_rojo, valor_resta_azul);
		}
		else
		{
			modifica_pixels(image_left, image_right, image_left, image_right, valor_resta_rojo, valor_resta_azul);
			cvexMakeStereoImageAnaglyph(image_left,image_right,render,dis-render->width/2,modeAnaglyph);
		}
		break;
	}
}

int main(void)
{
	mkdir("calibration_image",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	mkdir("capture",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	IplImage *image_left = 0;
	IplImage *image_right = 0;
	IplImage *image_stereo = 0;
	CvCapture *leftcam=0;
	CvCapture *rightcam=0;
	CvCapture *anaglifico=0;
	leftcam = cvCreateCameraCapture (0);
	rightcam = cvCreateCameraCapture (1);
	image_left = cvQueryFrame (leftcam);
	image_right = cvQueryFrame (rightcam);
	//IplImage* image_side =
	cvCreateImage(cvGetSize(1280,480),8,left->nChannels);
	IplImage* render = cvCloneImage(image_left);
	CvMat* hright = cvCreateMat(3,3,CV_64F);
	CvMat* hleft = cvCreateMat(3,3,CV_64F);
	cvSetIdentity(hleft);
	cvSetIdentity(hright);
	double w = IMAGE_WIDTH, h = IMAGE_HEIGHT;
	int ax, ay, lx, ly, rx, ry, key=0;
	int mode =0, pos_x=0, pos_y=0;
	int modeAnaglyph = 0;
	int dis = render->width/2;
	int v_shift=0;
	int h_shift=0;
	float valor_resta_rojo;
	float valor_resta_azul;
	double t_in = 0, t_out = 0, ms = 0;
	bool isRectification = false;
	bool calibracionRGB = false;
	bool isLRFlip = false;
	cvNamedWindow("WEBCAM 3D. SELECCION DE MODO", CV_WINDOW_AUTOSIZE);
	cvCreateTrackbar("Modo","WEBCAM 3D. SELECCION DE MODO",&mode,VIEW_DEPTHMAP, NULL);
	CvexStereoCameraCalibrationcalib(cvGetSize(render),CHESS_SIZE,30);
	while (key !=1048603) //ESC
	{
		//printf("%d\n", key);
		image_left = cvQueryFrame (leftcam);
		image_right = cvQueryFrame (rightcam);
		if (key == 1048690 && !isRectification) //TECLA r
		{
			ShiftRectificationParameter(image_left,image_right,&v_shift, &h_shift, calibracionRGB, valor_resta_rojo, valor_resta_azul, modeAnaglyph, dis);
		}
		if(key == 1048675) // TECLA c
		{
			bool is = calib.findChess(image_left,image_right);
			if(is)
			{
				cvexSaveImage(image_left,"calibration_image//left%03d.bmp",calib.getImageCount());
				cvexSaveImage(image_right,"calibration_image//right%03d.bmp",calib.getImageCount());
			}
			calib.drawChessboardCorners(image_left,image_left,CVEX_STEREO_CALIB_LEFT);
			calib.drawChessboardCorners(image_right,image_right,CVEX_STEREO_CALIB_RIGHT);
			if(is)
			{
				cvexSaveImage(image_left,"calibration_image//left%03d_.bmp",calib.getImageCount());
				cvexSaveImage(image_right,"calibration_image//right%03d_.bmp",calib.getImageCount());
			}
		}
		if(key == 1048688) //TECLA p
		{
			isRectification=true;
			calib.solveStereoParameter();
			calib.showIntrinsicParameters();
			calib.getRectificationMatrix(hleft,hright);
			calib.showRectificationHomography();
			calib.showExtrinsicParameters();
		}
		if(key == 1048697)//TECLA y
		{
			cvexSidebySideToAnaglyph(render, image_left, image_right, render->width/2);
		}
		if(key == 1114192 || key == 1179728)//TECLA P
		{
			isRectification=false;
		}
		if(key == 1048608)
		{
			if (modeAnaglyph ==0)
			{
				modeAnaglyph = 1;
			}
			else
			{
				modeAnaglyph = 0;
			}
		}
		if(key ==1048683) //TECLA k
		{
			cvSave ("HLEFT.xml",hleft);
			cvSave ("HRIGHT.xml",hright);
		}
		if(key == 1048684) //TECLA l
		{
			hleft = (CvMat*)cvLoad("HLEFT.xml");
			hright = (CvMat*)cvLoad("HRIGHT.xml");
			isRectification = true;
			/*cout<<"Left Rectification :"<<endl;
	cvexShowMatrix(hleft);
	cout<<"Right Rectification :"<<endl;
	cvexShowMatrix(hright);
			 */
		}
		if(key ==1048685) //TECLA m
		{
			calibracionRGB=false;
		}
		if(key ==1048691 ) //TECLA s
		{
			static int saveCount=0;
			cvexSaveImage(image_left,"capture//capture_left%03d.bmp",saveCount);
			cvexSaveImage(image_right,"capture//capture_right%03d.bmp",saveCount);
			cvWarpPerspective(image_left,render,hleft);
			cvexSaveImage(render,"capture//capture_left_warp%03d.bmp",saveCount);
			cvWarpPerspective(image_right,render,hright);
			cvexSaveImage(render,"capture//capture_right_warp%03d.bmp",saveCount++);
		}
		if(key==CVEX_KEY_ARROW_LEFT)
		{
			mode--;
			cvSetTrackbarPos("Modo","WEBCAM 3D. SELECCION DE MODO",mode);
		}
		if(key==CVEX_KEY_ARROW_RIGHT)
		{
			mode++;
			cvSetTrackbarPos("Modo","WEBCAM 3D. SELECCION DE MODO",mode);
		}
		if(isRectification)
		{
			cvWarpPerspective(image_left,render,hleft);
			cvCopy(render,image_left);
			cvWarpPerspective(image_right,render,hright);
			cvCopy(render,image_right);
			if (key == 1048690) //TECLA r
			{
				Move_image (image_left,image_right,&v_shift, &h_shift, calibracionRGB, valor_resta_rojo, valor_resta_azul, modeAnaglyph, dis);
			}
			//calib.rectifyImageRemap(image_left,image_left,CVEX_STEREO_CALIB_LEFT);
			//calib.rectifyImageRemap(image_right,image_right,CVEX_STEREO_CALIB_RIGHT);
			cvexWarpShift(image_right,image_right,h_shift,v_shift);
		}
		else
		{
			cvexWarpShift(image_right,image_right,h_shift,v_shift);
		}
		if(isLRFlip)
		{
			IplImage* swap;
			swap = image_left;
			image_left = image_right;
			image_right=swap;
		}
		if(key == 1048679) //TECLA g
		{
			int fps = 24; //Frames por segundo
			CvSize size = cvSize(640,480);
			CvVideoWriter* writer =
					cvCreateVideoWriter("Prueba1.avi", CV_FOURCC('X', 'V', 'I', 'D'), fps, size);
			if (writer == 0)
			{
				printf("No funciona el video\n");
			}
			while (key !=1048603)
			{
				if(key == 1048608)
				{
					if (modeAnaglyph ==0)
					{
						modeAnaglyph = 1;
					}
					else
					{
						modeAnaglyph = 0;
					}
				}
				image_left = cvQueryFrame (leftcam);
				image_right = cvQueryFrame (rightcam);
				if(isRectification)
				{
					cvWarpPerspective(image_left,render,hleft);
					cvCopy(render,image_left);
					cvWarpPerspective(image_right,render,hright);
					cvCopy(render,image_right);
					//calib.rectifyImageRemap(image_left,image_left,CVEX_STEREO_CALIB_LEFT);
					//calib.rectifyImageRemap(image_right,image_right,CVEX_STEREO_CALIB_RIGHT);
					cvexWarpShift(image_right,image_right,h_shift,v_shift);
				}
				else
				{
					cvexWarpShift(image_right,image_right,h_shift,v_shift);
				}
				tiempo_real (image_left, image_right, render, isRectification, calibracionRGB, valor_resta_rojo, valor_resta_azul, key, mode, modeAnaglyph);
				cvWriteFrame(writer,render);
				cvShowImage ("WEBCAM 3D. SELECCION DE MODO",
						render);
				key = cvWaitKey (10);
			}
			cvReleaseVideoWriter(&writer);
		}
		if(key ==1048696) //TECLA x
		{
			while (key !=1048603)
			{
				if(key == 1048608)
				{
					if (modeAnaglyph ==0)
					{
						modeAnaglyph = 1;
					}
					else
					{
						modeAnaglyph = 0;
					}
				}
				ms = 0;
				t_in = (double) cvGetTickCount();
				image_left = cvQueryFrame (leftcam);
				image_right = cvQueryFrame (rightcam);
				printf("Frame\n");
				if(isRectification)
				{
					cvWarpPerspective(image_left,render,hleft);
					cvCopy(render,image_left);
					cvWarpPerspective(image_right,render,hright);
					cvCopy(render,image_right);
					//calib.rectifyImageRemap(image_left,image_left,CVEX_STEREO_CALIB_LEFT);
					//calib.rectifyImageRemap(image_right,image_right,CVEX_STEREO_CALIB_RIGHT);
					cvexWarpShift(image_right,image_right,h_shift,v_shift);
				}
				else
				{
					cvexWarpShift(image_right,image_right,h_shift,v_shift);
				}
				while(ms <=1)
				{
					tiempo_real (image_left, image_right, render, isRectification, calibracionRGB, valor_resta_rojo, valor_resta_azul, key, mode, modeAnaglyph);
					t_out = (double) cvGetTickCount() - t_in;
					ms = t_out / (cvGetTickFrequency()*1000.0);
					printf("Modo: %d\n", mode);
					printf("Tiempo %f\n",ms);
					while(ms<=1)
					{
						t_out = (double) cvGetTickCount() - t_in;
						ms = t_out / (cvGetTickFrequency()*1000.0);
						//printf("Tiempo %f\n",ms);
					}
					//printf("Modo: %d\n", mode);
					//printf("Tiempo %f\n",ms);
				}
				cvShowImage ("WEBCAM 3D. SELECCION DE MODO",
						render);
				key = cvWaitKey (4);
			}
		}
		switch(mode)
		{
		case VIEW_LEFT:
			cvCopy(image_left,render);
			break;
		case VIEW_RIGHT:
			cvCopy(image_right,render);
			break;
		case VIEW_INTERLACE:
			cvexMakeStereoImageInterlace(image_left,image_right,render,dis-render->width/2);
			break;
		case VIEW_SIDEBYSIDE:
			cvexMakeStereoImageSidebySide(image_left,image_right,render,dis-render->width/2);
			break;
		case VIEW_DEPTHMAP:
			depth_estimation(image_left,image_right,render,100);
			break;
		default:
		case VIEW_ANAGLYPH:
			if (calibracionRGB == false)
			{
				calibracionRGB = true;
				RectificationRGB(image_left, image_right, render, dis-render->width/2, calibracionRGB, valor_resta_rojo, valor_resta_azul);
			}
			else
			{
				modifica_pixels(image_left, image_right, image_left, image_right, valor_resta_rojo, valor_resta_azul);
				cvexMakeStereoImageAnaglyph(image_left,image_right,render,dis-render->width/2,modeAnaglyph);
			}
			break;
		}
		cvShowImage ("WEBCAM 3D. SELECCION DE MODO", render);
		key = cvWaitKey(0);
	}
	cvReleaseMat (&hleft);
	cvReleaseMat (&hright);
	cvReleaseImage (&render);
	cvDestroyWindow("WEBCAM 3D. SELECCION DE MODO");
	cvReleaseImage(&image_left);
	cvReleaseImage(&image_right);
}
