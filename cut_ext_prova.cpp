/* 
	jpeg2ngc_cut							
	version: 0.1								
	author: Stefano Zamuner		
	date: 	 25/12/2010					
	description: it calculates the optimal drill-path to cut (externally) the input mask						
*/

#ifdef _CH_
#pragma package <opencv>
#endif

#define CV_NO_BACKWARD_COMPATIBILITY

#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
using namespace cv;
#endif

fstream file;
int dil_img_rows;

double PIX_MM;
double DRILL_MM;
double PRECISION=0.05;
double DEEP=0.0;
double SAFE_HEIGHT=2.0;
double G1_XY_SPEED=400.0;
double G1_Z_SPEED=100.0;

void init_variables(){
	cout<<"mm per pixel (x): ";
	cin>>PIX_MM;
	cout<<"drill diameter (mm): ";
	cin>>DRILL_MM;
	cout<<"deepness (mm): ";
	cin>>DEEP;
	cout<<"safe height (mm): ";
	cin>>SAFE_HEIGHT;
	cout<<"x-y feed rate (mm/min): ";
	cin>>G1_XY_SPEED;
	cout<<"z feed rate (mm/min): ";
	cin>>G1_Z_SPEED;
}

Mat dilate_image(Mat img){
	double pix_in_drill= DRILL_MM/PIX_MM;
	int scale=1;
	double intpart;
	double fractpart=modf (pix_in_drill , &intpart);
	Mat	mat_src=img;
	Mat mat_res;
	Mat mat_dil;
	Mat drill_element;

	while(fractpart>PRECISION & fractpart<(1.-PRECISION)){
		scale++;																						// fattore di scala per il resize
		PIX_MM=PIX_MM*(double)(scale-1)/(double)scale;												// numero di volte da dilatare
		pix_in_drill= DRILL_MM/PIX_MM;
		fractpart = modf (pix_in_drill , &intpart);
		if(fractpart>=0.5){
			intpart++;
		}
	}
	
	drill_element=getStructuringElement(MORPH_ELLIPSE, Size(intpart,intpart), Point(-1, -1));

	cout<<"L'immagine verra' riscalata di un fattore "<<scale<<endl;
	cout<<"la punta e' composta da "<<intpart<<" pixel."<<endl;

	resize(mat_src, mat_res , Size(), scale, scale, INTER_AREA);
	dilate(mat_res, mat_dil, drill_element, Point(-1, -1), 1, BORDER_TRANSPARENT, morphologyDefaultBorderValue());
	return mat_dil;
}

void write_circuit( CvSeq* d )	
{
	int total=d->total;
	double first_x;
	double first_y;
	for(int i=0;i<total;i++){
		CvPoint* p = CV_GET_SEQ_ELEM(CvPoint, d, i);
		if(i==0){
			first_x=double(p->x)*PIX_MM;
			first_y=((double)dil_img_rows-double(p->y))*PIX_MM;
			file<<"g0 x"<< first_x <<" y"<< first_y <<endl;
			file<<"g1 f"<<G1_Z_SPEED<<" z-"<<DEEP<<"\n";
		}
		if(i==1){
			file<<"g1 f"<<G1_XY_SPEED<<" x"<< double(p->x)*PIX_MM <<" y"<< ((double)dil_img_rows-double(p->y))*PIX_MM <<endl;
		}
		if(i>1){
			file<<"x"<< double(p->x)*PIX_MM <<"\t"<<" y"<<((double)dil_img_rows-double(p->y))*PIX_MM<<endl;
		}
		file<<"x"<< first_x <<"\t"<<" y"<<first_y<<endl;
		file<<"g0 z"<<SAFE_HEIGHT<<"\n";
	}
}

void contour_iterator(CvSeq* contour)		//itera tutti i contorni trovati dando la precedenza ai contorni più interni
{
	CvSeq* c=contour;
	int height=0;
	while(c->v_next!=NULL){
		c=c->v_next;
		height++;
	}
	for(int i=0;i<=height;i++){
		if(c->h_next!=NULL){
			contour_iterator(c->h_next);
			write_circuit(c);
		}
		if(c->h_next==NULL){
			write_circuit(c);
		}
	c=c->v_prev;
	}
}

void write_gcode(CvSeq* contours)
{
	file<<"g21\n";											//set unit to mm
	file<<"g0 z"<<SAFE_HEIGHT<<"\n";		//go to SAFE_HEIGHT
	contour_iterator(contours);					//iter on whole contours
	file<<"x0 y0"<<endl;								//return home
	file<<"z0"<<endl;										//return home
	file<<"M02\n";											//end gcode
}

int main( int argc, char** argv )
{
	char* filename = argv[1];		// image filename
	char ngc_filename[strlen(argv[1])];														// output filename
	for(int k = 0; k< strlen(argv[1])-4; k++){										// find basename: 'basename.jpg' --> 'basename'
		ngc_filename[k]=(char)argv[1][k];
	}
	ngc_filename[strlen(argv[1])-4]='.';
	ngc_filename[strlen(argv[1])-3]='n';
	ngc_filename[strlen(argv[1])-2]='g';
	ngc_filename[strlen(argv[1])-1]='c';
	ngc_filename[strlen(argv[1])]='\0';														// 'basename' --> 'basename.ngc'
	Mat img;																								// image
	Mat mat_dilated;																							// dilated image
	Mat mat_thres;																								// threshold image
	CvMemStorage* storage = cvCreateMemStorage(0);								// storage
	CvSeq* contours = 0;																					// sequence of points of contours
	
	init_variables();

	file.open(ngc_filename, ios::out);														// open output ngc file

	cout<<"filename: "<<filename<<endl;
	img = imread(filename,0);

	mat_dilated=dilate_image(img);																//dilate input image
	mat_thres=mat_dilated;											
	
	dil_img_rows=mat_dilated.rows;																//save number of rows in dilated image
	threshold(mat_dilated, mat_thres, 200.0, 255.0,THRESH_BINARY );	//threshold dilated image

	IplImage img_dilated=mat_dilated;															//convert to IplImage
	IplImage img_thres=mat_thres;																	//convert to IplImage

	cvFindContours( &img_thres, storage, &contours, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );																												//find contours

	write_gcode(contours);																				//write gcode

	return 0;
}

#ifdef _EiC
main(1,"");
#endif
