/* 
	jpeg2dxf							
	version: 0.1								
	author: Stefano Zamuner		
	date: 	 25/12/2010					
	description: it creates the CAD dxf file from a set of jpg images						
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
#include <sstream>
#include <string>
using namespace std;
using namespace cv;
#endif

fstream file;
int img_rows;
int position=0;
int v_space=3;

double PIX_MM;
string DXF_FILENAME="";

void init_variables(char* filename,bool Control){
	if(Control==false){
		cout<<"Project name\n";
		getline(cin, DXF_FILENAME);
	}
	if(Control==true){
		cout<<"image: "<<filename<<endl;
		cout<<"mm per pixel (x): ";
		cin>>PIX_MM;
		cout<<endl;
	}
}

void write_circuit( CvSeq* d )	
{
	int total=d->total;
	file<<"\t0\n";
	file<<"LWPOLYLINE\n";
	file<<"100\n";
	file<<"AcDbEntity\n";
	file<<"100\n";
	file<<"AcDbPolyline\n";
	file<<"\t90\n";
	file<<total<<endl;;
	file<<"\t70\n";
	file<<"1\n";
	for(int i=0;i<total;i++){
		CvPoint* p = CV_GET_SEQ_ELEM(CvPoint, d, i);
		file<<"\t10\n";
		file<<PIX_MM*double(p->x)<<endl;
		file<<"\t20\n";
		file<<PIX_MM*(double(position) + double(v_space) + double(img_rows)-double(p->y))<<endl;
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

int main( int argc, char** argv )
{
	Mat img;																											// image
	Mat mat_thres;																								// threshold image
	CvMemStorage* storage = cvCreateMemStorage(0);								// storage
	CvSeq* contours = 0;																					// sequence of points of contours
	char* test_image=(char*)"";
	init_variables(test_image,false);

	file.open(&DXF_FILENAME[0], ios::out);														// open output dxf file
	file<<"\t0\n";											//
	file<<"SECTION\n";
	file<<"\t2\n";
	file<<"ENTITIES\n";

	for(int j=1;j<argc;j++){
		char* image_filename=argv[j];
		init_variables(image_filename,true);
		img = imread(image_filename,0);
		img_rows=img.rows;
		threshold(img, mat_thres, 200.0, 255.0,THRESH_BINARY );	//threshold dilated image
		IplImage iplthr= mat_thres;															//convert to IplImage
		cvFindContours( &iplthr, storage, &contours, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );																												//find contours
		contour_iterator(contours);																				//write gcode
		position+=img_rows;
	}								

	file<<"\t0\n";
	file<<"EOF";
	return 0;
}

#ifdef _EiC
main(1,"");
#endif
