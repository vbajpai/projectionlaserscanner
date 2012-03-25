/*
 * Copyright (c) 2010, Vaibhav Bajpai <contact@vaibhavbajpai.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are 
 * those of the authors and should not be interpreted as representing official 
 * policies, either expressed or implied, of the FreeBSD Project.
 */

#include "calibrate.h"

CalibrateCamera::CalibrateCamera(){
  numberOfCorners = WIDTH * HEIGHT;
  corners = new CvPoint2D32f[numberOfCorners];		
}

CalibrateCamera::~CalibrateCamera(){
  delete corners;
}
	
void 
CalibrateCamera::allocateMatrixObjects(){
		
		objectPointsOld = cvCreateMat(numberOfCorners * numberOfImages, 3, CV_32FC1 );
		imagePointsOld = cvCreateMat(numberOfCorners * numberOfImages, 2, CV_32FC1 );
		pointCountsOld = cvCreateMat(numberOfImages, 1, CV_32SC1 );		
		cameraMatrix = cvCreateMat( 3, 3, CV_32FC1 );
		rMatrix = cvCreateMat( 3, 3, CV_32FC1 );
		distCoeffs = cvCreateMat( 5, 1, CV_32FC1 );	
	}
	
void 
CalibrateCamera::findChessboardCorner(){	
		
		for (int imageIndex=0; imageIndex<numberOfImages; imageIndex++) {
			
			/* load the image from filename */
			filename = imageList[imageIndex];						
			img = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_UNCHANGED);	
			
			/* search chessboard corners */
			int ifFound = cvFindChessboardCorners(img, cvSize(WIDTH, HEIGHT), corners, &cvFindCornerCount);	
			
			if(ifFound)
				cout << "[filename:"<< filename << "]" << "[corners:" << cvFindCornerCount << "]"<< endl;
			else
				cout << "[filename:"<< filename << "]" << "[corners: 0]" << endl;
			
			/* fill structures */		
			if(cvFindCornerCount == numberOfCorners){				
				CV_MAT_ELEM(*pointCountsOld, int, numberOfSuccessfulImages, 0) = numberOfCorners;				
				int step = numberOfSuccessfulImages*numberOfCorners;
				for( int i=step, j=0; j < numberOfCorners; ++i, ++j ){				
					CV_MAT_ELEM( *imagePointsOld, float, i, 0 ) = corners[j].x;
					CV_MAT_ELEM( *imagePointsOld, float, i, 1 ) = corners[j].y;
					CV_MAT_ELEM( *objectPointsOld, float, i, 0 ) = j/WIDTH;
					CV_MAT_ELEM( *objectPointsOld, float, i, 1 ) = j%WIDTH;
					CV_MAT_ELEM( *objectPointsOld, float, i, 2 ) = 0.0f;
				}	
				numberOfSuccessfulImages++;
			}	
		}	
	}
	
void 
CalibrateCamera::calibrateCamera(){
		
		if (numberOfSuccessfulImages != 0){
			
			/* reallocate the matrix objects */
			objectPointsNew = cvCreateMat(numberOfSuccessfulImages*numberOfCorners, 3, CV_32FC1 );
			imagePointsNew = cvCreateMat(numberOfSuccessfulImages*numberOfCorners, 2, CV_32FC1 );
			pointCountsNew = cvCreateMat(numberOfSuccessfulImages, 1, CV_32SC1 );				
			reAllocateMatrixObjects();	
			
			/* calibrate the camera */
			rvecs = cvCreateMat(numberOfSuccessfulImages, 3, CV_32FC1);
			tvecs = cvCreateMat(numberOfSuccessfulImages, 3, CV_32FC1);	
			cout << endl << endl << "running cvCalibrateCamera2() ..." << endl << endl;
			cvCalibrateCamera2(objectPointsNew, imagePointsNew, pointCountsNew, cvGetSize(img), cameraMatrix, distCoeffs, rvecs, tvecs);
			
			/* print intrinsic | extrinsic parameters */
			cout << endl << endl << "intrinsic matrix ..." << endl << endl;
			printParameters();				
      
		}else{	
			cout << endl << endl << "cannot calibrate the camera" << endl << endl;
		}
	}
	
void 
CalibrateCamera::reAllocateMatrixObjects(){
		
		// transfer to new matrix
		for( int i = 0; i < numberOfSuccessfulImages*numberOfCorners; ++i ){		
			CV_MAT_ELEM( *imagePointsNew, float, i, 0) = cvmGet(imagePointsOld, i, 0);
			CV_MAT_ELEM( *imagePointsNew, float, i, 1) = cvmGet(imagePointsOld, i, 1);
			CV_MAT_ELEM( *objectPointsNew, float, i, 0) = cvmGet(objectPointsOld, i, 0);
			CV_MAT_ELEM( *objectPointsNew, float, i, 1) = cvmGet(objectPointsOld, i, 1);
			CV_MAT_ELEM( *objectPointsNew, float, i, 2) = cvmGet(objectPointsOld, i, 2);
		}	
		for( int i=0; i < numberOfSuccessfulImages; ++i ){
			CV_MAT_ELEM(*pointCountsNew, int, i, 0) = CV_MAT_ELEM(*pointCountsOld, int, i, 0); 
		}
		
		/* release old matrix objects */
		cvReleaseMat(&imagePointsOld);
		cvReleaseMat(&objectPointsOld);
		cvReleaseMat(&pointCountsOld);
	}
	
void 
CalibrateCamera::printParameters(){
		
		if (cameraMatrix){			
			cout << "camera matrix = " << endl << endl;
			for (int i=0; i < cameraMatrix->rows; i++) {
				for (int j=0; j < cameraMatrix->cols; j++) {
					if (j!=0) 
						cout << setw(20);					
					cout << cvGetReal2D(cameraMatrix, i, j);
				}				
				cout << endl;
			}						
		}
		
		// print translation matrix
		if (tvecs){
			cout << endl << endl << "translation matrix - " << endl << endl;
			for (int j = 0; j < numberOfSuccessfulImages; j++){
				for (int i = 0; i < 3; i++){
					if(i!=0)
						cout << setw(20);		
					cout << cvmGet(tvecs, j, i);
				}
				cout << endl;
			}		
		}
		
		// print rotation matrix
		if (rvecs){
			cout << endl << endl << "rotation matrix - " << endl << endl;
			for (int j = 0; j < numberOfSuccessfulImages; j++){
				for (int i = 0; i < 3; i++){
					if(i!=0)
						cout << setw(20);			
					cout << cvmGet(rvecs, j, i);	
				}
				cout << endl;
			}		
		}	
	}
	
vector<CvMat*> 
CalibrateCamera::calibrateFromImages(vector<string> imageList){
		
		/* allocate matrix objects */
		this->imageList = imageList;
		numberOfImages = imageList.size();
		allocateMatrixObjects();
		
		/* findChessboardCorners */
		cout << endl << endl << "running cvFindChessboardCorners() ..." << endl << endl;
		findChessboardCorner();
		
		/* calibrate camera */
		calibrateCamera();			
		
		/* release resources */
		cvReleaseMat(&objectPointsNew);
		cvReleaseMat(&imagePointsNew);
		cvReleaseMat(&pointCountsNew);		
		cvReleaseMat(&distCoeffs);
		cvReleaseMat(&rMatrix);
		cvReleaseImage(&img);
		
		/* wrap parameters in a vector */
		vector<CvMat*> cameraParameters;
		cameraParameters.push_back(cameraMatrix);
		cameraParameters.push_back(rvecs);
		cameraParameters.push_back(tvecs);
		
		/* return cameraParameters */
		return cameraParameters;
	}