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

#ifndef projectionlaserscanner_calibrate_h
#define projectionlaserscanner_calibrate_h

#include <opencv2/opencv.hpp>
#include <iomanip>
#include <iostream>

using namespace std;

class CalibrateCamera{
	
private:
	
	/* constants */
	static const int WIDTH = 6;
	static const int HEIGHT = 4;
	
	/* variables */
	int numberOfCorners, cvFindCornerCount; 
  int numberOfImages, numberOfSuccessfulImages;
	CvPoint2D32f* corners; 
	IplImage *img, *dest;
  string filename;	
	vector<string> imageList;	
	CvMat* objectPointsOld, *imagePointsOld, *pointCountsOld;
	CvMat* objectPointsNew, *imagePointsNew, *pointCountsNew;
	CvMat* cameraMatrix, *distCoeffs, *rvecs, *tvecs, *rMatrix;
	
	void 
  allocateMatrixObjects();
	
	void 
  findChessboardCorner();
	
	void 
  calibrateCamera();
	
	void 
  reAllocateMatrixObjects();
	
	void 
  printParameters();
	
public:
	
	CalibrateCamera();
	
	~CalibrateCamera();
	
	vector<CvMat*> 
  calibrateFromImages(vector<string> imageList);		
  
};

#endif
