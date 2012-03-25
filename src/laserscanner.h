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

#ifndef projectionlaserscanner_laserscanner_h
#define projectionlaserscanner_laserscanner_h

/* c++ standard library */
#include <iostream>
#include <iomanip>
#include <fstream>

/* opencv library */
#include <opencv2/opencv.hpp>

/* posix library */
#include <dirent.h>

/* laser scanner */
#include "calibrate.h"
#include "transforms.h"
#include "pointcloud.h"

using namespace std;

class LaserScanner{
	
private:
	
	/* constants */
	static const int HOUGH_THRESH = 70;			//100
	static const int HOUGH_PARAM1 = 1;			//700
	static const int HOUGH_PARAM2 = 600;		//600
	
	static const int IMAGE_INDEX_START = 1; 
	static const int IMAGE_INDEX_END = 250;
	
	static const int OBJECT_THRESH = 100;
	static const int LASER_THRESH = 250;
	
	static const int IMAGE_FILE = 0;
	static const int POINTCLOUD_FILE = 1;
	
	static const string POINT_CLOUD_FILENAME;
	
	/* variables */	
	IplImage* src;	
	IplImage* referenceImage;
	vector<CvMat*> cameraParameters;
	
	
	string preparePath(string fileName, string dirName);
	
	void savePointCloud(vector<Point3DRGB*> pointCloud, string destinationDir);
	
	void saveImage(IplImage* img, string filename, string destinationDir);
	
	string prepareFilename(string dirName, int imageIndex, int flag);
	
	IplImage* takeDifferenceImage();
	
	void searchBrightestPixel(IplImage* differenceImage);
	
	vector < vector <CvPoint> > saveInVectors(IplImage* finalImage);
	
public:
	
	LaserScanner();
	
	LaserScanner(vector<CvMat*> param);
	
	int scanImages(int numberOfImages, 
                 string dirName, 
                 string referenceImageLocation, 
                 string destinationDir);
	
};

#endif
