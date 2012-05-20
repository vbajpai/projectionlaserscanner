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

#ifndef projectionlaserscanner_pointcloud_h
#define projectionlaserscanner_pointcloud_h

#include <opencv2/opencv.hpp>
using namespace std;

class Point3DRGB {
  
private:
	CvPoint3D32f point3D;
	int R;
	int G;
	int B;
  
public:
	Point3DRGB(CvPoint3D32f point3D, int B, int G, int R);
	
	CvPoint3D32f getPoint();
	int getR();
	int getG();
	int getB();
};
class PointCloud{
	
private:
	
	/* vector */
	vector <Point3DRGB*> pointCloud;
	
	/* camera parameters */
	CvMat* cameraMatrix;
	CvMat* leftRMatrix;
	CvMat* leftTVector;
	CvMat* rightRMatrix;
	CvMat* rightTVector;
	
	
	/* point vectors */
	vector<CvPoint> leftLaser;
	vector<CvPoint> object;
	vector<CvPoint> rightLaser;
	
	/* constants */
	static const int LEFT_SYSTEM = 1;
	static const int RIGHT_SYSTEM = 2;
	static const int ARBITRARY_LEFTPOINT_1 = 50;
	static const int ARBITRARY_LEFTPOINT_2 = 100;
	static const int ARBITRARY_RIGHTPOINT_1 = 50;
	
	
	
protected:
	
	CvMat* get3DPoint(CvPoint point2D, int flag, vector<double> *plane);
	
	vector <double> getPlaneEquation(CvMat* p1, CvMat* p2, CvMat* p3);
	
	CvMat* concatMatrix(CvMat* rMatrix, CvMat* tVector);
	
	void printMat(CvMat* mat);
  
public:
	
	PointCloud(vector <CvMat*> cameraParameters);
	
	void setSrc(vector < vector <CvPoint> > pointWrapper);
	
	void generate(IplImage* referenceImage);	
  
	vector <Point3DRGB*> getPointCloud();
};

#endif
