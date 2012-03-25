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

#include "pointcloud.h"

Point3DRGB::Point3DRGB(CvPoint3D32f point3D, int B, int G, int R){
		this->point3D = point3D;
		this->R = R;
		this->G = G;
		this->B = B;
	}	

CvPoint3D32f 
Point3DRGB::getPoint(){return point3D;}	

int 
Point3DRGB::getR(){return R;}

int 
Point3DRGB::getG(){return G;}

int 
Point3DRGB::getB(){return B;}  
  
  
PointCloud::PointCloud(vector <CvMat*> cameraParameters){
  
  cameraMatrix = cameraParameters[0];
  
  /* set rotation matrices */
  CvMat* rvecs = cameraParameters[1];
  
  CvMat* rvecLeft = cvCreateMat(1, 3, CV_32FC1);
  CvMat* rvecRight = cvCreateMat(1, 3, CV_32FC1);
  
  cvmSet(rvecLeft, 0, 0, cvmGet(rvecs, 0, 0));
  cvmSet(rvecLeft, 0, 1, cvmGet(rvecs, 0, 1));
  cvmSet(rvecLeft, 0, 2, cvmGet(rvecs, 0, 2));
  
  cvmSet(rvecRight, 0, 0, cvmGet(rvecs, 1, 0));
  cvmSet(rvecRight, 0, 1, cvmGet(rvecs, 1, 1));
  cvmSet(rvecRight, 0, 2, cvmGet(rvecs, 1, 2));
  
  leftRMatrix = cvCreateMat(3, 3, CV_32FC1);
  rightRMatrix = cvCreateMat(3, 3, CV_32FC1);
  
  cvRodrigues2(rvecLeft, leftRMatrix);
  cvRodrigues2(rvecRight, rightRMatrix);
  
  /* set translation vectors */
  CvMat* tvecs = cameraParameters[2];
  
  leftTVector = cvCreateMat(3, 1, CV_32FC1);
  rightTVector = cvCreateMat(3, 1, CV_32FC1);
  
  cvmSet(leftTVector, 0, 0, cvmGet(tvecs, 0, 0));
  cvmSet(leftTVector, 1, 0, cvmGet(tvecs, 0, 1));
  cvmSet(leftTVector, 2, 0, cvmGet(tvecs, 0, 2));
  
  cvmSet(rightTVector, 0, 0, cvmGet(tvecs, 1, 0));
  cvmSet(rightTVector, 1, 0, cvmGet(tvecs, 1, 1));
  cvmSet(rightTVector, 2, 0, cvmGet(tvecs, 1, 2));
  
  cvReleaseMat(&rvecLeft);
  cvReleaseMat(&rvecRight);
  cvReleaseMat(&rvecs);	
  cvReleaseMat(&tvecs);
}	

CvMat* 
PointCloud::get3DPoint(CvPoint point2D, int flag, vector<double> *plane){
		
		CvMat* point3D = cvCreateMat(3, 1, CV_32FC1);
		
		/* convert the point into homogeneous coordinates */
		CvMat* point = cvCreateMat(3, 1, CV_32FC1);
		cvmSet(point, 0, 0, point2D.x);
		cvmSet(point, 1, 0, point2D.y);
		cvmSet(point, 2, 0, 1.0);
		
		/* holders for matrix inverse */
		CvMat* aInverse = cvCreateMat(3, 3, CV_32FC1);
		CvMat* rInverse = cvCreateMat(3, 3, CV_32FC1);		
		
		/* holders for matrix multiplication */
		CvMat* rIaI = cvCreateMat(3, 3, CV_32FC1);
		CvMat* bVector = cvCreateMat(3, 1, CV_32FC1);
		CvMat* aVector = cvCreateMat(3, 1, CV_32FC1);						
		
    cvInvert(cameraMatrix, aInverse, CV_LU);
    
		if (flag == LEFT_SYSTEM){ 
			cvInvert(leftRMatrix, rInverse);		
			cvMatMul(rInverse, leftTVector, aVector);
		}
		if (flag == RIGHT_SYSTEM){ 
			cvInvert(rightRMatrix, rInverse);
			cvMatMul(rInverse, rightTVector, aVector);
		}
		
		cvMatMul(rInverse, aInverse, rIaI);
		cvMatMul(rIaI, point, bVector);
		
		/* compute scalar */
		
		double s = 0;		
		if(plane != NULL){
			
			/* scalar for object point */
			
			CvMat* nVector = cvCreateMat(3, 1, CV_32FC1);
			cvmSet(nVector, 0, 0, plane->at(0));
			cvmSet(nVector, 1, 0, plane->at(1));
			cvmSet(nVector, 2, 0, plane->at(2));
			
			double nVectordotaVector = cvDotProduct(nVector,aVector);
			double bVectordotnVector = cvDotProduct(bVector,nVector);
			
			s = ( nVectordotaVector - plane->at(3) ) / bVectordotnVector;	
      
			cvReleaseMat(&nVector);			
		}else{
			
			/* scalar for laser point */			
			s = cvmGet(aVector, 2, 0) / cvmGet(bVector, 2, 0);			
		}			
		cvmSet(bVector, 0, 0, cvmGet(bVector, 0, 0) * s);
		cvmSet(bVector, 1, 0, cvmGet(bVector, 1, 0) * s);
		cvmSet(bVector, 2, 0, cvmGet(bVector, 2, 0) * s);		
		
		/* get 3D point */		
		cvSub(bVector, aVector, point3D);
		
		/* transform right point to left coordinate system */
		if(flag == RIGHT_SYSTEM){
			
			/* convert point3D to homogenous coordinates */
			CvMat* point3DHomogenous = cvCreateMat(4, 1, CV_32FC1);
			cvmSet(point3DHomogenous, 0, 0, cvmGet(point3D, 0, 0));
			cvmSet(point3DHomogenous, 1, 0, cvmGet(point3D, 1, 0));
			cvmSet(point3DHomogenous, 2, 0, cvmGet(point3D, 2, 0));
			cvmSet(point3DHomogenous, 3, 0, 1.0);
			
			/* multiply [R2|T2] with point3DHomogenous */
			CvMat* rt = concatMatrix(rightRMatrix, rightTVector);
			CvMat* rtPointHomogeneous = cvCreateMat(3, 1, CV_32FC1);
			cvMatMul(rt, point3DHomogenous, rtPointHomogeneous);
      
			/* multiply with rInverse */
			CvMat* rIpHomogeneous = cvCreateMat(3, 1, CV_32FC1);			
			cvMatMul(rInverse, rtPointHomogeneous, rIpHomogeneous);
			cvSub(rIpHomogeneous, aVector, point3D);		
			
			cvReleaseMat(&rt);
			cvReleaseMat(&point3DHomogenous);
			cvReleaseMat(&rIpHomogeneous);						
		}
		
		/* release resources */
		cvReleaseMat(&point);		
		cvReleaseMat(&aInverse);
		cvReleaseMat(&rInverse);
		cvReleaseMat(&rIaI);
		cvReleaseMat(&bVector);
		cvReleaseMat(&aVector);
		
		return point3D;
	}	

vector <double> 
PointCloud::getPlaneEquation(CvMat* p1, CvMat* p2, CvMat* p3){
		
		CvMat* Dmatrix = cvCreateMat(3, 3, CV_32FC1);
		
		cvmSet(Dmatrix, 0, 0, cvmGet(p1, 0, 0));
		cvmSet(Dmatrix, 1, 0, cvmGet(p2, 0, 0));
		cvmSet(Dmatrix, 2, 0, cvmGet(p3, 0, 0));
		
		cvmSet(Dmatrix, 0, 1, cvmGet(p1, 1, 0));
		cvmSet(Dmatrix, 1, 1, cvmGet(p2, 1, 0));
		cvmSet(Dmatrix, 2, 1, cvmGet(p3, 1, 0));
		
		cvmSet(Dmatrix, 0, 2, cvmGet(p1, 2, 0));
		cvmSet(Dmatrix, 1, 2, cvmGet(p2, 2, 0));
		cvmSet(Dmatrix, 2, 2, cvmGet(p3, 2, 0));
		
		/* calculate D */
		double Dvalue = (-1) * cvDet(Dmatrix);
		
		/* calculate A */
		CvMat* Amatrix = cvCloneMat(Dmatrix);
		cvmSet(Amatrix, 0, 0, 1.0);
		cvmSet(Amatrix, 1, 0, 1.0);
		cvmSet(Amatrix, 2, 0, 1.0);		
		double Avalue = cvDet(Amatrix);		
		
		/* calculate B */
		CvMat* Bmatrix = cvCloneMat(Dmatrix);
		cvmSet(Bmatrix, 0, 1, 1.0);
		cvmSet(Bmatrix, 1, 1, 1.0);
		cvmSet(Bmatrix, 2, 1, 1.0);		
		double Bvalue = cvDet(Bmatrix);		
		
		/* calculate C */
		CvMat* Cmatrix = cvCloneMat(Dmatrix);
		cvmSet(Cmatrix, 0, 2, 1.0);
		cvmSet(Cmatrix, 1, 2, 1.0);
		cvmSet(Cmatrix, 2, 2, 1.0);		
		double Cvalue = cvDet(Cmatrix);				
		
		/* release resources */
		cvReleaseMat(&Amatrix);
		cvReleaseMat(&Bmatrix);
		cvReleaseMat(&Cmatrix);
		cvReleaseMat(&Dmatrix);
    
		/* wrap in a vector */
		double planeArray[] = {Avalue, Bvalue, Cvalue, Dvalue};
		vector<double> plane (planeArray, planeArray + sizeof(planeArray) / sizeof(double));		
		
		/* return vector */
		return plane;
	}

CvMat* 
PointCloud::concatMatrix(CvMat* rMatrix, CvMat* tVector){
		
		CvMat* rt = cvCreateMat(3, 4, CV_32FC1);
		cvmSet(rt, 0, 0, cvmGet(rMatrix, 0, 0));
		cvmSet(rt, 1, 0, cvmGet(rMatrix, 1, 0));
		cvmSet(rt, 2, 0, cvmGet(rMatrix, 2, 0));
		cvmSet(rt, 0, 1, cvmGet(rMatrix, 0, 1));
		cvmSet(rt, 1, 1, cvmGet(rMatrix, 1, 1));
		cvmSet(rt, 2, 1, cvmGet(rMatrix, 2, 1));
		cvmSet(rt, 0, 2, cvmGet(rMatrix, 0, 2));
		cvmSet(rt, 1, 2, cvmGet(rMatrix, 1, 2));
		cvmSet(rt, 2, 2, cvmGet(rMatrix, 2, 2));
		cvmSet(rt, 0, 3, cvmGet(tVector, 0, 0));
		cvmSet(rt, 1, 3, cvmGet(tVector, 1, 0));
		cvmSet(rt, 2, 3, cvmGet(tVector, 2, 0));
    
		return rt;
	}

void 
PointCloud::printMat(CvMat* mat){
		cout << cvmGet(mat, 0, 0) << endl;
		cout << cvmGet(mat, 1, 0) << endl;
		cout << cvmGet(mat, 2, 0) << endl;
	}  

void 
PointCloud::setSrc(vector < vector <CvPoint> > pointWrapper){
		leftLaser = pointWrapper[0];
		object = pointWrapper[1];
		rightLaser = pointWrapper[2];
	}	

void 
PointCloud::generate(IplImage* referenceImage){
		
		CvPoint leftLaserPoint1_2D = leftLaser[ARBITRARY_LEFTPOINT_1];
		CvPoint leftLaserPoint2_2D = leftLaser[ARBITRARY_LEFTPOINT_2];
		CvPoint rightLaserPoint_2D = rightLaser[ARBITRARY_RIGHTPOINT_1];
		
		/* get 3D points in left coordinate system */
		CvMat* leftLaserPoint1_3D = get3DPoint(leftLaserPoint1_2D, LEFT_SYSTEM, NULL);
		CvMat* leftLaserPoint2_3D = get3DPoint(leftLaserPoint2_2D, LEFT_SYSTEM, NULL);
		CvMat* rightLaserPoint_3D = get3DPoint(rightLaserPoint_2D, RIGHT_SYSTEM, NULL);
		
		/* get laser plane equation */	
		vector <double> plane = getPlaneEquation(leftLaserPoint1_3D, leftLaserPoint2_3D, rightLaserPoint_3D);
		
		/* get 3D points of the object */
		
		
		for (int i=0; i < object.size(); i++) {
			
			CvMat* objectPoint_3D = get3DPoint(object[i], LEFT_SYSTEM, &plane);
			double x = cvmGet(objectPoint_3D, 0, 0);
			double y = cvmGet(objectPoint_3D, 1, 0);
			double z = cvmGet(objectPoint_3D, 2, 0);
			
			CvScalar s = cvGet2D(referenceImage, object[i].x, object[i].y);					
			pointCloud.push_back(new Point3DRGB(cvPoint3D32f(x, y, z), s.val[0], s.val[1], s.val[2]));
			
			cvReleaseMat(&objectPoint_3D);
		}	
		
		/* release resources */
		cvReleaseMat(&leftLaserPoint1_3D);
		cvReleaseMat(&leftLaserPoint2_3D);
		cvReleaseMat(&rightLaserPoint_3D);		
	}	

vector <Point3DRGB*> 
PointCloud::getPointCloud(){return pointCloud;}