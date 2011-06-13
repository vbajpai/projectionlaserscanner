/* c++ standard library */
#include <iostream>
#include <iomanip>
#include <fstream>

/* opencv library */
#include <cv.h>
#include <highgui.h>

/* posix library */
#include <dirent.h>

using namespace std;

class Point3DRGB{

private:
	CvPoint3D32f point3D;
	int R;
	int G;
	int B;

public:
	Point3DRGB(CvPoint3D32f point3D, int B, int G, int R){
		this->point3D = point3D;
		this->R = R;
		this->G = G;
		this->B = B;
	}
	
	CvPoint3D32f getPoint(){return point3D;}	
	int getR(){return R;}
	int getG(){return G;}
	int getB(){return B;}	
};

class CalibrateCamera{
	
private:
	
	/* constants */
	static const int WIDTH = 6;
	static const int HEIGHT = 4;
	
	/* variables */
	int numberOfCorners, cvFindCornerCount, numberOfImages, numberOfSuccessfulImages;	
	CvPoint2D32f* corners; 
	IplImage *img, *dest;
	string filename;	
	vector<string> imageList;	
	CvMat* objectPointsOld, *imagePointsOld, *pointCountsOld;
	CvMat* objectPointsNew, *imagePointsNew, *pointCountsNew;
	CvMat* cameraMatrix, *distCoeffs, *rvecs, *tvecs, *rMatrix;
	
	void allocateMatrixObjects(){
		
		objectPointsOld = cvCreateMat(numberOfCorners * numberOfImages, 3, CV_32FC1 );
		imagePointsOld = cvCreateMat(numberOfCorners * numberOfImages, 2, CV_32FC1 );
		pointCountsOld = cvCreateMat(numberOfImages, 1, CV_32SC1 );		
		cameraMatrix = cvCreateMat( 3, 3, CV_32FC1 );
		rMatrix = cvCreateMat( 3, 3, CV_32FC1 );
		distCoeffs = cvCreateMat( 5, 1, CV_32FC1 );	
	}
	
	void findChessboardCorner(){	
		
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
	
	void calibrateCamera(){
		
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
	
	void reAllocateMatrixObjects(){
		
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
	
	void printParameters(){
		
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
	
public:
	
	CalibrateCamera(){
		numberOfCorners = WIDTH * HEIGHT;
		corners = new CvPoint2D32f[numberOfCorners];		
	}
	
	~CalibrateCamera(){
		delete corners;
	}
	
	vector<CvMat*> calibrateFromImages(vector<string> imageList){
		
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

};

class Transforms{
	
private:
	
	/* constants */
	static const int CANNY_LOWTHRESH = 80;
	static const int CANNY_HIGHTHRESH = 160;
	
	/* variables */
	IplImage* src;
	IplImage* dst;
	
public:
	
	Transforms(IplImage* src){this->src = src;}
	
	Transforms(){}	
	
	IplImage* doGauss(){
		dst = cvCloneImage(src);
		cvSmooth(src, dst, CV_GAUSSIAN, 5, 5, 0, 0);
		return dst;
	}
	
	IplImage* doCanny(int lowThresh, int highThresh, int appertureSize = 3){
		
		/* convert to gray scale */
		IplImage* dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
		cvCvtColor(src, dst, CV_RGB2GRAY);              
		cvCanny(dst, dst, lowThresh, highThresh, 3);
		return dst;
	}
	
	IplImage* doHough(int houghThresh, int houghParam1, int houghParam2){
		
		/* perform canny edge detection */
		IplImage* cannyImage = doCanny(CANNY_LOWTHRESH, CANNY_HIGHTHRESH);		
		
		/* create a memory storage */
		CvMemStorage *storage = cvCreateMemStorage(0);          
		
		/* returns all lines found */
		CvSeq* line = cvHoughLines2(cannyImage, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180,
									houghThresh, houghParam1, houghParam2);
		
		for (int i=0; i<line->total; i++) {
			
			CvPoint* lineEndPoints = (CvPoint*) cvGetSeqElem(line,i);						
			cvLine(src, lineEndPoints[0], lineEndPoints[1], CV_RGB(255, 0, 0), 5);
		}  
		
		return src;
	}	
	
	void setSrc(IplImage* src){this->src = src;}
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
	
	CvMat* get3DPoint(CvPoint point2D, int flag, vector<double> *plane){
		
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
	
	vector <double> getPlaneEquation(CvMat* p1, CvMat* p2, CvMat* p3){
		
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
	
	CvMat* concatMatrix(CvMat* rMatrix, CvMat* tVector){
		
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
	
	void printMat(CvMat* mat){
		cout << cvmGet(mat, 0, 0) << endl;
		cout << cvmGet(mat, 1, 0) << endl;
		cout << cvmGet(mat, 2, 0) << endl;
	}

public:
	
	PointCloud(vector <CvMat*> cameraParameters){
		
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
	
	void setSrc(vector < vector <CvPoint> > pointWrapper){
		leftLaser = pointWrapper[0];
		object = pointWrapper[1];
		rightLaser = pointWrapper[2];
	}
	
	void generate(IplImage* referenceImage){
		
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
	
	vector <Point3DRGB*> getPointCloud(){return pointCloud;}	
};

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
	
	
	string preparePath(string fileName, string dirName){
		string path;
		
		/* add a slash at the end of directory if not supplied */
		if (*(dirName.rbegin()) != '/')			
			dirName.append("/");	
		
		path = dirName + fileName;
		return path;
	}	
	
	void savePointCloud(vector<Point3DRGB*> pointCloud, string destinationDir){
		
		string file3D = "/scan000.3d";
		string fileFrames = "/scan000.frames";
		string filePose = "/scan000.pose";
		
		ofstream outFile3D(preparePath(file3D, destinationDir).c_str());
		ofstream outFileFrames(preparePath(fileFrames, destinationDir).c_str());
		ofstream outFilePose(preparePath(filePose, destinationDir).c_str());
		
		outFile3D << pointCloud.size() << "\n";		
		
		for (int i=0; i < pointCloud.size(); i++) {
			
			Point3DRGB *pointObject = pointCloud[i];			
			CvPoint3D32f point = pointObject->getPoint();
			outFile3D << point.x << " " << point.z << " " << point.y << " " << 
						 pointObject->getR() << " " << pointObject->getG() << " " << pointObject->getB() << "\n";
			
			delete(pointObject);
		}	
		
		outFilePose << "0 0 0" << "\n";  
		outFilePose << "-0 0 -0" << "\n";
		
		outFileFrames << "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 2" << "\n";
		outFileFrames << "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 2" << "\n";
		
		outFile3D.close();
		outFileFrames.close();
		outFilePose.close();
	}
	
	void saveImage(IplImage* img, string filename, string destinationDir){
		string path = preparePath(filename, destinationDir);
		cvSaveImage(path.c_str(), img);
		cout << path << endl;
	}
	
	string prepareFilename(string dirName, int imageIndex, int flag){
		
		string filePrefix = dirName.substr(dirName.find_last_of("/")).append("-");
		stringstream stream;
		stream << imageIndex;
		string filename = filePrefix + stream.str();
		filename.append(".ppm");				
		return filename;
	}	
	
	IplImage* takeDifferenceImage(){
		
		IplImage* differenceImage = (IplImage*)cvClone(src);
		
		/* absolute difference of laser image with reference image */
		cvAbsDiff( src, referenceImage, differenceImage);
		
		return differenceImage;
	}
	
	void searchBrightestPixel(IplImage* differenceImage){
		
		/* split RGB channels */
		IplImage* srcR = cvCreateImage(cvGetSize(differenceImage), IPL_DEPTH_8U, 1);
		IplImage* srcG = cvCreateImage(cvGetSize(differenceImage), IPL_DEPTH_8U, 1);
		IplImage* srcB = cvCreateImage(cvGetSize(differenceImage), IPL_DEPTH_8U, 1);                		
		cvSplit(differenceImage, srcB, srcG, srcR, NULL);		
		
		for (int i=0; i<(differenceImage->width); i++){
			for (int j=0; j<(differenceImage->height); j++){ 					
				
				if (cvGetReal2D(srcR, j, i) < 50) {
					
					/* darken every non-laser pixel */					
					CvScalar s = cvGet2D(differenceImage, j, i); 
					s.val[0]=0;s.val[1]=0;s.val[2]=0;					
					cvSet2D(differenceImage,j,i,s);					
				}else{
					
					/* color laser pixel as RED */
					CvScalar s = cvGet2D(differenceImage, j, i); 
					s.val[0]=255;s.val[1]=0;s.val[2]=0;					
					cvSet2D(differenceImage,j,i,s);					
				}				
			}
		}	
		
		/* release images */
		cvReleaseImage(&srcR);
		cvReleaseImage(&srcG);
		cvReleaseImage(&srcB);		
	}
	
	vector < vector <CvPoint> > saveInVectors(IplImage* finalImage){
		
		/* split RGB channels */
		IplImage* srcR = cvCreateImage(cvGetSize(finalImage), IPL_DEPTH_8U, 1);
		IplImage* srcG = cvCreateImage(cvGetSize(finalImage), IPL_DEPTH_8U, 1);
		IplImage* srcB = cvCreateImage(cvGetSize(finalImage), IPL_DEPTH_8U, 1);                		
		cvSplit(finalImage, srcB, srcG, srcR, NULL);
		
		/* define vectors */
		bool ifLeftLaser = true;
		vector < vector <CvPoint> > pointWrapper;
		vector<CvPoint> leftLaser;
		vector<CvPoint> object;
		vector<CvPoint> rightLaser;
		
		for(int i=0; i<finalImage->width; i++){
			for(int j=0; j<finalImage->height; j++){
				
				if(cvGetReal2D(srcB, j, i) > OBJECT_THRESH){
					object.push_back(cvPoint(j,i));
					ifLeftLaser = false;
				}
				
				if(cvGetReal2D(srcR, j, i) > LASER_THRESH){
					if(ifLeftLaser == true)
						leftLaser.push_back(cvPoint(j,i));    
					else 
						rightLaser.push_back(cvPoint(j,i));					
				}
			}
		}
		
		/* wrap the three vectors */
		pointWrapper.push_back(leftLaser);
		pointWrapper.push_back(object);
		pointWrapper.push_back(rightLaser);
		
		/* release images */
		cvReleaseImage(&srcR);
		cvReleaseImage(&srcG);
		cvReleaseImage(&srcB);	
		
		return pointWrapper;
	}
	
public:
	
	LaserScanner(){}
	
	LaserScanner(vector<CvMat*> param){cameraParameters = param;}
	
	int scanImages(int numberOfImages, string dirName, string referenceImageLocation, string destinationDir){
		
		int flag = 0;
		Transforms* transform = new Transforms();
		PointCloud* pointCloud = new PointCloud(cameraParameters);
		
		/* load the reference image */
		referenceImage = cvLoadImage(referenceImageLocation.c_str(), CV_LOAD_IMAGE_UNCHANGED);		
		
		/* create a posix directory structure */
		DIR* dir = opendir(dirName.c_str());
		if(dir != NULL){
			
			struct dirent* dirEntry;
			cout << endl << endl << "saving the images ..." << endl << endl;				
			
			/* iterate the directory listing */
			while((dirEntry = readdir(dir)) != NULL){
				
				string file = dirEntry->d_name;				
				if(file != "." && file != ".." && file != ".DS_Store"){
					
					/* load the image */
					
					string filename = preparePath(file, dirName);
					src = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_UNCHANGED);
					
					if (src != NULL) {
						
						/* take a difference from reference image */
						IplImage* intermediateImage = takeDifferenceImage();
						
						/* smoothen the difference image */		
						transform->setSrc(intermediateImage);
						intermediateImage = transform->doGauss();
						
						/* search brightest pixels along a column */
						searchBrightestPixel(intermediateImage);
						
						/* apply hough transform */
						transform->setSrc(intermediateImage);
						IplImage* finalImage = transform->doHough(HOUGH_THRESH, HOUGH_PARAM1, HOUGH_PARAM2);
						
						/* wrap object and laser points in different vectors */
						vector < vector <CvPoint> > pointWrapper = saveInVectors(finalImage);				
						
						/* generate the point cloud */
						pointCloud->setSrc(pointWrapper);
						pointCloud->generate(referenceImage);
						
						/* save the transforms */
						saveImage(finalImage, file, destinationDir);				
						
						/* release resources */
						cvReleaseImage(&src);	
						cvReleaseImage(&intermediateImage);
						
						/* images were loaded successfuly */
						flag = 1;
					}				
				}
			}
			
			/* get the final point cloud */
			vector <Point3DRGB*> pointCloudVector = pointCloud->getPointCloud();		
			
			/* save the point cloud */
			savePointCloud(pointCloudVector, destinationDir);		
			
		}
		else					
			cout << "Cannot Open Directory" << endl << endl;				
		
		cvReleaseImage(&referenceImage);	
		delete transform;
		delete pointCloud;
		return flag;
	}	
	
};

int main(int argc, char* argv[]){
	
	if (argc == 7) {		
		
		/* calibrate the camera */	
		CalibrateCamera *camera = new CalibrateCamera();
			
		vector <string> imageList;
		imageList.push_back(argv[4]);	
		imageList.push_back(argv[5]);
		vector<CvMat*> cameraParameters = camera->calibrateFromImages(imageList);	
		delete camera;
		
		/* scan the images */		
		LaserScanner *scanner = new LaserScanner(cameraParameters);
		int flag = scanner->scanImages(atoi(argv[1]), argv[2], argv[3], argv[6]);
		if (!flag) {cout << "cannot load files";}
		delete scanner;	
		
	}
	else {
		cout << "usage: laserscanner ";
		cout << "<number of images> <image directory> <reference image file>";
		cout << "<left checkerboard file> <right checkerboard file>";
		cout << "<destination directory>";
	}	
	return 0;
}