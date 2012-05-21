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

#include "laserscanner.h"

LaserScanner::LaserScanner(){}

LaserScanner::LaserScanner(vector<CvMat*> param){cameraParameters = param;}

string 
LaserScanner::preparePath(string fileName, string dirName){
		string path;
		
		/* add a slash at the end of directory if not supplied */
		if (*(dirName.rbegin()) != '/')			
			dirName.append("/");	
		
		path = dirName + fileName;
		return path;
	}	
	
void 
LaserScanner::savePointCloud(vector<Point3DRGB*> pointCloud, 
                             string destinationDir){
		
		string file3D = "/scan000.3d";
		string fileFrames = "/scan000.frames";
		string filePose = "/scan000.pose";
		
		ofstream outFile3D(preparePath(file3D, destinationDir).c_str());
		ofstream outFileFrames(preparePath(fileFrames, destinationDir).c_str());
		ofstream outFilePose(preparePath(filePose, destinationDir).c_str());
		
		outFile3D << pointCloud.size() << "\n";		
		
		for (unsigned int i=0; i < pointCloud.size(); i++) {
			
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
	
void 
LaserScanner::saveImage(IplImage* img, 
                        string filename, 
                        string destinationDir){
		string path = preparePath(filename, destinationDir);
		cvSaveImage(path.c_str(), img);
		cout << path << endl;
	}
	
string 
LaserScanner::prepareFilename(string dirName, 
                              int imageIndex, 
                              int flag){
		
		string filePrefix = dirName.substr(dirName.find_last_of("/")).append("-");
		stringstream stream;
		stream << imageIndex;
		string filename = filePrefix + stream.str();
		filename.append(".ppm");				
		return filename;
	}	
	
IplImage* 
LaserScanner::takeDifferenceImage(){
		
		IplImage* differenceImage = (IplImage*)cvClone(src);
		
		/* absolute difference of laser image with reference image */
		cvAbsDiff( src, referenceImage, differenceImage);
		
		return differenceImage;
	}
	
void 
LaserScanner::searchBrightestPixel(IplImage* differenceImage){
		
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
	
vector < vector <CvPoint> > 
LaserScanner::saveInVectors(IplImage* finalImage){
		
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
	
int 
LaserScanner::scanImages(int numberOfImages, 
                         string dirName, 
                         string referenceImageLocation, 
                         string destinationDir){
		
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
	
int 
main(int argc, char* argv[]){

const char usage_string[] =
  " <number of images>\n"
  "                     <image directory>\n"
  "                     <reference image fine>\n"
  "                     <left checkerboard file>\n"
  "                     <right checkerboard file>\n"
  "                     <destination directory>";


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
		cout << "\nprojection-based 3D laser scanner\n\n";
		cout << "usage: " << argv[0] << usage_string << endl;
	}	
	return 0;
}
