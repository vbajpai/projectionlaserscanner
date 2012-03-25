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

#include "transforms.h"

Transforms::Transforms(IplImage* src){this->src = src;}

Transforms::Transforms(){}	

IplImage* 
Transforms::doGauss(){
  dst = cvCloneImage(src);
  cvSmooth(src, dst, CV_GAUSSIAN, 5, 5, 0, 0);
  return dst;
}

IplImage* 
Transforms::doCanny(int lowThresh, 
                    int highThresh, 
                    int appertureSize){
  
  /* convert to gray scale */
  IplImage* dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  cvCvtColor(src, dst, CV_RGB2GRAY);              
  cvCanny(dst, dst, lowThresh, highThresh, 3);
  return dst;
}

IplImage* 
Transforms::doHough(int houghThresh, 
                    int houghParam1, 
                    int houghParam2){
  
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

void 
Transforms::setSrc(IplImage* src){this->src = src;}

