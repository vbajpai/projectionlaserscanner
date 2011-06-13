usage: 

laserscanner <number of images> <image directory> <reference image file>
 			 <left checkerboard file> <right checkerboard file> <destination directory>


-- <reference image file>: used to do a difference image with every input image (please supply a image without the laser)

-- <left checkerboard file>: image file containing only the left checkerboard.

-- <right checkerboard file>: image file containing only the right checkerboard.


example:

laserscanner 350 /home/vaibhav/inputImageDir /home/vaibhav/images/referenceImage.jpg
				 /home/vaibhav/leftBoard.jpg /home/vaibhav/rightBoard.jpg
				 /home/vaibhav/outDir



