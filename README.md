Projection-based 3D Laser Scanner
---------------------------------

![Imgur](http://i.imgur.com/Hw5JM.png)

Methodology
-----------

* Calibrate the Camera
	* Find Camera' Intrinsic Parameters
    * Find Camera' Extrinsic Parameters 

![Imgur](http://i.imgur.com/T4jhZ.png)


* Search for the Laser Line (and the object points)
    * Take a Difference Image to find the Laser
    * Smoothen the Difference Image
    * Color Threshold to remove everything else
    * Apply Hough Transform
    * Wrap the Points and Return the results

![Imgur](http://i.imgur.com/Pv7f3.png)


* Generate a Color 3D Point Cloud
    * Transform the Right Laser Points to Left Coordinate System
    * Get the 3D Laser Points using Camera Extrinsics
    * Get the Laser Plane Equation using 3 Laser Points
    * Get the 3D Object Point by intersecting the Laser Plane with Light Ray
    * Save the 3D Object Points with their color values using Reference Image
* Register different scans using Slam6D/ICP

![Imgur](http://i.imgur.com/obTwO.png)


