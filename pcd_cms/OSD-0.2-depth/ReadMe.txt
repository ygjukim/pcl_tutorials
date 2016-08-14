Object Segmentation Database (OSD)
==================================

Segmenting unknown objects from generic scenes is one of the ambitious and elusive goals
of computer vision. With the recent introduction of cheap and powerful 3D sensors 
(such as the Microsoft Kinect or Asus XtionPRO) which deliver a dense point cloud plus 
color for almost any indoor scene, a renewed interest in 3D methods holds the promise to 
push the envelope slightly further.

The Object Segmentation Database provides RGBD data in several subcategories to allow 
evaluation of object segmentation approaches. The database contains currently 111 entries:
./annotation/   ... annotation of the objects in image space as .png image
./image_color/ 	... the color image in .png format,
./pcd/		... the point cloud in PCL format (PointXYZRGBL) with ground truth
./disparity/    ... 16bit png image format with depth data

Learnset:
	learn  0-16 => Boxes
	learn 17-24 => Stacked Boxes
	learn 25-32 => Occluded Objects
	learn 33-44 => Cylindric Objects

Testset:
	test  0-15 => Boxes
	test 16-23 => Stacked Boxes
	test 24-30 => Occluded Objects
	test 31-42 => Cylindric Objects
	test 43-54 => Mixed Objects
	test 55-65 => Complex Scene

OSD Version 0.2: Higher accuracy of ground truth; unite point cloud and annotation in pcl-style PointXYZRGBL
=> OSD-0.2 also available as database of depth images


Author
======
DI Andreas Richtsfeld
Automation and Control Institute (ACIN)
Vienna University of Technology
Gusshausstraße 25-29
1040 Vienna
ari(at)acin.tuwien.ac.at

PS: Don't hasitate to contact me for further information.
