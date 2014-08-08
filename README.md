CvStereo - DisCODe Component Library
====================================

Description
-----------

DCL containing components and tasks realted to aquisition and processing of depth maps/point clouds from stereopairs.

Dependencies
------------

OpenCV - version 2.4.8

PCL - version 1.7.1.

CvCoreTypes - DisCODe DCL

Datasets
------------
Call __make dataset__ from the build directory in order to download and unrar the files required by exemplary tasks.
   * bible_lr - contains images acquited from left and right camera of the Velma's active head
   * bible_rgbxyz - dataset containing RGB-D images (pairs of RGB and XYZ files)    

Tasks
------------
Velma acquisition:
   * VelmaStereoCameraViewer - Displays left and right image acquired from the stereo pair mounted on Velma head
   * VelmaStereoCameraCloudViewer - Acquires stereo images from Velma head and transforms them into coloured point clouds 
Stereo sequences from files:   
   * StereoLRSequenceViewer - Displays a sequence of left and right images acquired from a stereo pair
   * StereoLRSequenceCloudViewer - Generates and displays point cloud generated from stereo-images (LR). Enables to write the RGB-D (a pair of RGB and XYZ images) to file.

Developers
----------

Dawid Kaczmarek (frikodwid@gmail.com)

Łukasz Żmuda (lukzmuda1@gmail.com)

Maintainer
----------

Tomasz Kornuta (tkornuta@gmail.com)

