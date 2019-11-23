# Stereo Test Kit
This is a build environment used to test out stereo techniques 

## Stereo Base classes
### Deimos base
This is the base stereo capture library for the deimos stereo camera. 

**[#TODO]** This differs slightly in implimentation from windows to linux but reference to the class is the same. 
Linux uses uvc cam library (which is uses Glib only avaiable on linux)

**[#TODO]** Windows usees hidapi library (aim to switch to uses this soley to avoid needed extra files)

### Phobos base
**[#TODO]**

This is the base stereo capture libray for the phobos stereo camera

## Video pair base
**[#TODO]**

Use stereo videos previously recorded to generate 3D. Requires the calibration paramters of the camera used to record the videos. 

Left.yaml - Calibration of the left camera

Right.yaml - Calibration of the right camera

## Image pair base
**[#TODO]**

Use stereo images previously recorded to generate 3D.
Requires the calibration parameters of the camera used to capture the images.

Left.yaml - Calibration of the left camera

Right.yaml - Calibration of the right camera

## Libraries
### OpenCV Calibration
**[#TODO]** 

This library has calibration routines needed to generate stereo calibration files. Will generate both XML for windows and YAML for linux.
Build option:
```bash
-DWITH_OPENCV_CAL=TRUE (Enabled by default)
```
### OpenCV Stereo Matchers (Block, SGBM)
This library has routines for generating 3D using OpenCV's Block and SGBM Stereo matching algorithms. 

**[#TODO]** 
Build option:
```bash
-DWITH_OPENCV_STEREO=TRUE (Enabled by default)
```
### I3DR Stereo Matcher (SGM)
**[#TODO]** 

This libary has routine for generating 3D using I3DR Stereo Matcher algorithm.

Build option:
```
-DWITH_I3DR_STEREO=TRUE (Disabled by default)
```
### OpenCV Point Clouds
**[#TODO]**

This library has routines for manipulating point clouds using OpenCV's _3DImage datatype.

Build option:
```
-DWITH_OPENCV_POINT_CLOUDS=TRUE (Disabled by default)
```
### PCL Point Clouds
**[#TODO]**

This library has routines for manipulating point clouds using Point Cloud Library datatype.

(Currently incompatible with UcoSLAM)

Build option:
```
-DWITH_PCL_POINT_CLOUDS=TRUE (Disabled by default)
```
### OpenCV Marker Detection
**[#TODO]**

This library has routines for detecting different marker types. Currently implimented: QR, ArUco. 

Build option:
```bash
-DWITH_OPENCV_MARKERS=TRUE (Enabled by default)
```
### UcoSLAM
This is a SLAM library for fast generation of location tracking using feature tracking. Additionally ArUco markers can be used to assist with tracking.

Current issue with using this library with the Point Cloud Library (PCL)

**[#TODO]** Build option:
```bash
-DWITH_UCOSLAM=TRUE (Disabled by default)
```
### RTABMAP 
**[#TODO]**

This is a SLAM library.

Build option:
```bash
-DWITH_RTABMAP=TRUE (Disabled by default)
```