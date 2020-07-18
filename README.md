# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Implementation Steps

### MP.1 Data Buffer Optimization

The "ring buffer" is successfully implemented. Here is code

		 // push image into data frame buffer
          DataFrame frame;
          frame.cameraImg = imgGray;
          dataBuffer.push_back(frame);

		// removing youngest data frame
          if(dataBuffer.size()>dataBufferSize)
        	{
                dataBuffer.erase(dataBuffer.begin());
            }

### MP.2 Keypoint Detection

HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT keypoint detectors were implemented and it is in the file src/matching2D_Student.cpp
There are 3 functions for all detectors

		void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false)
		void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false)
		void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis=false)


### MP.3 Keypoint Removal

		// only keep keypoints on the preceding vehicle
                    bool bFocusOnVehicle = true;
                    double N_size = 0;
                    cv::Rect vehicleRect(535, 180, 180, 150);
                    if (bFocusOnVehicle)
                    {
                        vector<cv::KeyPoint> keypoints_f;            

                        for (auto pts : keypoints)
                        {
                            if(vehicleRect.contains(pts.pt))
                            {
                                keypoints_f.push_back(pts);
                                N_size += pts.size;
                            }

                        }

                        keypoints = keypoints_f;            
                    }


### MP.4 Keypoint Descriptors
The implementation of BRIEF, ORB, FREAK, AKAZE and SIFT descriptors were done in the file src/matching2D_Student.cpp
		void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType)

### MP.5 Descriptor Matching
FLANN matcher and kNN matchers were implemented in the file src/matching2D_Student.cpp and function is

		void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)


### MP.6 Descriptor Distance Ratio

		double minDescDistRatio = 0.8;
        for (auto iter_ = knn_matches.begin(); iter_ != knn_matches.end(); ++iter_)
        {

            if ((*iter_)[0].distance < minDescDistRatio * (*iter_)[1].distance)
            {
                matches.push_back((*iter_)[0]);
            }
        }

## Performance Evaluation
### MP.7 - MP.9

Table formed using data from program given below. Detected keypoints, time taken and distribution neighborhood size are given in the table. Total time taken for both detectors and describers in all combinations for 10 images is calculated and sorted according to total time. All data can be accessed [spreadsheet data](../master/PerformanceEvaluation.xlsx)

| Detector  | Descriptor | Average time | Average keypoints | Average neighburhood size |
| --------- | ---------- | ------------ | ----------------- | ------------------------- |
|FAST |ORB |3,0677353 |409,4 |7 |
|FAST |BRIEF |3,1678278 |409,4 |7 |
|ORB |BRIEF |11,19028 |116,1 |56,0577832 |
|ORB |ORB |10,7794224 |116,1 |56,0577832 |
|HARRIS |ORB |25,4326867 |24,8 |6 |
|HARRIS |BRIEF |28,1377004 |24,8 |6 |
|SHITOMASI |ORB |25,3624216 |117,9 |4 |
|SHITOMASI |BRIEF |31,8234908 |117,9 |4 |
|SHITOMASI |SIFT |20,2173938 |117,9 |4 |
|HARRIS |SIFT |36,8260311 |24,8 |6 |
|FAST |SIFT |3,0968701 |409,4 |7 |
|FAST |FREAK |3,0340714 |409,4 |7 |
|ORB |FREAK |10,9423031 |116,1 |56,0577832 |
|SHITOMASI |FREAK |22,0501076 |117,9 |4 |
|HARRIS |FREAK |27,870707 |24,8 |6 |
|ORB |SIFT |10,992266 |116,1 |56,0577832 |
|AKAZE |BRIEF |127,5955323 |167 |7,6934134 |
|AKAZE |ORB |129,5579912 |167 |7,6934134 |
|AKAZE |SIFT |125,1735246 |167 |7,6934134 |
|AKAZE |FREAK |127,3950658 |167 |7,6934134 |
|SIFT |BRIEF |179,4640643 |138,6 |5,0323454 |
|AKAZE |AKAZE |123,5045318 |167 |7,6934134 |
|SIFT |FREAK |178,7101766 |138,6 |5,0323454 |
|SIFT |SIFT |154,2261487 |138,6 |5,0323454 |
|BRISK |BRIEF |467,1615334 |276,2 |21,9422022 |
|BRISK |ORB |454,2493615 |276,2 |21,9422022 |
|BRISK |FREAK |447,3425083 |276,2 |21,9422022 |
|BRISK |SIFT |449,4811271 |276,2 |21,9422022 |


From the table above, it is clear that FAST/ORB and FAST/BRIEF combination required the least time and it is also detecting highest number of keypoints. However distribution neighborhood size is not so large for these combinations. When all three parameters are taken into consideration (time, keypoint number, distribution neighborhood size), the better combinationations can be sorted out as BRISK/BRIEF and BRISK/ORB 


## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.