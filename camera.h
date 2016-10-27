#ifndef CAPTURE_H
#define CAPTURE_H
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <math.h>
//SURF追加
#include "opencv2/xfeatures2d.hpp"
#include <stdio.h>
#include "opencv2/core.hpp"


using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
int camera_open(void);
int capture (char *save_point);
int capture2 (char *save_point,Mat frame1,Mat frame2);
static int capture0 (char *save_point);
static void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status);
static void SURFdesu(Mat img_1, vector<Point2f>& points1, vector<KeyPoint>& keypoints_1);
int sfm(char *save_point,char *save_point_data,double scale,Mat frame1,Mat frame2);
#endif
