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

#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;
//SURF追加
using namespace cv::xfeatures2d;



void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{ 

//this function automatically gets rid of points for which tracking fails

  vector<float> err;					
  Size winSize=Size(21,21);
 																								
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
//Points2を上書きしている？
  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

// 要素数を合わせる？
 //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
     	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
     		  if((pt.x<0)||(pt.y<0))	{
     		  	status.at(i) = 0;
     		  }
     		  points1.erase (points1.begin() + (i - indexCorrection));
     		  points2.erase (points2.begin() + (i - indexCorrection));
     		  indexCorrection++;
     	}

     }

}
//KAZE追加
void KAZEdesu(Mat img_1, vector<Point2f>& points1,vector<KeyPoint>& keypoints_1){

 int minHessian = 30;
 Ptr<cv::KAZE> detector = KAZE::create( minHessian );
 //std::vector
 //vector<KeyPoint> keypoints_1;
 detector->detect( img_1, keypoints_1 );
 KeyPoint::convert(keypoints_1, points1, vector<int>());

}

//AKAZE追加
void AKAZEdesu(Mat img_1, vector<Point2f>& points1,vector<KeyPoint>& keypoints_1){

 int minHessian = 40;
 Ptr<cv::AKAZE> detector = AKAZE::create( minHessian );
 //std::vector
 //vector<KeyPoint> keypoints_1;
 detector->detect( img_1, keypoints_1 );
 KeyPoint::convert(keypoints_1, points1, vector<int>());

}
//ORB追加
void ORBdesu(Mat img_1, vector<Point2f>& points1,vector<KeyPoint>& keypoints_1){

 int minHessian = 40;
 Ptr<cv::ORB> detector = ORB::create( minHessian );
 //std::vector
 //vector<KeyPoint> keypoints_1;
 detector->detect( img_1, keypoints_1 );
 KeyPoint::convert(keypoints_1, points1, vector<int>());

}

//SIFT追加
void SIFTdesu(Mat img_1, vector<Point2f>& points1,vector<KeyPoint>& keypoints_1){

 int minHessian = 8000;
 Ptr<cv::xfeatures2d::SIFT> detector = SIFT::create( minHessian );
 //std::vector
 //vector<KeyPoint> keypoints_1;
 detector->detect( img_1, keypoints_1 );
 KeyPoint::convert(keypoints_1, points1, vector<int>());
}

//SURF追加
void SURFdesu(Mat img_1, vector<Point2f>& points1, vector<KeyPoint>& keypoints_1){
//大きいほど厳しい
 int minHessian = 400;
 Ptr<cv::xfeatures2d::SURF> detector = SURF::create( minHessian );
 //std::vector
 //vector<KeyPoint> keypoints_1;
 detector->detect( img_1, keypoints_1 );
 KeyPoint::convert(keypoints_1, points1, vector<int>());

}

//FAST追加
void featureDetection(Mat img_1, vector<Point2f>& points1,vector<KeyPoint>& keypoints_1)	{   //uses FAST as of now, modify parameters as necessary
  //vector<KeyPoint> keypoints_1;
  int fast_threshold = 40;
  bool nonmaxSuppression = true;
  FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  KeyPoint::convert(keypoints_1, points1, vector<int>());

}


//RTを出す関数
void Rt(Mat img_1_c, Mat img_2_c,Mat R,Mat t,Mat K){
//配列準備	
	Mat img_1,img_2;
	Mat mask;
	Mat E;
	double focal; //= 684.3265;
	Point2d pp(K.at<double>(0,2), K.at<double>(1,2));
	focal=sqrt(K.at<double>(0,0)*K.at<double>(0,0)+K.at<double>(1,1)*K.at<double>(1,1));

	
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);
//特徴抽出準備
	vector<KeyPoint> keypoints1;
	vector<KeyPoint> keypoints2;
	vector<KeyPoint> keypoints11;
	vector<KeyPoint> keypoints22;
	vector<Point2f> points1;
	vector<Point2f> points2;
	
	
// 特徴点抽出（選ぶ）
	//SURF
	SURFdesu(img_1, points1,keypoints1);
	SURFdesu(img_2, points2,keypoints2);
	//
	
	/*SIFT
	SIFTdesu(img_1, points1,keypoints1);
	SIFTdesu(img_2, points2,keypoints2);
	*/
	
	/*ORB
	ORBdesu(img_1, points1,keypoints1);
	ORBdesu(img_2, points2,keypoints2);
	*/
	
	/*FAST
	featureDetection(img_1, points1,keypoints1);
	featureDetection(img_2, points2,keypoints2);
	//
	
	/*KAZE
	KAZEdesu(img_1, points1,keypoints1);
	KAZEdesu(img_2, points2,keypoints2);
	*/
	
	/*AKAZE
	AKAZEdesu(img_1, points1,keypoints1);
	AKAZEdesu(img_2, points2,keypoints2);
	*/
	cout<<"抽出された特徴点の数="<<points1.size()<<endl;
	
// 特徴記述（選ぶ）
	Mat descriptor1, descriptor2;
	//SURF
	SURF::create()->compute(img_1, keypoints1, descriptor1);
	SURF::create()->compute(img_2, keypoints2, descriptor2);
	//
	
	/*SIFT
	SIFT::create()->compute(img_1, keypoints1, descriptor1);
	SIFT::create()->compute(img_2, keypoints2, descriptor2);
	*/
	
	/*ORB
	ORB::create()->compute(img_1, keypoints1, descriptor1);
	ORB::create()->compute(img_2, keypoints2, descriptor2);
	*/
	
	/*FAST
	SURF::create()->compute(img_1, keypoints1, descriptor1);
	SURF::create()->compute(img_2, keypoints2, descriptor2);
	*/
	
	/*KAZE
	KAZE::create()->compute(img_1, keypoints1, descriptor1);
	KAZE::create()->compute(img_2, keypoints2, descriptor2);
	*/
	
	/*AKAZE
	AKAZE::create()->compute(img_1, keypoints1, descriptor1);
	AKAZE::create()->compute(img_2, keypoints2, descriptor2);
	*/
// マッチング (アルゴリズムにはBruteForceを使用)
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
	std::vector<cv::DMatch> match, match12, match21;
	matcher->match(descriptor1, descriptor2, match12);
	matcher->match(descriptor2, descriptor1, match21);
//クロスチェック(1→2と2→1の両方でマッチしたものだけを残して精度を高める)
	for (size_t i = 0; i < match12.size(); i++)
	{
		cv::DMatch forward = match12[i];
		cv::DMatch backward = match21[forward.trainIdx];
	if (backward.trainIdx == forward.queryIdx)
	{
		match.push_back(forward);
	}
	}
	
//対応付けられていない特徴点を消す
	vector<Point2f> p1;
	vector<Point2f> p2;
	
	for (size_t i = 0; i < match.size(); i++)
	{
		cv::DMatch forward = match[i];
		int query = forward.queryIdx;
		int train = forward.trainIdx;
		p1.push_back(keypoints1[query].pt);
		p2.push_back(keypoints2[train].pt);
	}
	cout<<"対応付けられた特徴点の数="<<p1.size()<<endl<<endl;
	//cout<<"p2="<<p2.size()<<endl<<endl;
	
	
//E行列からR,tを求める
	E = findEssentialMat(p1, p2, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, p1, p2, R, t, focal, pp, mask);
	


}
