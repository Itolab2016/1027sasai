#include "camera.h"

#define SAVE "/home/ubuntu/Desktop/img/"
#define CAMERA 440.9741,0,290.2624,0,443.2639,242.4028,0,0,1
#define DISTORTION -0.39703,0.14661,0.001369,-0.0075874,-0.016642



using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//media/ubuntu/Transcend/tukuba_img

static Mat frame0,frame,dframe;
static int open=40;
static int frameNo=0;
static char str[255];//フレーム画像保存用
static VideoCapture cap(0);
static char file[]=SAVE;
static Mat R_f= Mat::eye(3, 3, CV_64FC1);//回転合計
static Mat t_f= Mat::zeros(3, 1, CV_64FC1);//並進合計
static char filename1[100];//3次元復元画像1
static char filename2[100];//3次元復元画像2
static Mat img_1, img_2;//グレー画像保存用
//特徴抽出準備
static vector<KeyPoint> keypoints1;
static vector<KeyPoint> keypoints2;
static vector<KeyPoint> keypoints11;
static vector<KeyPoint> keypoints22;
static vector<Point2f> points1;
static vector<Point2f> points2;
static Mat descriptor1, descriptor2;//特徴量記述用
static Mat K=(Mat_<double>(3,3,CV_64FC1) <<CAMERA);//K 
static Mat distortionCoefficient=(cv::Mat_<float>(4,1) <<DISTORTION);//歪み係数(本番カメラ）

	
int camera_open(void)
{
	
	if(!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
	{
		//読み込みに失敗したときの処理
		cout<<"読み込みエラー"<<endl;
		exit(1);
		return -1;
	}

	for(int i=0;i<open;i++)
	{
		//cout<<"capture"<<endl;
		capture0(file);//写真取る
	}
	
}

static int capture0 (char *save_point)
{	
	cap >> frame0;
	//フレーム画像を保存する．

	sprintf(str,"%sopen.png",save_point);
        imwrite(str, frame0);
}

int capture (char *save_point)
{	
	cap >> frame;
	//フレーム画像を保存する．
	frameNo++;
	sprintf(str,"%s%04d.png",save_point,frameNo);
        imwrite(str, frame);
}


int capture2 (char *save_point,Mat frame1,Mat frame2)
{	
	frame2=frame1.clone();
	cap >> frame1;
	//フレーム画像を保存する．
	frameNo++;
	sprintf(str,"%s%04d.png",save_point,frameNo);
        imwrite(str, frame1);
}


static void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{ 

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
static void SURFdesu(Mat img_1, vector<Point2f>& points1, vector<KeyPoint>& keypoints_1){
//大きいほど厳しい
 int minHessian = 400;
 Ptr<cv::xfeatures2d::SURF> detector = SURF::create( minHessian );
 //std::vector
 //vector<KeyPoint> keypoints_1;
 detector->detect( img_1, keypoints_1 );
 KeyPoint::convert(keypoints_1, points1, vector<int>());

}


int sfm(char *save_point,char *save_point_data,double scale,Mat frame1,Mat frame2){

	double focal=(K.at<float>(0,0)+K.at<float>(1,1))/2;//焦点距離（本番カメラ）
	Point2d pp(K.at<float>(0,2), K.at<float>(1,2));//中心座標(本番カメラ)

/*
	sprintf(filename1, "%s%04d.png",save_point,frameNo-1);
	sprintf(filename2, "%s%04d.png",save_point,frameNo);
*/
	
	//比較用画像を読み込む 
	Mat img_1_c = frame1;//imread(filename1);
	Mat img_2_c = frame2;//imread(filename2);
	cout<<"ok"<<endl;
	sprintf(str,"%stest1.png",save_point);
        imwrite(str, img_1_c);	
	sprintf(str,"%stest2.png",save_point);
        imwrite(str, img_2_c);	

	// グレーにする
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);
	// 特徴点抽出
	SURFdesu(img_1, points1,keypoints1);
	SURFdesu(img_2, points2,keypoints2);
	// 特徴記述
	Mat descriptor1, descriptor2;//特徴量記述用
	SURF::create()->compute(img_1, keypoints1, descriptor1);
	SURF::create()->compute(img_2, keypoints2, descriptor2);
	//

	// マッチング (アルゴリズムにはBruteForceを使用)
	Ptr<cv::DescriptorMatcher> knnmatcher = cv::DescriptorMatcher::create("BruteForce");
	vector<cv::DMatch> match;
	vector< vector<DMatch> > knnmatch12,knnmatch21;
	knnmatcher->knnMatch(descriptor1, descriptor2, knnmatch12, 2); //上位2位までの点を探す
	knnmatcher->knnMatch(descriptor2, descriptor1, knnmatch21, 2);
	//knntest
	vector<DMatch> bestMatches12;
	vector<KeyPoint> bestkey112,bestkey212;
	vector<Point2f> p1;
	vector<Point2f> p2;
	vector<Point2f> p11;
	vector<Point2f> p22;


	float match12_par = .5f; //対応点のしきい値
	for (int i=0; i<knnmatch12.size(); i++)
	{
		float dist1 = knnmatch12[i][0].distance;
		float dist2 = knnmatch12[i][1].distance;
		//良い点を残す（最も類似する点と次に類似する点の類似度から）
		if (dist1 <= dist2 * match12_par && dist1<=0.2f)
		{
		        bestMatches12.push_back(knnmatch12[i][0]);
		        bestkey112.push_back(keypoints1[knnmatch12[i][0].queryIdx]);
		        bestkey212.push_back(keypoints2[knnmatch12[i][0].trainIdx]);
			p1.push_back(keypoints1[knnmatch12[i][0].queryIdx].pt);
			p2.push_back(keypoints2[knnmatch12[i][0].trainIdx].pt);
		}
	}

	//E行列からR,tを求める
	vector<cv::DMatch> inlinerMatch;
	Mat E,R,t,mask;
  	E = findEssentialMat(p1, p2, focal, pp, RANSAC, 0.999, 1.0, mask);
  	recoverPose(E, p1, p2, R, t, focal, pp, mask);  
	
	//maskを使って精度を高める
	for (size_t i = 0; i < mask.rows; ++i)
	{
	        uchar *inliner = mask.ptr<uchar>(i);
	        if (inliner[0] == 1)
		{
			inlinerMatch.push_back(bestMatches12[i]);
			p11.push_back(keypoints1[bestMatches12[i].queryIdx].pt);
			p22.push_back(keypoints2[bestMatches12[i].trainIdx].pt);
        	}
	}


	//移動量計算
	t=t*scale;

	//Rt作
	Mat Rt=cv::Mat::eye(3, 4, CV_64FC1);
  	for(double m=0 ; m < 3; m++)
  		{
  		for(double b=0 ; b < 3; b++)
  			{
  			Rt.at<double>(m,b)=R.at<double>(m,b);
  			}
		}
	Rt.at<double>(0,3)=t.at<double>(0);
	Rt.at<double>(1,3)=t.at<double>(1);
	Rt.at<double>(2,3)=t.at<double>(2);

	//透視射影行列作成
	Mat Meye = cv::Mat::eye(3, 4, CV_64FC1);
	Mat M0 = cv::Mat::eye(3, 4, CV_64FC1);
	Mat M1=cv::Mat::eye(3, 4, CV_64FC1);
	M0=K*Meye;
  	M1=K*Rt;

	//三角測量による3次元復元
	Mat point4D;
	Mat point4Dh=(Mat_<float>(1,p11.size()*4,CV_32FC1));
	vector<Point3f> point3D;
	triangulatePoints(M0, M1, p11, p22, point4D);


	//ch変更1to4(convertPointsFromHomogeneousのため)		
	int n=0;
	for(int i=0;i<p11.size()/*match.size()*/;i++)
	{
		point4Dh.at<float>(0,n)=point4D.at<float>(0,i);
		point4Dh.at<float>(0,n+1)=point4D.at<float>(1,i);
		point4Dh.at<float>(0,n+2)=point4D.at<float>(2,i);
		point4Dh.at<float>(0,n+3)=point4D.at<float>(3,i);
		n=n+4;
	}
	//3次元座標にする
	convertPointsFromHomogeneous(point4Dh.reshape(4,1),point3D);

/*	カメラのみの場合使用
	//原点からの座標に変換
	for(int i=0;point3D.size()>i;i++)
	{
		point3D[i].x=point3D[i].x-t_f.at<double>(0);
		point3D[i].y=point3D[i].y-t_f.at<double>(1);
		point3D[i].z=point3D[i].z-t_f.at<double>(2);
	}
*/
	//総移動量計算
	t_f = t_f + R_f*t;
	R_f = R*R_f;

	//座標情報を貯める
	vector<Point3f> point3DW;
	vector<Point3f> pointme;
	for(int i=0;point3D.size()>i;i++)
	{
		point3DW.push_back(point3D[i]);
	}
	pointme.push_back(Point3f(t_f));


	//txtに書き込む
	char data[255];
	sprintf(data,"%s/3Ddata%04d.txt",save_point_data,frameNo-1);
	char ddata[255];
	ofstream fs(data);

	for(int i=0;i<point3D.size();i++)
	{
		sprintf(ddata,"%lf %lf %lf",point3D[i].x,point3D[i].y,point3D[i].z);
		fs<<ddata<<endl;
	}
	fs.close();
}

