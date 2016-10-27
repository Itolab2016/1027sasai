#include"camera.h"

#define SAVE "/home/syouyu/デスクトップ/work/test_img/"
#define SAVE_TXT "/home/syouyu/デスクトップ/work/test_data/"
char file0[]=SAVE;
char file1[]=SAVE_TXT;
Mat frame1,frame2;
double scale=100;
int main (){
	int count=0;
	camera_open();
	while(1)
	{
		capture2 (file0,frame1,frame2);
		if (count!=0){

			cout<<"sfm_start"<<endl;
			sfm(file0,file1,scale,frame1,frame2);
			cout<<"sfm_ok"<<endl;
		}
		count++;	
	}
}
