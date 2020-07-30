/*
 * FlowVector.cpp
 *
 *  Created on: Mar 14, 2019
 *      Author: bro
 */

#include "cam/FlowVector.h"

using namespace std;
using namespace cv;
using namespace Eigen;

Mat left_prev;
Mat right_prev;
pair<int,int> obstacles_detected;
vector<pair<int,int>> _obstacle_location;
int _count =1;


String FlowVector::identify_location(){
	String location="";
	std::vector < pair<int,int> > ::reverse_iterator revIt;
	for (revIt = _obstacle_location.rbegin(); revIt != _obstacle_location.rend(); revIt++){
		if(revIt->first>1 && revIt->second > 1){
			location = "Obstacles on both sides";
		}
		else{
		 if(revIt->first > 1){
				location="Obstacle on left";
			}
		else if(revIt->second > 1){
				location="Obstacle on right";
			}
		}

	}

	return location;
}

MatrixXd pinv(MatrixXd& m, double epsilon = 1E-9)
{
  typedef JacobiSVD<MatrixXd> SVD;
  SVD svd(m, ComputeFullU | ComputeFullV);
  typedef SVD::SingularValuesType SingularValuesType;
  const SingularValuesType singVals = svd.singularValues();
  SingularValuesType invSingVals = singVals;
  for(int i=0; i<singVals.rows(); i++) {
    if(singVals(i) <= epsilon) {
      invSingVals(i) = 0.0; // FIXED can not be safely inverted
    }
    else {
      invSingVals(i) = 1.0 / invSingVals(i);
    }
  }
  return MatrixXd(svd.matrixV() *
      invSingVals.asDiagonal() *
      svd.matrixU().transpose());
}

void FlowVector::detected_obstacle(){
	_obstacle_location.clear();
	//int _count=1;
	//obstacles_detected.first=0;
	//obstacles_detected.second=0;
	VideoCapture cap(0);
	// Check if camera opened successfully
	if(!cap.isOpened())
		{
			cout << "Error opening camera";
			exit(0);
		}
	while(1){
			Mat  frame, grey, left, right;
			cap >> frame;
			cvtColor(frame, grey, CV_BGR2GRAY);
			Rect left_ROI(0, 0, grey.cols/2, grey.rows);
			Rect right_ROI(grey.cols/2,0,grey.cols/2, grey.rows);
			left=grey(left_ROI);
			right=grey(right_ROI);
			_locMtx.lock();
			if(_count > 1)
				{
				obstacles_detected.first = optic_vector(left_prev,left);
				obstacles_detected.second = optic_vector(right_prev,right);
				}
			_obstacle_location.push_back(obstacles_detected);
			_locMtx.unlock();
			left_prev=left;
			right_prev=right;
			_count++;
			//cout<<obstacles_detected.first<<endl;
			//cout<<obstacles_detected.second<<endl;
      String leftText = "Obstacle on the left";
      String rightText = "Obstacle on the right";
			putText(left, to_string(obstacles_detected.first), cvPoint(left.cols/2, left.rows/2),
			        FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(255, 255, 0), 1, CV_AA);
			putText(right, to_string(obstacles_detected.second), cvPoint(right.cols/2, right.rows/2),
					        FONT_HERSHEY_SIMPLEX, 0.8, cvScalar(255, 255, 0), 1, CV_AA);

			imshow("left", left);
			imshow("right", right);

		    char c = (char) waitKey(1);
		    if (c == 27)
		    	break;
	}
	//return obstacles_detected;
}

int optic_vector(Mat first_image_in,Mat second_image_in)
{
  Mat first_image;
  Mat second_image;
  float vec_threshold=50;
  int n=11;
  GaussianBlur(first_image_in,first_image,Size(n,n),0,0);
  GaussianBlur(second_image_in,second_image,Size(n,n),0,0);
  int obstacles=0;
  Mat optic_image;
  cvtColor(first_image,optic_image, CV_GRAY2RGB);
  int line_thickness=1;
  cv::Scalar line_color=CV_RGB(64, 64, 255);
  int maxCorners=100;
  std::vector<cv::Point2f> corners;
  corners.reserve(maxCorners);
  goodFeaturesToTrack(first_image,corners,maxCorners,0.01,5);
 for(int feature_num=0;feature_num<maxCorners;feature_num++)
 {
    int i=corners[feature_num].x;
    int j=corners[feature_num].y;
    MatrixXd A(2,n*n);
    MatrixXd B(n*n,1);
    int temp_counter=0;
    for(int k=0;k<n;k++)
    {
        int row_marker=k-1;
        for(int l=0;l<n;l++)
        {
          int col_marker=l-1;
          float x_d=(first_image.at<uchar>(i-1+row_marker,j+col_marker)-first_image.at<uchar>(i+1+row_marker,j+col_marker))/2;
          float y_d=(first_image.at<uchar>(i+row_marker,j-1+col_marker)-first_image.at<uchar>(i+row_marker,j+1+col_marker))/2;

          A(0,temp_counter)=x_d;
          A(1,temp_counter)=y_d;
          B(temp_counter,0)=second_image.at<uchar>(i+row_marker,j+col_marker)-first_image.at<uchar>(i+row_marker,j+col_marker);//time_derivative.at<uchar>(i+row_marker,j+col_marker);
          temp_counter++;
        }
      }

      MatrixXd flow_matrix=A*A.transpose();
      flow_matrix=pinv(flow_matrix)*A*B;
      int force=abs(flow_matrix(0,0))+abs(flow_matrix(1,0));

      if(force>vec_threshold)
      {
        std::cout<<"okokok"<<std::endl;

    	  CvPoint p,q;
    	        p.y=i;
    	        p.x=j;
    	        q.y=flow_matrix(0,0)+i;
    	        q.x=flow_matrix(1,0)+j;
    	        double angle;
    	        angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
    	        double hypotenuse;  hypotenuse = sqrt(pow((p.y - q.y),2) +pow((p.x - q.x),2));
    	        q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
    	        q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
    	        line( optic_image, p, q, line_color, line_thickness, CV_AA, 0 );
    	        p.x = (int) (q.x + 9 * cos(angle + PI / 4));
    	        p.y = (int) (q.y + 9 * sin(angle + PI / 4));
    	        line( optic_image, p, q, line_color, line_thickness, CV_AA, 0 );
    	        p.x = (int) (q.x + 9 * cos(angle - PI / 4));
    	        p.y = (int) (q.y + 9 * sin(angle - PI / 4));
    	        line( optic_image, p, q, line_color, line_thickness, CV_AA, 0 );

    	        obstacles++;
    	  }
        imshow("optical", optic_image);
  }
return obstacles;
}





