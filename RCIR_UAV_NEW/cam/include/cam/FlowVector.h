/*
 * FlowVector.h
 *
 *  Created on: Mar 14, 2019
 *      Author: bro
 */

#ifndef FLOWVECTOR_H_
#define FLOWVECTOR_H_


#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <highgui.h>
#include <cv.h>
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <mutex>

#define PI 3.14


using namespace std;
using namespace cv;
using namespace Eigen;


int optic_vector(Mat first_image_in,Mat second_image_in);
MatrixXd pinv(MatrixXd& m, double );


class FlowVector {
public:
	void detected_obstacle();
	float Square(float x);
	String identify_location();

private:
	//vector<pair<int,int>> _obstacle_location;
	mutex _locMtx;



};

#endif /* FLOWVECTOR_H_ */
