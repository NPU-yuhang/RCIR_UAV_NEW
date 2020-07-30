#include "cam/orb_detection.h"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "sparse_optical_flow_gpu");
    ros::NodeHandle nh;
    ros::start();

    image_transport::ImageTransport it( nh );
    pub_cluster_image = it.advertise( "/image/cluster", 1 );
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 50);
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera_BGR", 1);
    img_sub.registerCallback(image_callback);
    ros::spin();
    ros::shutdown();

    return 0;
}

void image_callback(const sensor_msgs::ImageConstPtr &img)
{
    cur_frame = cv_bridge::toCvShare(img, "bgr8")->image;
    orb->detect(cur_frame, keypoints_2);
    orb->compute(cur_frame, keypoints_2, descriptors_2);

    if(has_got_first_frame)
    {
        vector<DMatch> matches;
        BFMatcher matcher (NORM_HAMMING);
        matcher.match(descriptors_1, descriptors_2, matches);

        double min_dist=0, max_dist=0;//定义距离
        for (int i = 0; i < descriptors_1.rows; ++i)//遍历
        {
            double dist = matches[i].distance;
            if(dist<min_dist) min_dist = dist;
            if(dist>max_dist) max_dist = dist;
        }

        //printf("Max dist: %f\n", max_dist);
        //printf("Min dist: %f\n", min_dist);
        kp_frames.clear();
        std::vector<DMatch> good_matches;
        int img_dis_count = 0;
        for (int j = 0; j < descriptors_1.rows; ++j)
        {
            if (matches[j].distance <= 0.3*max_dist)
            {
              float dx = keypoints_1[matches[j].queryIdx].pt.x - keypoints_2[matches[j].trainIdx].pt.x;
              float dy = keypoints_1[matches[j].queryIdx].pt.y - keypoints_2[matches[j].trainIdx].pt.y;
              float distance = sqrt(dx*dx + dy*dy);
              //kp_frames.insert(pair<KeyPoint, KeyPoint>(keypoints_1[matches[j].queryIdx],keypoints_2[matches[j].trainIdx]));
              kp_frames[matches[j].queryIdx] = matches[j].trainIdx;

              good_matches.push_back(matches[j]);
              if(distance>25)
                img_dis_count++;
            }
        }

        if(img_dis_count > 25)
        {
          Mat R, t;
          pose_estimation_2d2d(keypoints_1, keypoints_2, good_matches, R, t);

          std::vector<Point3d> points;
          triangulation( keypoints_1, keypoints_2, good_matches, R, t, points );

          cloud.points.resize(points.size());

          myClusterAnalysis.clear();
          myClusterAnalysis.Init(points,6,3);
          myClusterAnalysis.DoDBSCANRecursive();
          has_got_cluster = true;

          float step = 255/(myClusterAnalysis.clusternum - 1);
          //std::cout<<"cluster num: "<<myClusterAnalysis.clusternum<<std::endl;
          point_cluster.clear();
          point_cluster_3d.clear();
          kps_1.clear();
          center_pts.clear();

          cur_frame.copyTo(pre_frame);
          keypoints_1 = keypoints_2;
          for(auto kp:keypoints_1)
            kps_1.push_back(kp.pt);

          descriptors_2.copyTo(descriptors_1);

          for(int i=0; i<points.size(); i++)
          {
            int cluster_id = myClusterAnalysis.dadaSets[i].GetClusterId();
            if (cluster_id != -1)
            {
              circle(cur_frame, keypoints_2[good_matches[i].trainIdx].pt, 3, Scalar(0, 255-cluster_id*step, cluster_id*step), -1, 8);
              cloud.points[i].x = points[i].x;
              cloud.points[i].y = points[i].y;
              cloud.points[i].z = points[i].z;
              cloud.points[i].r = cluster_id*step;
              cloud.points[i].g = 255 - cluster_id*step;
              cloud.points[i].b = 0;
            }
            else
            {
              cloud.points[i].x = points[i].x;
              cloud.points[i].y = points[i].y;
              cloud.points[i].z = points[i].z;
              cloud.points[i].r = 255;
              cloud.points[i].g = 255;
              cloud.points[i].b = 255;
            }
            //points_cluster.insert(pair<KeyPoint, int>(keypoints_2[good_matches[i].trainIdx], cluster_id
            point_cluster.push_back(pair<int, int>(good_matches[i].trainIdx, cluster_id));
            point_cluster_3d.push_back(pair<int, Point3d>(cluster_id, points[i]));
            //int cluster_id = Labels.at<int>(i, 0);
          }

          double min_dis = 9999;
          for(int j=0; j<myClusterAnalysis.clusternum; j++)
          {
            Point3d cpt;
            int count = 0;
            pair<int, Point3d> cpt_clus;
            for(int s=0; s<point_cluster_3d.size(); s++)
            {
              if(point_cluster_3d[s].first == j)
              {
                cpt.x = cpt.x+point_cluster_3d[s].second.x;
                cpt.y = cpt.y+point_cluster_3d[s].second.y;
                cpt.z = cpt.z+point_cluster_3d[s].second.z;
                count++;
              }
            }
            cpt.x = cpt.x/count;
            cpt.y = cpt.y/count;
            cpt.z = cpt.z/count;
            cpt_clus = pair<int, Point3d>(j, cpt);
            std::cout<<"x: "<<cpt.x<<"--y: "<<cpt.y<<"--z: "<<cpt.z<<std::endl;
            center_pts.push_back(cpt_clus);

            double dis = sqrt(cpt.x*cpt.x + cpt.y*cpt.y + cpt.z*cpt.z);
            if(dis<min_dis)
            {
              min_dis = dis;
              nearest_cluster = j;
            }

          }
          pcl::toROSMsg(cloud, output);

        }
        else
        {
          if(has_got_cluster)
          {
            vector<float> err;
            calcOpticalFlowPyrLK(pre_frame, cur_frame, kps_1, kps_2, status, err, winSize,
                                 3, termcrit, 0, 0.001);

            float step = 255/(myClusterAnalysis.clusternum - 1);
            for(int i=0; i<point_cluster.size(); i++)
            {
                //int cluster_id = points_cluster[good_matches[i].queryIdx];
                int cluster_id = point_cluster[i].second;
                if(cluster_id != -1)
                {
                  //std::cout<<"cluster_id: "<<cluster_id<<std::endl;
                  //KeyPoint kp2 = keypoints_2[kp_frames[point_cluster[i].first]];
                  Point2f kp2 = kps_2[point_cluster[i].first];
                  circle(cur_frame, kp2, 3, Scalar(0, 255-cluster_id*step, cluster_id*step), -1, 8);
                }
            }
          }

        }

        cluster_rects.clear();
        for(int i=0; i<myClusterAnalysis.clusternum; i++)
        {
          Point2f P_lu, P_rd;
          pair<Point2f, Point2f> rect;
          pair< int, pair<Point2f, Point2f> > clus_rect;
          float xmin = 999;
          float xmax = 0;
          float ymin = 999;
          float ymax = 0;
          for(int j=0; j<point_cluster.size(); j++)
          {
            if(point_cluster[j].second == i)
            {
              if(img_dis_count > 25)
              {
                if(keypoints_2[point_cluster[j].first].pt.x < xmin)
                  xmin = keypoints_2[point_cluster[j].second].pt.x;
                if(keypoints_2[point_cluster[j].first].pt.x > xmax)
                  xmax = keypoints_2[point_cluster[j].second].pt.x;
                if(keypoints_2[point_cluster[j].first].pt.y < ymin)
                  ymin = keypoints_2[point_cluster[j].first].pt.y;
                if(keypoints_2[point_cluster[j].first].pt.y > ymax)
                  ymax = keypoints_2[point_cluster[j].first].pt.y;
              }

              else
              {
                if(kps_2[point_cluster[j].first].x < xmin)
                  xmin = kps_2[point_cluster[j].first].x;
                if(kps_2[point_cluster[j].first].x > xmax)
                  xmax = kps_2[point_cluster[j].first].x;
                if(kps_2[point_cluster[j].first].y < ymin)
                  ymin = kps_2[point_cluster[j].first].y;
                if(kps_2[point_cluster[j].first].y > ymax)
                  ymax = kps_2[point_cluster[j].first].y;
              }
              P_lu = Point2f(xmin, ymin);
              P_rd = Point2f(xmax, ymax);
              rect = pair<Point2f, Point2f>(P_lu, P_rd);
              clus_rect = pair< int, pair<Point2f, Point2f> >(i, rect);
              cluster_rects.push_back(clus_rect);

            }
          }
          //std::cout<<"clus: "<<i<<"--xmin: "<<xmin<<"--xmax: "<<xmax<<"--ymin: "<<ymin<<"--ymax: "<<ymax<<std::endl;
          cv::Rect rectangle;
          rectangle.x = xmin;
          rectangle.y = ymin;
          rectangle.height = ymax - ymin;
          rectangle.width = xmax - xmin;
          if(nearest_cluster == i)
            cv::rectangle(cur_frame, rectangle, Scalar(0, 0, 255), 2);
        }

        output.header.stamp = ros::Time::now();
        output.header.frame_id = "odom";
        cloud_pub.publish(output);

        frame->image = cur_frame;
        frame->header.stamp = ros::Time::now();
        pub_cluster_image.publish(frame->toImageMsg());

        imshow("cluster", cur_frame);
        waitKey(1);

    }
    else
    {
        cur_frame.copyTo(pre_frame);
        keypoints_1 = keypoints_2;
        for(auto kp:keypoints_1)
          kps_1.push_back(kp.pt);

        descriptors_2.copyTo(descriptors_1);
        has_got_first_frame = true;
    }
}

void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t )
{
    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
    //cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    Point2d principal_point ( 400.735172, 308.768174 );	//相机光心, TUM dataset标定值
    double focal_length = 630.5;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    //cout<<"essential_matrix is "<<endl<<essential_matrix<<endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    //cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;

}

void triangulation (
    const vector< KeyPoint >& keypoint_1,
    const vector< KeyPoint >& keypoint_2,
    const std::vector< DMatch >& matches,
    const Mat& R, const Mat& t,
    vector< Point3d >& points )
{
    Mat T1 = (Mat_<float> (3,4) <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );

    Mat K = ( Mat_<double> ( 3,3 ) << 630.489651, 0, 400.735172, 0, 630.434369, 308.768174, 0, 0, 1 );
    vector<Point2f> pts_1, pts_2;
    for ( DMatch m:matches )
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back ( pixel2cam( keypoint_1[m.queryIdx].pt, K) );
        pts_2.push_back ( pixel2cam( keypoint_2[m.trainIdx].pt, K) );
    }

    Mat pts_4d;
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );

    // 转换成非齐次坐标
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        Point3d p (
            x.at<float>(0,0),
            x.at<float>(1,0),
            x.at<float>(2,0)
        );
        points.push_back( p );
    }
}

Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2f
    (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0),
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1)
    );
}
