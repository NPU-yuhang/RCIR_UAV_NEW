#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

Mat cur_frame, pre_frame;
bool has_got_first_frame = false;
Ptr<ORB> orb = ORB::create();

int main( int argc, char **argv )
{
  ros::init( argc, argv, "pub_cam_node" );
  ros::NodeHandle n;

  // Open camera with CAMERA_INDEX (webcam is typically #0).
  cv::VideoCapture capture( "/media/nvidia/SD1/catkin_m100/src/cam/video/VID_20190930_080650.mp4" ); //摄像头视频的读操作

  if( not capture.isOpened() )
  {
    ROS_ERROR_STREAM(
      "Failed to open camera with index " << "/media/nvidia/SD/flyvideo/2222.avi" << "!"
    );
    ros::shutdown();
  }

  //1 捕获视频

  //2 创建ROS中图像的发布者

  cv::Size dsize = cv::Size(800, 450);
  cv::Mat cur_frame = cv::Mat(dsize, CV_32S);

  while( ros::ok() ) {
    cv::Mat cap;
    capture >> cap; //流的转换
    cv::resize(cap, cur_frame, dsize);

    if(has_got_first_frame)
    {
        //初始化
        //首先创建两个关键点数组，用于存放两张图像的关键点，数组元素是KeyPoint类型
        std::vector<KeyPoint> keypoints_1, keypoints_2;

        //创建两张图像的描述子，类型是Mat类型
        Mat descriptors_1, descriptors_2;

        //创建一个ORB类型指针orb，ORB类是继承自Feature2D类
        //class CV_EXPORTS_W ORB : public Feature2D
        //这里看一下create()源码：参数较多，不介绍。
        //creat()方法所有参数都有默认值，返回static　Ptr<ORB>类型。
        /*
        CV_WRAP static Ptr<ORB> create(int nfeatures=500,
                                       float scaleFactor=1.2f,
                                       int nlevels=8,
                                       int edgeThreshold=31,
                                       int firstLevel=0,
                                       int WTA_K=2,
                                       int scoreType=ORB::HARRIS_SCORE,
                                       int patchSize=31,
                                       int fastThreshold=20);
        */
        //所以这里的语句就是创建一个Ptr<ORB>类型的orb，用于接收ORB类中create()函数的返回

        //第一步：检测Oriented FAST角点位置.
        //detect是Feature2D中的方法，orb是子类指针，可以调用
        //看一下detect()方法的原型参数：需要检测的图像，关键点数组，第三个参数为默认值
        /*
        CV_WRAP virtual void detect( InputArray image,
                                     CV_OUT std::vector<KeyPoint>& keypoints,
                                     InputArray mask=noArray() );
        */
        orb->detect(pre_frame, keypoints_1);
        orb->detect(cur_frame, keypoints_2);


        //第二步：根据角点位置计算BRIEF描述子
        //compute是Feature2D中的方法，orb是子类指针，可以调用
        //看一下compute()原型参数：图像，图像的关键点数组，Mat类型的描述子
        /*
        CV_WRAP virtual void compute( InputArray image,
                                      CV_OUT CV_IN_OUT std::vector<KeyPoint>& keypoints,
                                      OutputArray descriptors );
        */
        orb->compute(pre_frame, keypoints_1, descriptors_1);
        orb->compute(cur_frame, keypoints_2, descriptors_2);

        //定义输出检测特征点的图片。
        Mat outimg1;
        //drawKeypoints()函数原型参数：原图，原图关键点，带有关键点的输出图像，后面两个为默认值
        /*
        CV_EXPORTS_W void drawKeypoints( InputArray image,
                                         const std::vector<KeyPoint>& keypoints,
                                         InputOutputArray outImage,
                                         const Scalar& color=Scalar::all(-1),
                                         int flags=DrawMatchesFlags::DEFAULT );
        */
        //注意看，这里并没有用到描述子，描述子的作用是用于后面的关键点筛选。
        drawKeypoints(pre_frame, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

        imshow("ORB",outimg1);


        //第三步：对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离

        //创建一个匹配点数组，用于承接匹配出的DMatch，其实叫match_points_array更为贴切。matches类型为数组，元素类型为DMatch
        vector<DMatch> matches;

        //创建一个BFMatcher匹配器，BFMatcher类构造函数如下：两个参数都有默认值，但是第一个距离类型下面使用的并不是默认值，而是汉明距离
        //CV_WRAP BFMatcher( int normType=NORM_L2, bool crossCheck=false );
        BFMatcher matcher (NORM_HAMMING);

        //调用matcher的match方法进行匹配,这里用到了描述子，没有用关键点。
        //匹配出来的结果写入上方定义的matches[]数组中
        matcher.match(descriptors_1, descriptors_2, matches);

        //第四步：遍历matches[]数组，找出匹配点的最大距离和最小距离，用于后面的匹配点筛选。
        //这里的距离是上方求出的汉明距离数组，汉明距离表征了两个匹配的相似程度，所以也就找出了最相似和最不相似的两组点之间的距离。
        double min_dist=0, max_dist=0;//定义距离

        for (int i = 0; i < descriptors_1.rows; ++i)//遍历
        {
            double dist = matches[i].distance;
            if(dist<min_dist) min_dist = dist;
            if(dist>max_dist) max_dist = dist;
        }

        printf("Max dist: %f\n", max_dist);
        printf("Min dist: %f\n", min_dist);

        //第五步：根据最小距离，对匹配点进行筛选，
        //当描述自之间的距离大于两倍的min_dist，即认为匹配有误，舍弃掉。
        //但是有时最小距离非常小，比如趋近于0了，所以这样就会导致min_dist到2*min_dist之间没有几个匹配。
        // 所以，在2*min_dist小于30的时候，就取30当上限值，小于30即可，不用2*min_dist这个值了
        std::vector<DMatch> good_matches;
        for (int j = 0; j < descriptors_1.rows; ++j)
        {
            if ((matches[j].distance <= 0.8*max_dist) && (matches[j].distance > 100))
            {
              std::cout<<"okokok: "<<matches[j].distance<<std::endl;
              good_matches.push_back(matches[j]);
            }

        }
        std::cout<<good_matches.size()<<std::endl;

        vector<DMatch> m_Matches=good_matches;
        int ptCount = (int)m_Matches.size();
        Mat p1(ptCount, 2, CV_32F);
        Mat p2(ptCount, 2, CV_32F);

        // 把Keypoint转换为Mat
        Point2f pt;
        for (int i=0; i<ptCount; i++)
        {
          pt = keypoints_1[m_Matches[i].queryIdx].pt;
          p1.at<float>(i, 0) = pt.x;
          p1.at<float>(i, 1) = pt.y;

          pt = keypoints_2[m_Matches[i].trainIdx].pt;
          p2.at<float>(i, 0) = pt.x;
          p2.at<float>(i, 1) = pt.y;
        }

        // 用RANSAC方法计算F
        Mat m_Fundamental;
        vector<uchar> m_RANSACStatus;       // 这个变量用于存储RANSAC后每个点的状态
        //findFundamentalMat(p1, p2, m_RANSACStatus, FM_RANSAC);

        if(good_matches.size() > 30)
        {
          //第六步：绘制匹配结果

          Mat img_match;//所有匹配点图
          //这里看一下drawMatches()原型参数，简单用法就是：图1，图1关键点，图2，图2关键点，匹配数组，承接图像，后面的有默认值
          /*
          CV_EXPORTS_W void drawMatches( InputArray img1,
                                         const std::vector<KeyPoint>& keypoints1,
                                         InputArray img2,
                                         const std::vector<KeyPoint>& keypoints2,
                                         const std::vector<DMatch>& matches1to2,
                                         InputOutputArray outImg,
                                         const Scalar& matchColor=Scalar::all(-1),
                                         const Scalar& singlePointColor=Scalar::all(-1),
                                         const std::vector<char>& matchesMask=std::vector<char>(),
                                         int flags=DrawMatchesFlags::DEFAULT );
          */

          drawMatches(pre_frame, keypoints_1, cur_frame, keypoints_2, matches, img_match);
          imshow("pipei", img_match);

          Mat img_goodmatch;//筛选后的匹配点图
          drawMatches(pre_frame, keypoints_1, cur_frame, keypoints_2, good_matches, img_goodmatch);
          imshow("saisuan", img_goodmatch);
          cur_frame.copyTo(pre_frame);
        }
        waitKey(1);

    }
    else
    {
        cur_frame.copyTo(pre_frame);
        has_got_first_frame = true;
    }

    cv::waitKey( 1 );//opencv刷新图像 3ms
    ros::spinOnce();
    //loop_rate.sleep();
  }

  capture.release();  //释放流
  return EXIT_SUCCESS;
}
