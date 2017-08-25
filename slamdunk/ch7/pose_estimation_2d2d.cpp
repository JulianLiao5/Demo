/*************************************************************************
	> File Name: pose_estimation_2d2d.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年08月25日 星期五 17时29分19秒
 ************************************************************************/

#include<iostream>

#include <opencv2/highgui/highgui.hpp>
        // imread

using namespace std;
using namespace cv;


void find_feature_matches(const Mat& img_1, const Mat& img_2, vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, vector<DMatch> matches);

bool compareDescriptorDistance(DMatch& match1, DMatch& match2) {
  return match1.distance < match2.distance;
}


void find_feature_matches(const Mat img_1, const Mat img_2, vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_1, vector<DMatch> matches) {

  Mat descriptor_1, descriptor_2;

  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor =  ORB::create();

  Ptr<DescriptorMatcher> matcher = DescriptorMatcher.create("BruteForce-Hamming");
  // 第一步：检测角点位置
  detector.detect(img_1, keypoints_1);
  detector.detect(img_2, keypoints_2);

  // 第二步：根据角点位置计算BRIEF描述子
  descriptor.compute(img_1, keypoints_1, descriptor_1);
  descriptor.compute(img_2, keypoints_2, descriptor_2);


  vector<DMatch> all_matches;
  // 第三步：对两帧图像的描述子进行匹配，并得到所有匹配对的汉明距离
  matcher.match(descriptor_1, descriptor_2, all_matches);

  // std::min_element
  // std::max_element
  // 第四步：找出所有匹配对的最大距离和最小距离
  double min_dist = 1000, max_dist;
  min_dist = std::min_element(all_matches.begin(), all_matches.end(), compareDescriptorDistance).distance;
  max_dist = std::max_element(all_matches.begin(), all_matches.end(), compareDescriptorDistance).distance;

  cout<<"min_dist: "<< min_dist<<", max_dist: "<<max_dist<<endl;

  // 当描述子之间的距离大于两倍的最小距离时，即认为匹配有误，但有时最小距离会非常小，因此，取经验值30作为下限
  for(int i = 0; i < descriptor_1.rows; i++) {
    double dist = all_matches[i].distance;
    if(dist < std::max(30, 2*min_dist)) {
      matches.push_back(all_matches[i]);
    }
  }
  cout<<"matches.size(): "<<matches.size()<<endl;
}


void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1, std::vector<KeyPoint> keypoints_2, std::vector<DMatch> matches, Mat& R, Mat& t) {
  // 相机内参
  Mat k = (Matx33d << 334.53272577769917, 0, 305.58190566496421, 0, 332.43734869658425, 230.98113455840436, 0, 0, 1);

  // 把匹配点转换为vector<Point2f>的形式
  // 一边的数据放的是KeyPoint，脚点，一边的数据放的是x,y，以Pixel为单位
  vector<Point2f> points1;
  vector<Point2f> points2;

  for(int i = 0; i < matches.size(); i++) {
    points1.push_back(keypoints_1[matches[i].queryIdx].pt);
    points2.push_back(keypoints_2[matches[i].trainIdx].pt);
  }

  // 计算基础矩阵
  Mat fundamental_matrix;
  fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
  cout << "fundamental_matrix: " << endl << fundamental_matrix << endl;

  // 计算本质矩阵
  Point2d principal_point(305.58190566496421, 230.98113455840436);    // 主点，光心
  int focal_length = 334;    // 焦距
  Mat essential_matrix;
  essential_matrix = findEssentialMat(points1, points2, k, principal_point, RANSAC);
  cout<<"essential_matrix: " << endl << essential_matrix << endl;

  // 计算单应矩阵
  Mat homography_matrix;
  homography_matrix = findHomography(points1, points2, RANSAC, 3, noArray(), 2000, 0.99);
  cout<<"homography_matrix: "<<endl<<homography_matrix<<endl;

  // 从本质矩阵中恢复旋转和平移信息
  recoverPose(fundamental_matrix, points1, points2, k, R, t);
  cout<<"R is "<< endl << R << endl;
  cout<<"t is " << endl << t << endl;
}


Point2d pixel2cam(const Point2d& p, const Mat& K) {
  return Point2d((p.x - K.at<double>(0,2))/K.at<double>(0,0), (p.y - K.at<double>(1,2))/K.at<double>(1,1));
}

int main(int argc, char *argv[]) {
  if(argc != 3) {
    cout<<"Usage: %s img1 img2"<<endl;
    return 1;
  }

  Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
  
  vector<KeyPoint> keypoints_1;
  vector<KeyPoint> keypoints_2;
  vector<DMatch> good_matches;

  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, good_matches);
  cout<<"一共找到了"<<good_matches.size()<<"对匹配点"<<endl;

  // 估计两帧图像间的运动
  Mat R,t;
  pose_estimation_2d2d(keypoints_1, keypoints_2, good_matches, R, t);

  // 验证 E=t^R*scale，E是本质矩阵
  Mat t_x = Matx33d <<
      0,                 -t.at<double>(2,0),    t.at<double>(1,0),
    t.at<double>(2,0)    0,                     -t.at<double>(0,0),
    -t.at<double>(1, 0)  t.at<double>(0, 0),    0;
  cout<<"t^R="<<endl<<t_x*R<<endl;

  // 验证对极约束
  // 内参矩阵
  Mat k = (Matx33d << 334.53272577769917, 0, 305.58190566496421, 0, 332.43734869658425, 230.98113455840436, 0, 0, 1);
  for(DMatch m : matches) {
    Point2d pt1 = pixel2cam(keypoints_1[m.queryIndex].pt, K);
    Mat y1 = Mat3d << pt1.x, pt1.y, 1;
    Point2d pt2 = pixel2cam(keypoints_2[m.trainIndex].pt, K);
    Mat y2 = Mat3d << pt2.x, pt2.y, 1;
    Mat d = y2.t() * t_x * R * y1;
    cout<<"epipolar constraint = "<< d << endl;
  }

  return 0;
}
