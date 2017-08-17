/*************************************************************************
	> File Name: simple_play_img.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年08月17日 星期四 15时03分06秒
 ************************************************************************/

#include<iostream>
#include<fstream>    // ifstream
#include<string>
#include<vector>

#include<cstdlib>
        // system

#include <glog/logging.h>
#include <gflags/gflags.h>

#include <opencv2/highgui/highgui.hpp>
        // imread | imshow

using namespace std;
using namespace cv;


DEFINE_double(frameRate, 1000/30, "frame rate of images");
DEFINE_string(stereoDir, "./", "the image list's folder");

bool ReadImgList(std::string imgFolder, std::string& img_prefix, std::vector<string>& imgList) {
  const string extract_img_name = "ls "+imgFolder+" > img_name_list.txt";
  std::cout<<"extract_img_name: "<<extract_img_name<<std::endl;
  if(system(extract_img_name.c_str()) < 0) {
    std::cout<<"cmd execute failure!"<<std::endl;
  }

  string imgName;
  std::ifstream txtFile("./img_name_list.txt");
  while(getline(txtFile, imgName)) {
    std::cout<<"imgName: "<<imgName<<std::endl;
    if(!imgName.compare(0, img_prefix.size(), img_prefix)) {
      CHECK_EQ(imgName.size(), img_prefix.size() + 13 + 4);  // 13-bit timestamp + ".png"
      imgList.push_back(imgName);
    }
  }

  return true;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_stereoDir.empty())<<"please set image dir";

  std::string img_prefix("stereo_img_l_");
  std::vector<std::string> imgList;
  bool result = false;
  result = ReadImgList(FLAGS_stereoDir, img_prefix, imgList);
  CHECK_EQ(result, true);

  std::cout<<"imgList.size(): "<<imgList.size()<<std::endl;
  CHECK_EQ(imgList.size(), 40);

  Mat curImg;
  bool bHitPause = false;
  bool bQuitDisplay = false;
  for(unsigned int i = 0; ((i < imgList.size()) && !bQuitDisplay); i++) {
    string imgPath = FLAGS_stereoDir + "/" + imgList[i];
    std::cout<<"imgPath: "<<imgPath<<std::endl;
    curImg = imread(imgPath);
    imshow("mono", curImg);
    char c;
    if(bHitPause) {
      c = waitKey(0);
    } else {
      c = waitKey(FLAGS_frameRate);
    }
    switch(c) {
      case 'p':  // pause 
      {
        bHitPause = true;
        break;
      }
      case 'c':
      {
        bHitPause = false;
        break;
      }
      case 'l':
      {
        i -= 2;
        break;
      }
      case 0x1B:
      case 'q':
      {
        bQuitDisplay = true;
        break;
      }
    }
  }

  return 0;
}
