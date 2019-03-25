/*================= include ====================================================*/
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <string>
#include <opencv2/rgbd.hpp>
#include <opencv2/rgbd/linemod.hpp>


/*= global variables ===========================================================*/
cv::Ptr<cv::linemod::Detector> detector= cv::linemod::getDefaultLINEMOD();
std::vector<std::string> FILENAME;


/*= static func ================================================================*/
static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename);
void drawResponse(const std::vector<cv::linemod::Template>& templates,
		  int num_modalities,
		  cv::Mat& display,
		  cv::Mat& display_draw,
		  cv::Point offset,
		  int T);


/*= class rosLinemod ===========================================================*/
class rosLinemod
{
private:
  int matching_threshold = 80;
  geometry_msgs::PoseStamped obj_pose_;
  geometry_msgs::Quaternion quat_pose;
  cv::Mat depth;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_rgb_image_;
  image_transport::Subscriber sub_depth_image_;
  ros::Publisher obj_pose_pub_;

  ros::NodeHandle nh;
  ros::Publisher obj_num_pub;

  void depthCb(const sensor_msgs::ImageConstPtr& depth_image);
  void imageCb(const sensor_msgs::ImageConstPtr& rgb_image);

public:
  rosLinemod();
  ~rosLinemod();
};

rosLinemod::rosLinemod() : nh_(), it_(nh_)
{
  sub_rgb_image_ = it_.subscribe("/camera/rgb/image_raw",1, &rosLinemod::imageCb, this);
  sub_depth_image_ = it_.subscribe("/camera/depth_registered/image_raw",1, &rosLinemod::depthCb, this);
  obj_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/linemod/obj_pose", 1);
  obj_pose_.pose.position.x = 0.0;
  obj_pose_.pose.position.y = 0.0; 
  obj_pose_.pose.position.z = 0.0;
  obj_pose_.pose.orientation.x = 0.0;
  obj_pose_.pose.orientation.y = 0.0; 
  obj_pose_.pose.orientation.z = 0.0;
  obj_pose_.pose.orientation.w = 0.0;
}

rosLinemod::~rosLinemod(){}


/*- func drawResponse -----------------------------------------------------------*/
void drawResponse(const std::vector<cv::linemod::Template>& templates,
		  int num_modalities,
		  cv::Mat& display,
		  cv::Mat& display_draw,
		  cv::Point offset,
		  int T)
{
  static const cv::Scalar COLORS[5] = {CV_RGB(0, 255, 0), CV_RGB(0, 0, 255)};

  for(int m = 0; m < num_modalities; ++m)
    {
    cv::Scalar color = COLORS[m];
    /* 検出したオブジェクトの概形を矩形で囲う */
    // cv::rectangle(display,
    // 		  cv::Point(offset.x, offset.y),
    // 		  cv::Point(offset.x+templates[m].width,offset.y+templates[m].height),
    // 		  CV_RGB(255,255,255),
    // 		  -1,
    // 		  CV_AA);

    /* 検出したオブジェクトの中心に円を描く  */
    cv::Point obj_center(offset.x+templates[m].width/2, offset.y+templates[m].height/2);
    cv::circle(display, obj_center, 2 , cv::Scalar(0,0,255), -1); //円の描画

    for (int i = 0; i < (int)templates[m].features.size(); ++i)
      {   
      /* 複数の小さい円で検出したオブジェクトをかたどる */
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);

    }
  }
}


/*- func readLinemod -----------------------------------------------------------*/
static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename){
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());
  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}


/*- func depthCb ---------------------------------------------------------------*/
void rosLinemod::depthCb(const sensor_msgs::ImageConstPtr& depth_image)
{
  cv_bridge::CvImagePtr cv_depth;
  try{
    cv_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  depth = cv_depth->image;
}


/*- func imageCb ---------------------------------------------------------------*/
void rosLinemod::imageCb(const sensor_msgs::ImageConstPtr& rgb_image)
{
  int num_modalities;
  num_modalities = (int)detector->getModalities().size();

  cv_bridge::CvImagePtr cv_rgb;
  try {
    cv_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  cv::Mat color = cv_rgb->image;
  cv::Mat display = color.clone();
  cv::Mat display_draw = color.clone();
  std::vector<cv::Mat> sources;
  sources.push_back(color);

  std::vector<cv::linemod::Match> matches;
  std::vector<cv::String> class_ids;
  std::vector<cv::Mat> quantized_images;

  std::cout << "imageCb" << std::endl;

  detector->match(sources, (float)matching_threshold,
  		  matches, class_ids,  quantized_images);

  if(matches.size() > 0){
    cv::linemod::Match m = matches[0];
    const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);
    
    double roll, pitch, yaw, q_x, q_y, q_z, q_w = {0.0};
    
    std::cout << "similarity: " << m.similarity << std::endl;
    std::cout << "template_id: " << m.template_id << std::endl;

    drawResponse(templates, num_modalities, display,
		 display_draw, cv::Point(m.x, m.y), detector->getT(0));
    sources.clear();
    sources.shrink_to_fit();
    sources.push_back(display);

    std::string similarity = std::to_string((int)m.similarity)+"%";
    cv::putText(display, similarity,
		cv::Point2i(m.x,m.y),
		cv::FONT_HERSHEY_SIMPLEX, 1,
		cv::Scalar(0,0,0), 2);

    cv::imshow("color",display);
    cv::waitKey(10);
  }else{
    std::cout << "match error" << std::endl;
  }
}




/*= main =======================================================================*/
int main(int argc, char** argv){
  std::cout << "ok" << std::endl;
  ros::init(argc, argv, "linemod_abe");

  cv::namedWindow("color", CV_WINDOW_NORMAL);

  detector = readLinemod(argv[1]);
  FILENAME.push_back(argv[1]);
  
  rosLinemod lm;
  ros::spin();
  
  return 0;
}
