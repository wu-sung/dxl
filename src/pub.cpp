#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "dxl/dxl.hpp"
#include <memory>
#include "sensor_msgs/msg/compressed_image.hpp"  // CompressedImage 메시지를 사용하기 위해 포함
#include "cv_bridge/cv_bridge.h"  // OpenCV와 ROS 메시지 간 변환을 위해 포함
#include "opencv2/opencv.hpp"  // OpenCV 라이브러리를 포함
#include <chrono>

std::string src = "nvarguscamerasrc sensor-id=0 ! \
    video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink"; 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_dxlpub");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile );
    geometry_msgs::msg::Vector3 vel;

    auto cam_node = std::make_shared<rclcpp::Node>("campub");  // 새로운 노드 생성
    auto cam_mypub = cam_node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);
    // 퍼블리셔 생성(메세지 형태)

    std_msgs::msg::Header hdr;  // 메시지 헤더 생성
    sensor_msgs::msg::CompressedImage::SharedPtr msg;  // CompressedImage 메시지 포인터 생성

    cv::VideoCapture cap(src, cv::CAP_GSTREAMER);  // GStreamer를 사용하여 비디오 캡처 생성
    if (!cap.isOpened()) {  // 비디오 캡처 열기 실패 시
        RCLCPP_ERROR(cam_node->get_logger(), "Video Missing!");  // 에러 로그 출력
        rclcpp::shutdown();  // ROS 2 종료
        return -1;  // 프로그램 종료
    }
    cv::Mat frame;  // 프레임을 저장할 변수

    vel.x = 0;
    vel.y = 0;
    vel.z = 0;
    rclcpp::WallRate loop_rate(20.0); //20hz->50msec
    int vel1=0,vel2=0;
    int goal1=0,goal2=0;
   
    while(rclcpp::ok())
    {
        cap >> frame;  // 비디오 캡처에서 프레임을 읽기
        if (frame.empty()) {  // 프레임이 비어 있으면
            RCLCPP_ERROR(cam_node->get_logger(), "frame empty");  // 에러 로그 출력
            break;  // 루프 종료
        }
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();  // 프레임을 CompressedImage 메시지로 변환
        cam_mypub->publish(*msg);  // 메시지 퍼블리시
        loop_rate.sleep();  // 루프 속도 유지
        if (Dxl::kbhit())
        {
            char c = Dxl::getch();
            switch(c)
            {
            case ' ': goal1 = 0; goal2 = 0; break;
            case 'f': goal1 = 50; goal2 = -50; break;
            case 's': goal1 = -50; goal2 = 50; break;
            case 'a': goal1 = -50; goal2 = -50; break;
            case 'd': goal1 = 50; goal2 = 50; break;
            default : goal1 = 0; goal2 = 0; break;
            }         
        }
        
        // generate accel and decel motion
        if(goal1>vel1) vel1 += 5;
        else if(goal1<vel1) vel1 -= 5;
        else vel1 = goal1;

        if(goal2>vel2) vel2 += 5;
        else if(goal2<vel2) vel2 -= 5;
        else vel2 = goal2;
        vel.x = vel1;
        vel.y = vel2;               
        
        RCLCPP_INFO(node->get_logger(), "Publish: %lf,%lf", vel.x, vel.y);
        mypub->publish(vel);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}