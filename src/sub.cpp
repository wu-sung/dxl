#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "dxl/dxl.hpp"
#include <memory>
#include <functional>
#include "sensor_msgs/msg/compressed_image.hpp"  // 압축된 이미지 메시지 타입 포함
#include "opencv2/opencv.hpp"  // OpenCV 라이브러리 포함
#include "cv_bridge/cv_bridge.h"  // OpenCV와 ROS 메시지 간의 브릿지 포함
#include <memory>  // std::shared_ptr 사용을 위한 헤더 포함
#include <signal.h>  // 신호 처리를 위한 헤더 포함

using namespace std::placeholders;

cv::VideoWriter writer;  // VideoWriter 전역 변수 선언
bool ctrl_c_pressed = false;  // Ctrl+C 플래그

// Ctrl+C 핸들러 함수
void ctrlc_handler(int)
{
    ctrl_c_pressed = true;
}

// 콜백 함수: 메시지를 수신했을 때 호출됨
void mysub_callback2(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 압축된 이미지 메시지를 OpenCV 행렬로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (!frame.empty()) {  // 프레임이 비어 있지 않다면
        cv::imshow("Received Image", frame);  // 프레임을 창에 표시
        cv::waitKey(1);  // 키 입력 대기 (1밀리초)

        // 프레임을 동영상 파일에 저장
        writer.write(frame);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Empty frame");  // 오류 메시지 출력
    }
}

void mysub_callback(rclcpp::Node::SharedPtr node, Dxl& dxl, const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Received message: %lf, %lf", msg->x, msg->y);
    dxl.setVelocity((int)msg->x, (int)msg->y);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    Dxl dxl;
    auto node = std::make_shared<rclcpp::Node>("node_camsub");
    auto cam_node = std::make_shared<rclcpp::Node>("camsub");  // 노드 생성

    // Ctrl+C 신호 핸들러 설정
    signal(SIGINT, ctrlc_handler);

    // VideoWriter 객체 초기화
    writer.open("output.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30, cv::Size(640, 360));
    if (!writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open VideoWriter");
        rclcpp::shutdown();
        return -1;
    }

    if(!dxl.open())
    {
        RCLCPP_ERROR(node->get_logger(), "Dynamixel open error");
        rclcpp::shutdown();
        return -1;
    } 

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    std::function<void(const geometry_msgs::msg::Vector3::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, dxl, _1);
    
    auto mysub = node->create_subscription<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile, fn);
    // 서브스크립션 생성 및 콜백 함수 설정
    auto cam_mysub = cam_node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, mysub_callback2);
    
    rclcpp::spin(node);
    // 노드 스핀 (콜백 함수 실행)
    rclcpp::spin_some(cam_node);

    writer.release();
    cv::destroyAllWindows();
    dxl.close();
    rclcpp::shutdown();
    return 0;
}