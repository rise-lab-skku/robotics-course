#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "turtle_control/kf.h"

class SensorModelAnalysis
{
public:
    SensorModelAnalysis() {}
    ~SensorModelAnalysis() {}
    //! @brief ros publisher와 subscriber를 초기화한다.
    //!
    //! @param[in] nh 노드 핸들
    void Init(ros::NodeHandle &nh)
    {
        // laser scan 메시지를 구독한다.
        sub_scan_ = nh.subscribe("/scan", 3, &SensorModelAnalysis::LaserScanCallback, this);
        r_sum_ = 0.0;
        r_sq_sum_ = 0.0;
        r_count_ = 0;
    }
    //! @brief laser scan callback
    //!
    //! @param[in] msg 구독자가 수신한 laser scan 메시지
    void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // 측정할 물체의 roi를 박스로 설정한다.
        const float &roi_x_min = 0.10;  // x축 최소
        const float &roi_x_max = 1.0;   // x축 최대
        const float &roi_y_min = -0.40; // y축 최소
        const float &roi_y_max = 0.40;  // y축 최대

        // 측정한 물체의 laser scan의 포인트들의 x를 통계낸다.
        float x_sum = 0;        // 합
        float x_sq_sum = 0;     // 제곱의 합
        unsigned int count = 0; // 통계낸 x의 개수
        float r_min = std::numeric_limits<float>::max();
        bool detected = false;
        // laser scan은 z축 기준으로 회전하면서 x축부터 angle이 0 radian으로 시작, 반시계로 돌아 한바퀴 돌면 2 pi radian이 된다.
        float ang = msg->angle_min;
        for (auto &r : msg->ranges)
        {
            // 측정한 angle 에서 range를 이용, point의 좌표를 얻는다.
            const float &x = r * std::cos(ang);
            const float &y = r * std::sin(ang);

            // roi 바깥의 포인트는 필요 없으니 건너뛴다.
            if (x < roi_x_min || x > roi_x_max || y < roi_y_min || y > roi_y_max)
            {
                continue;
            }

            r_min = r < r_min ? r : r_min;
            detected = true;
            ang += msg->angle_increment; // 다음 angle로 업데이트
        }
        // 측정이 이루어지지 않은 것. devide by zero 회피
        if (!detected)
        {
            ROS_INFO("no detection.");
            return;
        }

        // 통계를 마무리 한다.
        r_sum_ += r_min;
        r_sq_sum_ += r_min * r_min;
        r_count_ += 1;
        if (r_count_ > 1)
        {
            const float &mean = r_sum_ / r_count_;
            const float &var = r_sq_sum_ / r_count_ - (mean * mean);
            // 화면에 출력
            ROS_INFO("count: %d, mean: %f, var: %f", r_count_, mean, var);
        }
    }

private:
    float r_sum_;
    float r_sq_sum_;
    unsigned int r_count_;

    ros::Subscriber sub_scan_;
};

int main(int argc, char *argv[])
{
    // 노드 초기화
    ros::init(argc, argv, "turtle_sensor_model_analysis");
    // 노드 핸들을 private namespace로 초기화
    ros::NodeHandle nh("~");
    // 생성
    SensorModelAnalysis ma;
    // 초기화
    ma.Init(nh);
    // 콜백 작동 시작
    ros::spin();
    return 0;
}
