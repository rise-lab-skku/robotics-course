#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "turtle_control/kf.h"
#include <limits>
class KalmanFilter1D
{
public:
    //! @brief 1D 칼만필터를 생성한다.
    //!
    //! @param[in] x 초기 상태
    //! @param[in] p 초기 상태 공분산
    KalmanFilter1D(float x, float p) : x_kf_(x), x_odom_(x), p_(p)
    {
    }
    //! @brief ros publisher와 subscriber를 초기화한다.
    //!
    //! @param[in] nh 노드 핸들
    void Init(ros::NodeHandle &nh)
    {
        // laser scan 메시지를 구독한다.
        sub_scan_ = nh.subscribe("/scan", 3, &KalmanFilter1D::LaserScanCallback, this);
        // odometry message를 구독한다.
        sub_odom_ = nh.subscribe("/odom", 3, &KalmanFilter1D::OdometryCallback, this);
        // 칼만필터 결과를 발행한다.
        pub_kf_ = nh.advertise<turtle_control::kf>("kf", 3);
        // 이전 측정 타임스탬프 초기화
        prev_prediction_ = ros::Time(0);

        enable_odom_ = true;
        enable_scan_ = false;
    }
    //! @brief 기본 파괴자
    ~KalmanFilter1D() {}
    //! @brief odometer로부터 칼만 필터의 prediction 단계를 수행한다.
    //!
    //! @param[in] u 모션 모델의 입력
    //! @param[in] q 모션 모델의 상태 예측에 대한 공분산
    void Predict(float u, float q)
    {
        // 여기서 u는 이동한 거리. 모션모델.
        x_kf_ = x_kf_ + u;
        x_odom_ = x_odom_ + u;
        // p_는 위치의 분산. q는 이동한 거리의 분산.
        p_ = p_ + q;
    }
    //! @brief laser scan으로부터 칼만필터의 update단계를 수행한다.
    //!
    //! @param[in] z 센서 관측. 여기서는 x축 위치를 나타냄
    //! @param[in] r 센서 관측 노이즈.
    void Update(float z, float r)
    {
        // kalman gain
        float K = 0;
        // divide by zero 에러 제거
        if (p_ != 0)
        {
            K = p_ / (p_ + r);
        }

        // state 업데이트
        x_kf_ = x_kf_ + K * (z - x_kf_);

        // variance 업데이트
        p_ = (1 - K) * p_;
    }

    //! @brief odometry callback
    //!
    //! @param[in] msg 구독자가 수신한 odometry 메시지
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (!enable_odom_)
        {
            return;
        }
        // 첫 번째 odometry에서 마지막 측정의 타임스탬프를 저장 후 아무것도 하지 않음.
        // 이동한 거리를 측정하려면 두 개의 시점이 필요하다.
        if (prev_prediction_ == ros::Time(0))
        {
            prev_prediction_ = msg->header.stamp;
            return;
        }
        // delta t를 구한다.
        const float &dt = (msg->header.stamp - prev_prediction_).toSec();
        // x축으로 이동한 거리
        float u = msg->twist.twist.linear.x * dt;
        // 이동 후 x의 공분산 증분. 1mm 의 표준편차가 있을 것으로 가정함.
        float q = 0.0000001;

        Predict(u, q);
        // 타임스탬프 업데이트
        prev_prediction_ = msg->header.stamp;
        enable_odom_ = false;
        enable_scan_ = true;
    }
    //! @brief laser scan callback
    //!
    //! @param[in] msg 구독자가 수신한 laser scan 메시지
    void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        if (!enable_scan_)
        {
            return;
        }
        // 측정할 물체의 roi를 박스로 설정한다.
        const float &roi_x_min = 0.10;  // x축 최소
        const float &roi_x_max = 1.0;   // x축 최대
        const float &roi_y_min = -0.10; // y축 최소
        const float &roi_y_max = 0.10;  // y축 최대

        // 측정한 물체의 laser scan의 포인트들의 x를 통계낸다.
        float x_sum = 0;        // 합
        float x_sq_sum = 0;     // 제곱의 합
        unsigned int count = 0; // 통계낸 x의 개수

        // range 의 minimum. 초기화는 float의 최대임.
        float r_min = std::numeric_limits<float>::max();
        // roi 에서 점을 찾았는지 여부
        bool r_detected = false;

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
            r_detected = true;             // detected flag 세움
            r_min = r < r_min ? r : r_min; // minimum 업데이트
            ang += msg->angle_increment;   // 다음 angle로 업데이트
        }
        // 측정이 이루어지지 않은 것. 건너뛴다.
        if (!r_detected)
        {
            return;
        }
        // 칼만필터 업데이트
        const float &z = 0.95 - r_min; // 좌표변환.
        // 거리에 따라 공분산을 다르게 적용.
        // 센서데이터가 z <0.35m 범위에서 약 0.1m 오차가 존재함.
        // 0.35m 밖에서 실험적으로 측정된 노이즈 공분산은 0.000014 m^2
        const float &z_var = z < 0.35 ? 0.01 : 0.000014;
        Update(z, z_var);
        // 화면에 출력
        ROS_INFO("x_kf: %f, x_odom: %f, x_sensor: %f, x_sensor_var: %f", x_kf_, x_odom_, z, z_var);
        // 메시지 생성 후 칼만 필터 결과를 publish한다.
        turtle_control::kf kf_msg;
        kf_msg.x_kf = x_kf_;
        kf_msg.x_odom = x_odom_;
        kf_msg.x_sensor = z;
        kf_msg.x_sensor_var = z_var;
        pub_kf_.publish(kf_msg);

        enable_odom_ = true;
        enable_scan_ = false;
    }

private:
    float x_kf_;   // 칼만 필터의 현재 위치
    float x_odom_; // odometry로 부터 얻은 위치
    float p_;      // 칼만 필터의 현재 분산

    bool enable_odom_;
    bool enable_scan_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_scan_;
    ros::Time prev_prediction_;
    ros::Publisher pub_kf_;
};

int main(int argc, char *argv[])
{
    // 노드 초기화
    ros::init(argc, argv, "turtle_localization");
    // 노드 핸들을 private namespace로 초기화
    ros::NodeHandle nh("~");
    // 칼만필터 생성
    // KalmanFilter1D kf(0.10847, 0.05);
    KalmanFilter1D kf(0.0, 0.000025);
    // 칼만필터 초기화
    kf.Init(nh);
    // 콜백 작동 시작
    ros::spin();
    return 0;
}
