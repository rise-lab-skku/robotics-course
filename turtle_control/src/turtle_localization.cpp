#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "turtle_control/kf.h"

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
        // 이동 후 x의 공분산 증분.
        float q = 0.0001;

        Predict(u, q);
        // 타임스탬프 업데이트
        prev_prediction_ = msg->header.stamp;
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
            // 포인트의 x의 통계를 업데이트
            x_sum += x;                  // x의 합
            x_sq_sum += x*x;               // x 제곱의 합
            count += 1;                  // 개수 1 증가
            ang += msg->angle_increment; // 다음 angle로 업데이트
        }
        // 측정이 이루어지지 않은 것. devide by zero 회피
        if (count == 0)
        {
            return;
        }
        // 통계를 마무리 한다.
        const float &x_mean = x_sum / count;                     //x의 평균
        const float &z = 1.0 - x_mean;                           // 물체는 원래 로봇보다 1m 앞에 있었다. 따라서 1m 에서 물체까지의 거리를 빼면 로봇의 현재 위치가 된다.
        const float &z_var = x_sq_sum / count - x_mean * x_mean; // 제곱의 평균 - 평균의 제곱 = 분산. 1-x 의 분산은 x 의 분산과 동일함.
        // 칼만필터 업데이트
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
    }

private:
    float x_kf_;  // 칼만 필터의 현재 위치
    float x_odom_;  // odometry로 부터 얻은 위치
    float p_;  // 칼만 필터의 현재 분산
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
    KalmanFilter1D kf(0.10847, 0.05);
    // 칼만필터 초기화
    kf.Init(nh);
    // 콜백 작동 시작
    ros::spin();
    return 0;
}
