clear; clc;
%=========== Parameters =========== 터틀봇3의 물리 설정
% http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#hardware-specifications
wheel_radius = 0.0325;       % [m] 로봇의 바퀴 반지름
wheel_interval = 0.158;      % 2L [m] 바퀴간 거리
L = wheel_interval / 2.0;    % 바퀴간 거리를 통한 로봇의 회전 반지름 ( = 바퀴간 거리/2)
max_linear_vel = 0.22;       % [m/sec] 로봇의 최대 선속도
max_rotational_vel = 2.84;   % [rad/sec] 로봇의 최대 회전속도

max_wheel_rotational_vel = 6.7692; % [rad/sec] 로봇의 최대 바퀴 회전속도

lim_linear_vel = 0.2; % 안전을 위해 최대 선속도보다 낮게 리미트 설정
lim_rotational_vel = 0.3; % 안전을 위해 최대 회전속도보다 낮게 리미트 설정

%=========== Subscribers & Publisher ===========
pose_sub = rossubscriber('/odom','BufferSize',2); % ROS에서 토픽명 /odom을 Subscribe
pause(2);  % Ready for the first receiving

vel_pub = rospublisher('/cmd_vel', 'turtlebot3_fake/WheelMsg'); % /cmd_vel 토픽으로 turtlebot3_fake/WheelMsg 타입의 msg를 Publish
vel_msg = rosmessage(vel_pub); % Publish 할 msg 객체 생성

%============== PID =========================== % PID Control을 위한 값 설정
% position error에 사용될 PID
Kx_p = 0.4;
Kx_i = 0;
Kx_d = 0;

% theta(목표점에 향하기 위한 로봇의 각도)값에 사용될 PID
Kt_p = 0.3;
Kt_i = 0.07;
Kt_d = 0.015;

% alpha(orientaion error)값에 사용될 PID
Ka_p = 0.3;
Ka_i = 0.05;
Ka_d = 0;

lookAhead = 0.02; % way point에 도착했음을 판단할 threshold
goalRadius = 0.05; % goal point에 도착했음을 판단할 threshold

%PID initial : theta와 alpha값의 integral값과 differential 값 설정
int_th = 0;
int_al = 0;
df_th = 0;
df_al = 0;

%=============== Trajectory =============
% 로봇이 따라갈 path 생성
dubinsSpace = stateSpaceDubins([-5 5; -5 5; -pi pi]); % 맵에서 사용할 boundary 설정, x,y 값은 -5 ~ 5(m), Heading 값은 -pi ~ pi로 설정
dubinsSpace.MinTurningRadius = 0.074; % 경로에서 사용할 회전 반지름 설정

pathobj = navPath(dubinsSpace); % matlab에서 사용하는 navPath 객체 생성

waypoints = [...
    0 0 0;
    1 0 pi/4;
    1 1 pi*3/4;
    0 1 pi*3/4;
    0 2 pi/4;
    1 2 0]; % waypoint 설정, x,y,heading값 설정

append(pathobj, waypoints); % navPath object에 waypoint 추가
path_num = 50; % 총 path에 사용할 local target point 갯수 설정
interpolate(pathobj, path_num); % interpolation을 통한 path 생성
trajectory = pathobj.States; % navPath 객체에서 local point 값만 따로 때서 저장

%==== visualize =================
% matlab figure에 경로 그리기
figure;
grid on;
axis equal;
hold on;
plot(pathobj.States(:,1), pathobj.States(:,2), "-b");
plot(waypoints(:,1), waypoints(:,2), "*r", "MarkerSize", 10)

%===== robot position initialize ==========

robotGoal = trajectory(end,1:2); % 최종 목표점 설정

position = pose_sub.LatestMessage.Pose.Pose.Position; % ros에서 터틀봇의 현재 position 값 받아옴
quat = pose_sub.LatestMessage.Pose.Pose.Orientation; % ros에서 터틀봇의 현재 orientation 값 받아옴 (쿼터니언)
euler_angle = quat2eul([quat.X quat.Y quat.Z quat.W]); % 쿼터니언 값을 오일러 값으로 변환
Heading = euler_angle(3);  % z축으로 회전하므로 z축 값을 Current Heading [radian]으로 설정

index = 2; % local target point index 설정

iP = [trajectory(index,1);trajectory(index,2)]; % 다음 목표 position 설정
target_ori = trajectory(index,3); % 다음 목표 orientation 설정

distance = 0; % 로봇의 position과 다음 목표점간의 거리 initialize

prev_th = Heading; % 이전 theta값 저장
prev_al = target_ori - Heading; % 이전 alpha값 저장

dt = 1/100; % step time 설정

goal_dist = norm([position.X; position.Y] - robotGoal'); % goal과의 거리 계산
step_dist = norm([trajectory(index,1); trajectory(index,2)] - [trajectory(index+1,1); trajectory(index+1,2)]); % local point간 거리 계산

%========== simulation ================
while( goal_dist > goalRadius )
    rate = rosrate(2);
    reset(rate); % ros rate 설정
    
    % 로봇의 현재 position과 orientation 값 받아옴
    position = pose_sub.LatestMessage.Pose.Pose.Position;
    quat = pose_sub.LatestMessage.Pose.Pose.Orientation;
    euler_angle = quat2eul([quat.X quat.Y quat.Z quat.W]);
    Heading = euler_angle(3);  % Current Heading [radian]
    
    % 로봇과 matlab의 좌표계 일치과정
    iQ = [position.X; position.Y];
    iD = iP - iQ;
    ri_R = [
        cos(Heading) sin(Heading);
        -sin(Heading) cos(Heading);
        ];
    
    rP = ri_R * iD;
    
    goal_dist = norm(iQ - robotGoal'); % 목표점까지 거리 계산
    obj_dist = norm(iQ - iP); % 다음 local point까지 거리 계산
    
    if obj_dist < lookAhead % obj_dist 값이 threshold 값 이내일 경우, 다음 포인트를 목표점으로 지정
        index = index + 1;
        if index >= path_num % 다음 목표점이 없을 경우, path를 모두 지난 것이므로 프로그램 종료
            break
        end
        iP = [trajectory(index,1); trajectory(index,2)]; % 목표 position 업데이트
        target_ori = trajectory(index,3); % 목표 orientaion 업데이트
        step_dist = norm([trajectory(index,1); trajectory(index,2)] - [trajectory(index-1,1); trajectory(index-1,2)]); % step 거리 업데이트
    end
    
    % theta값 계산
    theta = atan2(rP(2),rP(1));
    % alpha값 계산 (값을 -pi ~ pi 사이로 계산해야함)
    alpha = target_ori - Heading;
    alpha = mod(alpha,2*pi);
    if alpha > pi
        alpha = alpha - 2*pi;
    end
    
    % PID control을 위한 theta와 alpha의 intergral, differential 값 계산
    int_th = int_th + dt*theta;
    int_al = int_al + dt*alpha;
    df_th = (theta - prev_th)/dt;
    df_al = (alpha - prev_al)/dt;
    
    % 현재 theta와 alpha값을 다음 스텝에 사용하기 위해 prev 값에 저장
    prev_th = theta;
    prev_al = alpha;
    
    % 로봇의 선속도와 회전속도 계산 (PID control)
    % 선속도는 P control만 실행
    linear_velocity  = Kx_p * step_dist;
    % 회전은 theta와 alpha에 대한 PID control 시행
    angular_velocity = Kt_p*theta + Kt_i*int_th + Kt_d*df_th + Ka_p*alpha + Ka_i*int_al + Ka_d*df_al;
    
    % 선속도가 limit를 넘지 않도록 조정
    linear_velocity = min(linear_velocity,lim_linear_vel);
    linear_velocity = max(linear_velocity, -lim_linear_vel);
    % 각 바퀴의 속도 계산
    leftWheel_vel  = linear_velocity - (L * angular_velocity);
    rightWheel_vel = linear_velocity + (L * angular_velocity);

    %=========== Publishing ===========
    % 터틀봇에 계산된 바퀴의 속도 전달
    vel_msg.LeftWheel = leftWheel_vel;
    vel_msg.RightWheel = rightWheel_vel;
    send(vel_pub, vel_msg);
   
    % 지나간 경로 plot
    hold on
    scatter(iQ(1,1), iQ(2,1), 1.5, 'k');
    
    waitfor(rate);
end