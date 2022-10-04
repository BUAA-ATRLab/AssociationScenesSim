function result = setParameter()
result.delta_t = 0.1;                        %采样间隔，单位：s
result.g = 9.8;                              %当地重力加速度：单位：m/s^2
result.Ttotal = 158;                          %总运动时间
result.k_C_L = 0.05 ;                        %空气动力系数

%% //飞机参数
result.num_plane = 1;                              %飞机数目
result.m_plane = 8495;                             %飞机质量，单位：kg
result.S_plane = 27.87;                            %机翼面积,单位：m^2
result.V_plane = 300 ;                             %飞行速度，单位m/s
result.T1_plane = 3;                               %S机动进入时间
result.max_Ang_b = 10;                       %最大滚转角，单位：度
result.T2_plane = 60;                              %S机动转弯时间
result.T3_plane = 3;                               %S机动改出时间
result.H_plane = 1e4;                              %飞机初始高度，单位：m
H = result.H_plane;
result.density = (20300-H)/(20300+H);        %空气密度
result.Vz_plane = -20;                             %飞机高度变化率
result.x0_plane = 38e3;                            %飞机初始x0
result.y0_plane = 38e3;                            %飞机初始y0

%% // 船参数
result.num_boat = 1;                         %船数目
result.x0_boat = 38e3;                       %船初始x0
result.y0_boat = 38e3;                       %船初始y0
result.z0_boat = 20;                         %船初始z0
result.V_boat = 10;                               %初始船速20海里/时，约为10m/s
result.T1_boat = 30;                         %匀速时间
result.T2_boat = 90;                         %转弯时间
result.T3_boat = result.Ttotal-result.T1_boat-result.T2_boat;%转弯后匀速时间
result.wr_boat = 1/4 * 2*pi /90;                %转弯角速度

%% //设置量测误差
result.radar_noise_r = 10;                      %雷达测距离误差，单位：m
result.radar_noise_theta = 0.2 *pi/180;         %雷达测俯仰角误差，单位：弧度
result.radar_noise_phi = 0.2 *pi/180;           %雷达测方位角误差，单位：弧度

result.telescope_noise_theta = 20/3600 *pi/180;       %光学望远镜测俯仰角误差，单位：弧度,20角秒
result.telescope_noise_phi = 20/3600 *pi/180;         %光学望远镜测方位角误差，单位：弧度,20角秒
end

