
%Version：setParameter.m
%Description: 用于设置界面输入的参数
%  单个弹头再入段机动目标

function result0 = setParameter()

result0.iMisNum=1;                     %[含义：trajectory编号][单位：1][来源：界面]
result0.iTypeChos=1;                   %[含义：采用哪种方式设置launch point和landing point（1：经纬高；2：雷达球坐标系）][单位：1][来源：界面]

result0.dLatiLaunch=38.2362;            %[含义：launch point纬度][单位：°][来源：界面]
result0.dLongLaunch=156.3867;          %[含义：launch point经度][单位：°][来源：界面]
result0.dAltiLaunch=0;                  %[含义：launch point高度] [单位：m][来源：界面]
result0.dLatiLand=39.2362;              %[含义：landing point纬度][单位：°][来源：界面]
result0.dLongLand=115.7546;            %[含义：landing point经度][单位：°][来源：界面]
result0.dAltiLand=100;                  %[含义：landing point高度][单位：m][来源：界面]

result0.dLatiRadar=39.2362;			   %[含义：雷达位置的纬度][单位：°][来源：界面]+
result0.dLongRadar=115.7546;              %[含义：雷达位置的经度][单位：°][来源：界面]+
result0.dAltiRadar=10;                    %[含义：雷达位置的高度][单位：m][来源：界面]+

result0.dAzLaunch=0;                    %[含义：launch point在雷达球坐标系下的方位角][单位：°][来源：界面]
result0.dRnLaunch=0;                    %[含义：launch point在雷达球坐标系下的距离][单位：m][来源：界面]
result0.dAlLaunch=0;                    %[含义：launch point在雷达球坐标系下的仰角][单位：°][来源：界面]
result0.dAzLand=0;                      %[含义：landing point在雷达球坐标系下的方位角][单位：°][来源：界面]
result0.dRnLand=0;                      %[含义：landing point在雷达球坐标系下的距离][单位：m][来源：界面]
result0.dAlLand=0;                      %[含义：landing point在雷达球坐标系下的仰角][单位：°][来源：界面]

%雷达散射截面(Radar Cross section,缩写RCS)
result0.dDDRCS=0.1;
result0.dHeavyDecoyRCS=0.05;
result0.dLightDecoyRCS=0.03;
result0.dDebrisRCS=0.01;
result0.dPiecesRCS=0;

result0.miukInitial=0.15;           %质量比迭代初值
result0.v0Initial=0.577;            %地面重推比迭代初值
result0.Pb0=240;                    %地面比推力
result0.arfa=288/result0.Pb0;       %发动机高空特性系数
result0.PM=10000;                   %起飞截面载荷

result0.delta_t=0.01;               %仿真时间步长

result0.mWarhead=500;               %warhead质量，单位，kg
result0.mWarhead2=500;              %二级warhead质量，单位，kg
result0.mHeavyDecoy=450;            %Heavy Decoy质量，单位：kg
result0.S_m=3;                      %飞行器特性面积,单位：平方米

%再入段机动参数
result0.ManeuverStartTime = 20;     % 进入再入段经过多少秒机动
result0.ManeuvergY = 10;            % 机动加速度，位于升力方向
result0.ManeuvergZ = 10;            % 机动加速度，垂直于弹道平面
result0.ManeuverVzmax = 40;        %垂直于弹道平面速度最大值

%设置量测误差
result0.radar_noise_r = 10;                      %雷达测距离误差，单位：m
result0.radar_noise_theta = 0.2 *pi/180;         %雷达测俯仰角误差，单位：弧度
result0.radar_noise_phi = 0.2 *pi/180;           %雷达测方位角误差，单位：弧度

result0.telescope_noise_theta = 20/3600 *pi/180;       %光学望远镜测俯仰角误差，单位：弧度,20角秒
result0.telescope_noise_phi = 20/3600 *pi/180;         %光学望远镜测方位角误差，单位：弧度,20角秒
end

