%Date：2021.1.31
%Version：setEnvironment.m
%Description: 用于设置仿真过程需要的环境变量

function result0 = setEnvironment()
    result0.a=6378135;                  %椭球长轴半径
    result0.f=1/298.26;                 %地球扁率
    result0.e_2=2*result0.f-power(result0.f,2);         %第一偏心率的平方
    
    result0.g0=9.80665;           %重力加速度
    
    result0.r0=6356766.0;           %标定地球半径
    result0.p0=1.01325e5;           %海平面高度的大气压强
    result0.rou0=1.2250;            %海平面高度的大气密度标准值
    result0.fM=3.986005e14;         %地心引力常数
    
    result0.R_R=6371000;            %地球平均半径
    result0.R_=29.27;               %以重力工程制单位表示的海平面干燥空气之气体常数，单位：kg*m/kg*0C
    result0.T0=288.15;              %海平面绝对温度
    result0.beta=1/(result0.R_*result0.T0);
end