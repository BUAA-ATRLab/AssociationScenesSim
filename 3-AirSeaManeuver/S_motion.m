function [ x,y,z,Vx,Vy,Vz,Ang_h,Ang_e,Ang_b ] = S_motion(para)
%--------------与坐标系相关的角度------------
% 载机坐标系Ob，绕轴旋转角：Angel_head Ang_h航向角，Angel_elevation Ang_e仰角, Angle_bank Ang_b滚转角
% 地固坐标系Oi,
% 航迹坐标系Ow, 夹角：Angle_attack Ang_a迎角 Angle_sideslip Ang_s侧滑角 

%--------------F16战斗机参数--------------
% density:大气密度，与飞行高度H有关 
% S:飞机的机翼面积（m^2）
% D:阻力
% C_D:阻力系数，与迎角Angle_attack有关，根据F16阻力系数-迎角曲线图，迎角0~20deg，C_D近似均匀变化0.15~0.4，可知近似斜率k-C_D为0.0125
% L:升力  C_L:升力系数
% m:空载质量（空重kg）
% V:飞行速度（m/s）
% 最大滚转角：max_Ang_b
% 航向角速度为w,即为航向角度变化率delta_Ang_h

%-------------S机动过程简述-----------------
% 在确定高度（H）处，以恒定速度（V）做匀速水平面S机动，由于整个过程中并无竖直高度的变化，
% 飞机保持在竖直方向始终保持受力平衡，飞机所需升力与重力相等，由公式可计算出平衡状态下飞机所需迎角大小，记为Ang_ass，
% 由于整个过程中速度方向只在水平面内发生变化，则有俯仰角=迎角，
% 受力分析：整个过程中，飞机的推力（机头方向）和阻力（推力反方向）始终保持平衡，可以不作考虑；
% 水平飞行时，升力（与机翼方向垂直）与重力平衡，S机动时，由飞机倾斜带来做转弯运动的向心力，
% 为了保持飞行高度不变，升力在竖直方向的分力需要保持不变，在水平方向的分力即为向心力，故要求升力增大，
% 又因为需要保持速度不变，所以飞机员需要拉起舵面使机头上仰获得更大的迎角（飞机速度在竖直方向的分速度方向不变），即俯仰角
% 飞机在短时间内由平衡状态绕载机坐标系的X轴旋转到最大滚转角，机身倾斜，
% 从而获得升力在一个向心力

%--------------仿真参数---------------------
delta_t = para.delta_t;                        %采样间隔，单位：s
m = para.m_plane;                             %飞机质量，单位：kg
g = para.g;                              %当地重力加速度：单位：m/s^2
H = para.H_plane;                              %目标高度，单位：m
S = para.S_plane;                            %机翼面积,单位：m^2
density = para.density;        %空气密度
V = para.V_plane;                             %飞行速度，单位m/s 
k_C_L = para.k_C_L;

%% -----------------S机动前半周期----------------------------- 
% S机动第一阶段
% 飞机由平衡态在短时间内滚转到最大滚转角
T1 = para.T1_plane;                                                %进入时间3s
t1 = 0:delta_t:T1-delta_t;
max_Ang_b1 = para.max_Ang_b * pi / 180 ;                           %最大滚转角10度
delta_Ang_b1 = max_Ang_b1 / T1 ;                       %滚转角变化率
Ang_b1 = delta_Ang_b1 * t1;                            %滚转角
w1 = g * tan(Ang_b1)/V;                                %航向角速度
sum1 = 0;
for num1 = 1 : T1/delta_t
    sum1 = sum1 + w1(num1) * delta_t;                  %航向角  
    Ang_h1(num1) = sum1;
end                                  
L1 = (m * g) ./ cos(Ang_b1);     
Ang_a1 = L1 / (0.5 * density * k_C_L * V.^2 * S) ;     %迎角变化幅度
Ang_e1 = Ang_a1;                                       %俯仰角
                  

% S机动第二阶段
% 飞机在60秒内做了半个周期的S机动，航向角变化为60°,航向角速度为恒量，滚转角保持不变，即保持最大值，俯仰角
T2 = para.T2_plane/3;                                              %转弯时间25s                              
t2 = T1:delta_t:T1+T2-delta_t;   
w2 = g * tan(max_Ang_b1)/V ;                           %滚转角
Ang_h2 = Ang_h1(T1/delta_t) + w2 * (t2-T1) ;           %航向角
Ang_b2 = ones(1,T2/delta_t) * max_Ang_b1 ;             %滚转角保持不变，即保持最大值
Ang_e2 = ones(1,T2/delta_t) * Ang_e1(T1/delta_t);      %俯仰角保持不变

% S机动第三阶段
% 飞机在与第一阶段相同的时间内将最大滚转角转为0
T3 = para.T3_plane;                                               %改出时间3s
t3 = T1+T2:delta_t:T1+T2+T3-delta_t; 
Ang_b3 = max_Ang_b1 - delta_Ang_b1 * (t3-T1-T2);        %滚转角变小
w3 = g * tan(Ang_b3)/V;                                 %航向角速度
L3 = (m * g) ./ cos(Ang_b3);
Ang_a3 = L3 / (0.5 * density * k_C_L * V.^2 * S) ;      %迎角变小
Ang_e3 = Ang_a3;                                        %俯仰角变小
sum3=0;
for num3 = 1 : T3/delta_t
    sum3 = sum3 + w3(num3) * delta_t;                   %航向角  
    Ang_h3(num3) = Ang_h2(T2/delta_t) + sum3;
end

%% --------------------S机动后半周期latter----------------------------
% S机动第一阶段
% 飞机由平衡态在短时间内滚转到最大滚转角
T4 = para.T1_plane; 
t4 = 0:delta_t:T4-delta_t;       
max_Ang_b4 = -max_Ang_b1 ;                              %最大滚转角-10度
delta_Ang_b4 = max_Ang_b4 / T4 ;                        %滚转角变化率
Ang_b4 = delta_Ang_b4 * t4;                             %滚转角
w4 = g * tan(Ang_b4)/V;                                 %航向角速度
sum4 = 0 ;
for num4 = 1 : T4/delta_t
    sum4 = sum4 + w4(num4) * delta_t;                   %航向角  
    Ang_h4(num4) = Ang_h3(T3/delta_t) + sum4;
end                                  
L4 = (m * g) ./ cos(Ang_b4);
Ang_a4 = L4 / (0.5 * density * k_C_L * V.^2 * S) ;      %迎角变化幅度
Ang_e4 = Ang_a4;                                        %俯仰角
                  

% S机动第二阶段
% 飞机在60秒内做了半个周期的S机动，航向角变化为60°,航向角速度为恒量，滚转角保持不变，即保持最大值，俯仰角保持不变
T5 = para.T2_plane;
t5 = T4:delta_t:T4+T5-delta_t;    
w5 = g * tan(max_Ang_b4)/V ;
Ang_h5 = Ang_h4(T4/delta_t) + w5 * (t5-T4) ;             %航向角
Ang_b5 = ones(1,T5/delta_t) * max_Ang_b4 ;               %滚转角
Ang_e5 = ones(1,T5/delta_t) * Ang_e1(T4/delta_t);        %俯仰角

% S机动第三阶段
% 飞机在与第一阶段相同的时间内将最大滚转角转为0
T6 = para.T3_plane;
t6 = T4+T5:delta_t:T4+T5+T6-delta_t; 
Ang_b6 = max_Ang_b4 - delta_Ang_b4 * (t6-T4-T5);        %滚转角变小
w6 = g * tan(Ang_b6)/V;                                 %航向角速度
L6 = (m * g) ./ cos(Ang_b6);
Ang_a6 = L6 / (0.5 * density * k_C_L * V.^2 * S) ;      %迎角变小，
Ang_e6 = Ang_a6;                                        %俯仰角变小
sum6 = 0;
for num6 = 1 : T4/delta_t
    sum6 = sum6 + w6(num6) * delta_t;                   %航向角  
    Ang_h6(num6) = Ang_h5(T5/delta_t)+sum6;
end

% 飞机正向增大滚转角到最大
T7 = para.T1_plane; 
t7 = 0:delta_t:T7-delta_t;                              %最大滚转角10度
max_Ang_b7 = max_Ang_b1 ;
delta_Ang_b7 = max_Ang_b7 / T7 ;                        %滚转角变化率
Ang_b7 = delta_Ang_b7 * t7;                             %滚转角
w7 = g * tan(Ang_b7)/V;                                 %航向角速度
sum7 = 0 ;
for num7 = 1 : T7/delta_t
    sum7 = sum7 + w7(num7) * delta_t;                   %航向角  
    Ang_h7(num7) = Ang_h6(T6/delta_t) + sum7;
end                                  
L7 = (m * g) ./ cos(Ang_b7);      
Ang_a7 = L7 / (0.5 * density * k_C_L * V.^2 * S) ;      %迎角变化幅度
Ang_e7 = Ang_a7;                                        %俯仰角
                  

% S机动第二阶段
% 飞机在60秒内做了半个周期的S机动，航向角变化为60°,航向角速度为恒量，滚转角保持不变，即保持最大值，俯仰角保持不变？？？
T8 = para.T2_plane;
t8 = T7:delta_t:T7+T8-delta_t;   
w8 = g * tan(max_Ang_b7)/V ;
Ang_h8 = Ang_h7(T7/delta_t) + w8 * (t8-T7) ;            %航向角
Ang_b8 = ones(1,T8/delta_t) * max_Ang_b7 ;              %滚转角
Ang_e8 = ones(1,T8/delta_t) * Ang_e7(T7/delta_t);       %俯仰角

% S机动第三阶段
% 飞机在与第一阶段相同的时间内将最大滚转角转为0
T9 = para.T3_plane;
t9 = T7+T8:delta_t:T7+T8+T9-delta_t; 
Ang_b9 = max_Ang_b7 - delta_Ang_b7 * (t9-T7-T8);        %滚转角变小
w9 = g * tan(Ang_b9)/V;                                 %航向角速度
L9 = (m * g) ./ cos(Ang_b9);
Ang_a9 = L9 / (0.5 * density * k_C_L * V.^2 * S) ;      %迎角变小
Ang_e9 = Ang_a9;                                        %俯仰角变小
sum9=0;
for num9 = 1 : T9/delta_t
    sum9 = sum9 + w9(num9) * delta_t;                   %航向角  
    Ang_h9(num9) = Ang_h8(T8/delta_t) + sum9;
end


%% 位置、速度、姿态角的变化曲线
% 整合九段曲线
Ang_b = [Ang_b1 Ang_b2 Ang_b3 Ang_b4 Ang_b5 Ang_b6 Ang_b7 Ang_b8 Ang_b9] ;     %滚转角
Ang_e = [Ang_e1 Ang_e2 Ang_e3 Ang_e4 Ang_e5 Ang_e6 Ang_e7 Ang_e8 Ang_e9] ;     %俯仰角
Ang_e = Ang_e*pi/180;
Ang_h = [Ang_h1 Ang_h2 Ang_h3 Ang_h4 Ang_h5 Ang_h6 Ang_h7 Ang_h8 Ang_h9] ; %航向角

% Ang_b_rad2deg = Ang_b * 180 / pi;
% Ang_e_rad2deg = Ang_e * 180 / pi;
% Ang_h_rad2deg = Ang_h * 180 / pi;
% T_tol = T1+T2+T3+T4+T5+T6+T7+T8+T9;
% t = 0 : delta_t : T_tol-delta_t;
% figure;
% subplot(1,3,1);
% plot(t,Ang_b_rad2deg,'LineWidth',2);
% title('滚转角变化曲线图');
% xlabel('时间(秒)');
% ylabel('滚转角(度)');
% axis([0 200 -15 15]);
% grid on
% subplot(1,3,2);
% plot(t,Ang_e_rad2deg,'LineWidth',2);
% title('俯仰角变化曲线图');
% xlabel('时间(秒)');
% ylabel('俯仰角(度)');
% grid on
% subplot(1,3,3);
% plot(t,Ang_h_rad2deg,'LineWidth',2);
% title('航向角变化曲线图');
% xlabel('时间(秒)');
% ylabel('俯仰角(度)');
% grid on

total = (T1+T2+T3+T4+T5+T6+T7+T8+T9)/delta_t;
Vx = V * cos(Ang_h);
Vy = V * sin(Ang_h);
Vz = para.Vz_plane;
sumx = 0;
sumy = 0;
sumz = 0;

x0 = para.x0_plane;              % 初始位置x=38km，y=38km,z=H    
y0 = para.y0_plane;
z0 = H;
x = zeros(total,1);
y = zeros(total,1);
z = zeros(total,1);
for nump = 1 : (T1+T2+T3+T4+T5+T6+T7+T8+T9)/delta_t
    sumx = sumx + Vx(nump) * delta_t ;
    sumy = sumy + Vy(nump) * delta_t ;
    sumz = sumz + Vz * delta_t;
    x(nump) = x0 + sumx;
    y(nump) = y0 + sumy;
    z(nump) = z0 + sumz;
end
Vx = Vx';
Vy = Vy';
Vz = Vz';

% figure;
% plot(x,y);
% title('飞机在x-y平面航迹');
% xlabel('x轴(米)');
% ylabel('y轴(米)');
% 
% figure;
% plot3(x,y,z,'LineWidth',3);
% title('飞机在x-y-z平面航迹');
% view(124,24);
% xlabel('X(North)');
% ylabel('Y(East)');
% zlabel('Z(Down)');
% grid on;

end

