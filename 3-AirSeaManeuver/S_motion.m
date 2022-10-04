function [ x,y,z,Vx,Vy,Vz,Ang_h,Ang_e,Ang_b ] = S_motion(para)
%--------------������ϵ��صĽǶ�------------
% �ػ�����ϵOb��������ת�ǣ�Angel_head Ang_h����ǣ�Angel_elevation Ang_e����, Angle_bank Ang_b��ת��
% �ع�����ϵOi,
% ��������ϵOw, �нǣ�Angle_attack Ang_aӭ�� Angle_sideslip Ang_s�໬�� 

%--------------F16ս��������--------------
% density:�����ܶȣ�����и߶�H�й� 
% S:�ɻ��Ļ��������m^2��
% D:����
% C_D:����ϵ������ӭ��Angle_attack�йأ�����F16����ϵ��-ӭ������ͼ��ӭ��0~20deg��C_D���ƾ��ȱ仯0.15~0.4����֪����б��k-C_DΪ0.0125
% L:����  C_L:����ϵ��
% m:��������������kg��
% V:�����ٶȣ�m/s��
% ����ת�ǣ�max_Ang_b
% ������ٶ�Ϊw,��Ϊ����Ƕȱ仯��delta_Ang_h

%-------------S�������̼���-----------------
% ��ȷ���߶ȣ�H�������Ժ㶨�ٶȣ�V��������ˮƽ��S�������������������в�����ֱ�߶ȵı仯��
% �ɻ���������ֱ����ʼ�ձ�������ƽ�⣬�ɻ�����������������ȣ��ɹ�ʽ�ɼ����ƽ��״̬�·ɻ�����ӭ�Ǵ�С����ΪAng_ass��
% ���������������ٶȷ���ֻ��ˮƽ���ڷ����仯�����и�����=ӭ�ǣ�
% �������������������У��ɻ�����������ͷ���򣩺�����������������ʼ�ձ���ƽ�⣬���Բ������ǣ�
% ˮƽ����ʱ���������������ֱ��������ƽ�⣬S����ʱ���ɷɻ���б������ת���˶�����������
% Ϊ�˱��ַ��и߶Ȳ��䣬��������ֱ����ķ�����Ҫ���ֲ��䣬��ˮƽ����ķ�����Ϊ����������Ҫ����������
% ����Ϊ��Ҫ�����ٶȲ��䣬���Էɻ�Ա��Ҫ�������ʹ��ͷ������ø����ӭ�ǣ��ɻ��ٶ�����ֱ����ķ��ٶȷ��򲻱䣩����������
% �ɻ��ڶ�ʱ������ƽ��״̬���ػ�����ϵ��X����ת������ת�ǣ�������б��
% �Ӷ����������һ��������

%--------------�������---------------------
delta_t = para.delta_t;                        %�����������λ��s
m = para.m_plane;                             %�ɻ���������λ��kg
g = para.g;                              %�����������ٶȣ���λ��m/s^2
H = para.H_plane;                              %Ŀ��߶ȣ���λ��m
S = para.S_plane;                            %�������,��λ��m^2
density = para.density;        %�����ܶ�
V = para.V_plane;                             %�����ٶȣ���λm/s 
k_C_L = para.k_C_L;

%% -----------------S����ǰ������----------------------------- 
% S������һ�׶�
% �ɻ���ƽ��̬�ڶ�ʱ���ڹ�ת������ת��
T1 = para.T1_plane;                                                %����ʱ��3s
t1 = 0:delta_t:T1-delta_t;
max_Ang_b1 = para.max_Ang_b * pi / 180 ;                           %����ת��10��
delta_Ang_b1 = max_Ang_b1 / T1 ;                       %��ת�Ǳ仯��
Ang_b1 = delta_Ang_b1 * t1;                            %��ת��
w1 = g * tan(Ang_b1)/V;                                %������ٶ�
sum1 = 0;
for num1 = 1 : T1/delta_t
    sum1 = sum1 + w1(num1) * delta_t;                  %�����  
    Ang_h1(num1) = sum1;
end                                  
L1 = (m * g) ./ cos(Ang_b1);     
Ang_a1 = L1 / (0.5 * density * k_C_L * V.^2 * S) ;     %ӭ�Ǳ仯����
Ang_e1 = Ang_a1;                                       %������
                  

% S�����ڶ��׶�
% �ɻ���60�������˰�����ڵ�S����������Ǳ仯Ϊ60��,������ٶ�Ϊ��������ת�Ǳ��ֲ��䣬���������ֵ��������
T2 = para.T2_plane/3;                                              %ת��ʱ��25s                              
t2 = T1:delta_t:T1+T2-delta_t;   
w2 = g * tan(max_Ang_b1)/V ;                           %��ת��
Ang_h2 = Ang_h1(T1/delta_t) + w2 * (t2-T1) ;           %�����
Ang_b2 = ones(1,T2/delta_t) * max_Ang_b1 ;             %��ת�Ǳ��ֲ��䣬���������ֵ
Ang_e2 = ones(1,T2/delta_t) * Ang_e1(T1/delta_t);      %�����Ǳ��ֲ���

% S���������׶�
% �ɻ������һ�׶���ͬ��ʱ���ڽ�����ת��תΪ0
T3 = para.T3_plane;                                               %�ĳ�ʱ��3s
t3 = T1+T2:delta_t:T1+T2+T3-delta_t; 
Ang_b3 = max_Ang_b1 - delta_Ang_b1 * (t3-T1-T2);        %��ת�Ǳ�С
w3 = g * tan(Ang_b3)/V;                                 %������ٶ�
L3 = (m * g) ./ cos(Ang_b3);
Ang_a3 = L3 / (0.5 * density * k_C_L * V.^2 * S) ;      %ӭ�Ǳ�С
Ang_e3 = Ang_a3;                                        %�����Ǳ�С
sum3=0;
for num3 = 1 : T3/delta_t
    sum3 = sum3 + w3(num3) * delta_t;                   %�����  
    Ang_h3(num3) = Ang_h2(T2/delta_t) + sum3;
end

%% --------------------S�����������latter----------------------------
% S������һ�׶�
% �ɻ���ƽ��̬�ڶ�ʱ���ڹ�ת������ת��
T4 = para.T1_plane; 
t4 = 0:delta_t:T4-delta_t;       
max_Ang_b4 = -max_Ang_b1 ;                              %����ת��-10��
delta_Ang_b4 = max_Ang_b4 / T4 ;                        %��ת�Ǳ仯��
Ang_b4 = delta_Ang_b4 * t4;                             %��ת��
w4 = g * tan(Ang_b4)/V;                                 %������ٶ�
sum4 = 0 ;
for num4 = 1 : T4/delta_t
    sum4 = sum4 + w4(num4) * delta_t;                   %�����  
    Ang_h4(num4) = Ang_h3(T3/delta_t) + sum4;
end                                  
L4 = (m * g) ./ cos(Ang_b4);
Ang_a4 = L4 / (0.5 * density * k_C_L * V.^2 * S) ;      %ӭ�Ǳ仯����
Ang_e4 = Ang_a4;                                        %������
                  

% S�����ڶ��׶�
% �ɻ���60�������˰�����ڵ�S����������Ǳ仯Ϊ60��,������ٶ�Ϊ��������ת�Ǳ��ֲ��䣬���������ֵ�������Ǳ��ֲ���
T5 = para.T2_plane;
t5 = T4:delta_t:T4+T5-delta_t;    
w5 = g * tan(max_Ang_b4)/V ;
Ang_h5 = Ang_h4(T4/delta_t) + w5 * (t5-T4) ;             %�����
Ang_b5 = ones(1,T5/delta_t) * max_Ang_b4 ;               %��ת��
Ang_e5 = ones(1,T5/delta_t) * Ang_e1(T4/delta_t);        %������

% S���������׶�
% �ɻ������һ�׶���ͬ��ʱ���ڽ�����ת��תΪ0
T6 = para.T3_plane;
t6 = T4+T5:delta_t:T4+T5+T6-delta_t; 
Ang_b6 = max_Ang_b4 - delta_Ang_b4 * (t6-T4-T5);        %��ת�Ǳ�С
w6 = g * tan(Ang_b6)/V;                                 %������ٶ�
L6 = (m * g) ./ cos(Ang_b6);
Ang_a6 = L6 / (0.5 * density * k_C_L * V.^2 * S) ;      %ӭ�Ǳ�С��
Ang_e6 = Ang_a6;                                        %�����Ǳ�С
sum6 = 0;
for num6 = 1 : T4/delta_t
    sum6 = sum6 + w6(num6) * delta_t;                   %�����  
    Ang_h6(num6) = Ang_h5(T5/delta_t)+sum6;
end

% �ɻ����������ת�ǵ����
T7 = para.T1_plane; 
t7 = 0:delta_t:T7-delta_t;                              %����ת��10��
max_Ang_b7 = max_Ang_b1 ;
delta_Ang_b7 = max_Ang_b7 / T7 ;                        %��ת�Ǳ仯��
Ang_b7 = delta_Ang_b7 * t7;                             %��ת��
w7 = g * tan(Ang_b7)/V;                                 %������ٶ�
sum7 = 0 ;
for num7 = 1 : T7/delta_t
    sum7 = sum7 + w7(num7) * delta_t;                   %�����  
    Ang_h7(num7) = Ang_h6(T6/delta_t) + sum7;
end                                  
L7 = (m * g) ./ cos(Ang_b7);      
Ang_a7 = L7 / (0.5 * density * k_C_L * V.^2 * S) ;      %ӭ�Ǳ仯����
Ang_e7 = Ang_a7;                                        %������
                  

% S�����ڶ��׶�
% �ɻ���60�������˰�����ڵ�S����������Ǳ仯Ϊ60��,������ٶ�Ϊ��������ת�Ǳ��ֲ��䣬���������ֵ�������Ǳ��ֲ��䣿����
T8 = para.T2_plane;
t8 = T7:delta_t:T7+T8-delta_t;   
w8 = g * tan(max_Ang_b7)/V ;
Ang_h8 = Ang_h7(T7/delta_t) + w8 * (t8-T7) ;            %�����
Ang_b8 = ones(1,T8/delta_t) * max_Ang_b7 ;              %��ת��
Ang_e8 = ones(1,T8/delta_t) * Ang_e7(T7/delta_t);       %������

% S���������׶�
% �ɻ������һ�׶���ͬ��ʱ���ڽ�����ת��תΪ0
T9 = para.T3_plane;
t9 = T7+T8:delta_t:T7+T8+T9-delta_t; 
Ang_b9 = max_Ang_b7 - delta_Ang_b7 * (t9-T7-T8);        %��ת�Ǳ�С
w9 = g * tan(Ang_b9)/V;                                 %������ٶ�
L9 = (m * g) ./ cos(Ang_b9);
Ang_a9 = L9 / (0.5 * density * k_C_L * V.^2 * S) ;      %ӭ�Ǳ�С
Ang_e9 = Ang_a9;                                        %�����Ǳ�С
sum9=0;
for num9 = 1 : T9/delta_t
    sum9 = sum9 + w9(num9) * delta_t;                   %�����  
    Ang_h9(num9) = Ang_h8(T8/delta_t) + sum9;
end


%% λ�á��ٶȡ���̬�ǵı仯����
% ���ϾŶ�����
Ang_b = [Ang_b1 Ang_b2 Ang_b3 Ang_b4 Ang_b5 Ang_b6 Ang_b7 Ang_b8 Ang_b9] ;     %��ת��
Ang_e = [Ang_e1 Ang_e2 Ang_e3 Ang_e4 Ang_e5 Ang_e6 Ang_e7 Ang_e8 Ang_e9] ;     %������
Ang_e = Ang_e*pi/180;
Ang_h = [Ang_h1 Ang_h2 Ang_h3 Ang_h4 Ang_h5 Ang_h6 Ang_h7 Ang_h8 Ang_h9] ; %�����

% Ang_b_rad2deg = Ang_b * 180 / pi;
% Ang_e_rad2deg = Ang_e * 180 / pi;
% Ang_h_rad2deg = Ang_h * 180 / pi;
% T_tol = T1+T2+T3+T4+T5+T6+T7+T8+T9;
% t = 0 : delta_t : T_tol-delta_t;
% figure;
% subplot(1,3,1);
% plot(t,Ang_b_rad2deg,'LineWidth',2);
% title('��ת�Ǳ仯����ͼ');
% xlabel('ʱ��(��)');
% ylabel('��ת��(��)');
% axis([0 200 -15 15]);
% grid on
% subplot(1,3,2);
% plot(t,Ang_e_rad2deg,'LineWidth',2);
% title('�����Ǳ仯����ͼ');
% xlabel('ʱ��(��)');
% ylabel('������(��)');
% grid on
% subplot(1,3,3);
% plot(t,Ang_h_rad2deg,'LineWidth',2);
% title('����Ǳ仯����ͼ');
% xlabel('ʱ��(��)');
% ylabel('������(��)');
% grid on

total = (T1+T2+T3+T4+T5+T6+T7+T8+T9)/delta_t;
Vx = V * cos(Ang_h);
Vy = V * sin(Ang_h);
Vz = para.Vz_plane;
sumx = 0;
sumy = 0;
sumz = 0;

x0 = para.x0_plane;              % ��ʼλ��x=38km��y=38km,z=H    
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
% title('�ɻ���x-yƽ�溽��');
% xlabel('x��(��)');
% ylabel('y��(��)');
% 
% figure;
% plot3(x,y,z,'LineWidth',3);
% title('�ɻ���x-y-zƽ�溽��');
% view(124,24);
% xlabel('X(North)');
% ylabel('Y(East)');
% zlabel('Z(Down)');
% grid on;

end

