function result = setParameter()
result.delta_t = 0.1;                        %�����������λ��s
result.g = 9.8;                              %�����������ٶȣ���λ��m/s^2
result.Ttotal = 158;                          %���˶�ʱ��
result.k_C_L = 0.05 ;                        %��������ϵ��

%% //�ɻ�����
result.num_plane = 1;                              %�ɻ���Ŀ
result.m_plane = 8495;                             %�ɻ���������λ��kg
result.S_plane = 27.87;                            %�������,��λ��m^2
result.V_plane = 300 ;                             %�����ٶȣ���λm/s
result.T1_plane = 3;                               %S��������ʱ��
result.max_Ang_b = 10;                       %����ת�ǣ���λ����
result.T2_plane = 60;                              %S����ת��ʱ��
result.T3_plane = 3;                               %S�����ĳ�ʱ��
result.H_plane = 1e4;                              %�ɻ���ʼ�߶ȣ���λ��m
H = result.H_plane;
result.density = (20300-H)/(20300+H);        %�����ܶ�
result.Vz_plane = -20;                             %�ɻ��߶ȱ仯��
result.x0_plane = 38e3;                            %�ɻ���ʼx0
result.y0_plane = 38e3;                            %�ɻ���ʼy0

%% // ������
result.num_boat = 1;                         %����Ŀ
result.x0_boat = 38e3;                       %����ʼx0
result.y0_boat = 38e3;                       %����ʼy0
result.z0_boat = 20;                         %����ʼz0
result.V_boat = 10;                               %��ʼ����20����/ʱ��ԼΪ10m/s
result.T1_boat = 30;                         %����ʱ��
result.T2_boat = 90;                         %ת��ʱ��
result.T3_boat = result.Ttotal-result.T1_boat-result.T2_boat;%ת�������ʱ��
result.wr_boat = 1/4 * 2*pi /90;                %ת����ٶ�

%% //�����������
result.radar_noise_r = 10;                      %�״���������λ��m
result.radar_noise_theta = 0.2 *pi/180;         %�״�⸩��������λ������
result.radar_noise_phi = 0.2 *pi/180;           %�״�ⷽλ������λ������

result.telescope_noise_theta = 20/3600 *pi/180;       %��ѧ��Զ���⸩��������λ������,20����
result.telescope_noise_phi = 20/3600 *pi/180;         %��ѧ��Զ���ⷽλ������λ������,20����
end

