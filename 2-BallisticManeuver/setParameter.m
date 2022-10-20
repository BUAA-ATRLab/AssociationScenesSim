
%Version��setParameter.m
%Description: �������ý�������Ĳ���
%  ������ͷ����λ���Ŀ��

function result0 = setParameter()

result0.iMisNum=1;                     %[���壺trajectory���][��λ��1][��Դ������]
result0.iTypeChos=1;                   %[���壺�������ַ�ʽ����launch point��landing point��1����γ�ߣ�2���״�������ϵ��][��λ��1][��Դ������]

result0.dLatiLaunch=38.2362;            %[���壺launch pointγ��][��λ����][��Դ������]
result0.dLongLaunch=156.3867;          %[���壺launch point����][��λ����][��Դ������]
result0.dAltiLaunch=0;                  %[���壺launch point�߶�] [��λ��m][��Դ������]
result0.dLatiLand=39.2362;              %[���壺landing pointγ��][��λ����][��Դ������]
result0.dLongLand=115.7546;            %[���壺landing point����][��λ����][��Դ������]
result0.dAltiLand=100;                  %[���壺landing point�߶�][��λ��m][��Դ������]

result0.dLatiRadar=39.2362;			   %[���壺�״�λ�õ�γ��][��λ����][��Դ������]+
result0.dLongRadar=115.7546;              %[���壺�״�λ�õľ���][��λ����][��Դ������]+
result0.dAltiRadar=10;                    %[���壺�״�λ�õĸ߶�][��λ��m][��Դ������]+

result0.dAzLaunch=0;                    %[���壺launch point���״�������ϵ�µķ�λ��][��λ����][��Դ������]
result0.dRnLaunch=0;                    %[���壺launch point���״�������ϵ�µľ���][��λ��m][��Դ������]
result0.dAlLaunch=0;                    %[���壺launch point���״�������ϵ�µ�����][��λ����][��Դ������]
result0.dAzLand=0;                      %[���壺landing point���״�������ϵ�µķ�λ��][��λ����][��Դ������]
result0.dRnLand=0;                      %[���壺landing point���״�������ϵ�µľ���][��λ��m][��Դ������]
result0.dAlLand=0;                      %[���壺landing point���״�������ϵ�µ�����][��λ����][��Դ������]

%�״�ɢ�����(Radar Cross section,��дRCS)
result0.dDDRCS=0.1;
result0.dHeavyDecoyRCS=0.05;
result0.dLightDecoyRCS=0.03;
result0.dDebrisRCS=0.01;
result0.dPiecesRCS=0;

result0.miukInitial=0.15;           %�����ȵ�����ֵ
result0.v0Initial=0.577;            %�������Ʊȵ�����ֵ
result0.Pb0=240;                    %���������
result0.arfa=288/result0.Pb0;       %�������߿�����ϵ��
result0.PM=10000;                   %��ɽ����غ�

result0.delta_t=0.01;               %����ʱ�䲽��

result0.mWarhead=500;               %warhead��������λ��kg
result0.mWarhead2=500;              %����warhead��������λ��kg
result0.mHeavyDecoy=450;            %Heavy Decoy��������λ��kg
result0.S_m=3;                      %�������������,��λ��ƽ����

%����λ�������
result0.ManeuverStartTime = 20;     % ��������ξ������������
result0.ManeuvergY = 10;            % �������ٶȣ�λ����������
result0.ManeuvergZ = 10;            % �������ٶȣ���ֱ�ڵ���ƽ��
result0.ManeuverVzmax = 40;        %��ֱ�ڵ���ƽ���ٶ����ֵ

%�����������
result0.radar_noise_r = 10;                      %�״���������λ��m
result0.radar_noise_theta = 0.2 *pi/180;         %�״�⸩��������λ������
result0.radar_noise_phi = 0.2 *pi/180;           %�״�ⷽλ������λ������

result0.telescope_noise_theta = 20/3600 *pi/180;       %��ѧ��Զ���⸩��������λ������,20����
result0.telescope_noise_phi = 20/3600 *pi/180;         %��ѧ��Զ���ⷽλ������λ������,20����
end

