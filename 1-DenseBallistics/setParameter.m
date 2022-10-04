%Version��setParameter.m
%Description: �������ý�������Ĳ���

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
		
	result0.iNumPieces3 = 2;                %[���壺��������Ƭ��Ŀ][��λ��1][��Դ������]
		
	result0.iSumGroup=1;                     %����warhead������������
% 	result0.numSecondLaunchTime=501;            %�ڶ���warhead����ʱ�̣���Free section�ĵ�����ʾ����0��ʾû�еڶ���warhead��
%     result0.numThirdLaunchTime=601;
    result0.numSecondLaunchTime=0;
    result0.numThirdLaunchTime=0;            %������warhead����ʱ�̣���Free section�ĵ�����ʾ����0��ʾû�е�����warhead��

    
    result0.iSumHeavyDecoyFirst  = 2;% ��һ������Heavy Decoy�ĸ���
    result0.iSumHeavyDecoySecond = 0;% �ڶ�������Heavy Decoy�ĸ���
    result0.iSumHeavyDecoyThird  = 0;% ����������Heavy Decoy�ĸ���
        
    result0.iSumLightDecoyFirst  = 2;% ����Light Decoy�ĸ�������Heavy Decoy�������������ġ�
    result0.iSumLightDecoySecond = 0;
    result0.iSumLightDecoyThird  = 0;
        
    result0.iSumDebrisFirst      = 2;  % ����debris������ÿ���Ų��������debris
    result0.iSumDebrisSecond     = 0;
    result0.iSumDebrisThird      = 0;
        
    result0.iSumHeavyDecoy=[result0.iSumHeavyDecoyFirst,result0.iSumHeavyDecoySecond,result0.iSumHeavyDecoyThird];%����Heavy Decoy�ĸ���
    result0.iSumLightDecoy=[result0.iSumLightDecoyFirst ,result0.iSumLightDecoySecond,result0.iSumLightDecoyThird];%����Light Decoy�ĸ���
	result0.iSumDebris=[result0.iSumDebrisFirst,result0.iSumDebrisSecond,result0.iSumDebrisThird];        %����debris����
    
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

    result0.releaseTime = 5;              %Decoy�ͷ�ʱ�̣�Free section��ʼ��Ķ����룩��λ��s
    
    result0.VxOffsetHeavyDecoyFirst=50;
    result0.VzOffsetHeavyDecoyFirst=50;
    result0.VxOffsetHeavyDecoySecond=-50;
    result0.VzOffsetHeavyDecoySecond=-50;
    result0.VxOffsetHeavyDecoyThird=50;
    result0.VzOffsetHeavyDecoyThird=-50;
    result0.VxOffsetHeavyDecoyFourth=-50;
    result0.VzOffsetHeavyDecoyFourth=50;
    result0.VxOffsetLightDecoyFirst=100;
    result0.VzOffsetLightDecoyFirst=0;
    result0.VxOffsetLightDecoySecond=-100;
    result0.VzOffsetLightDecoySecond=0;
    result0.VxOffsetLightDecoyThird=0;
    result0.VzOffsetLightDecoyThird=-100;
    result0.VxOffsetLightDecoyFourth=0;
    result0.VzOffsetLightDecoyFourth=100;
    
    result0.VOffsetDebrisLower=-100;%Debrisƫ���ٶ��½�
    result0.VOffsetDebrisUpper=100;%Debrisƫ���ٶ��Ͻ�
    result0.VOffsetPiecesLower=-100;%Piecesƫ���ٶ��½�
    result0.VOffsetPiecesUpper=100;%Piecesƫ���ٶ��Ͻ�
    
    result0.XOffsetSecondWarheadB=0.0;
    result0.YOffsetSecondWarheadB=2.0e4;
    result0.ZOffsetSecondWarheadB=0.0;
    result0.XOffsetThirdWarheadB=0.0;
    result0.YOffsetThirdWarheadB=1.0e4;
    result0.ZOffsetThirdWarheadB=0.0;
    
    %�����������
    result0.radar_noise_r = 10;                      %�״���������λ��m
    result0.radar_noise_theta = 0.2 *pi/180;         %�״�⸩��������λ������
    result0.radar_noise_phi = 0.2 *pi/180;           %�״�ⷽλ������λ������

    result0.telescope_noise_theta = 20/3600 *pi/180;       %��ѧ��Զ���⸩��������λ������,20����
    result0.telescope_noise_phi = 20/3600 *pi/180;         %��ѧ��Զ���ⷽλ������λ������,20����
    
end
    
    