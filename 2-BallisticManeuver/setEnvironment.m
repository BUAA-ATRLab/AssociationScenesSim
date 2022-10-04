%Date��2021.1.31
%Version��setEnvironment.m
%Description: �������÷��������Ҫ�Ļ�������

function result0 = setEnvironment()
    result0.a=6378135;                  %������뾶
    result0.f=1/298.26;                 %�������
    result0.e_2=2*result0.f-power(result0.f,2);         %��һƫ���ʵ�ƽ��
    
    result0.g0=9.80665;           %�������ٶ�
    
    result0.r0=6356766.0;           %�궨����뾶
    result0.p0=1.01325e5;           %��ƽ��߶ȵĴ���ѹǿ
    result0.rou0=1.2250;            %��ƽ��߶ȵĴ����ܶȱ�׼ֵ
    result0.fM=3.986005e14;         %������������
    
    result0.R_R=6371000;            %����ƽ���뾶
    result0.R_=29.27;               %�����������Ƶ�λ��ʾ�ĺ�ƽ��������֮���峣������λ��kg*m/kg*0C
    result0.T0=288.15;              %��ƽ������¶�
    result0.beta=1/(result0.R_*result0.T0);
end