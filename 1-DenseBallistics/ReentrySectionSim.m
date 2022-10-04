%Date��2021.3.1
%Version��ReentrySectionSim.m
%Description: ���ɵ�ͷ���ն�����Ƭ����������ݣ����ϳɺ�����ߴ����ظ�������
%���84����ϵ��X,Y,Z�Լ�VX��VY��VZ��thetaX,thetaY,thetaZ
function [XReentryS,YReentryS,ZReentryS,VXReentryS,VYReentryS,VZReentryS,thetaXReentryS,thetaYReentryS,thetaZReentryS,num3]=...
    ReentrySectionSim(r_e,V_e,theta_e,h_e,XYZ_e,num_point3,mTarget,fFreeStart,fFreeEnd,launchBLA,launchLandLatiLong,settings,environment)

    g0=environment.g0; %�������ٶ�
    R_R=environment.R_R;%����ƽ���뾶 
    delta_t=settings.delta_t;
    beta=environment.beta;
	S_m=settings.S_m; % �������������,��λ��ƽ����
    rou0=environment.rou0; % �ܶ�

    h3_t=zeros(1,num_point3);
	V3_t=zeros(1,num_point3);
	theta3_t=zeros(1,num_point3);
    f3=zeros(1,num_point3);
        
    h3_t(1)=h_e;
	[~,sonic,~] = calculateRSP(h3_t(1),environment);
	Ma_2=V_e/sonic;
	Cx_ma2=0.09+0.05/Ma_2;%����ϵ��
    
	L3_t=0.0;	%Reentry section�����(�������)
	V3_t(1)=V_e;
	theta3_t(1)=theta_e;
    f3(1)=L3_t/R_R;

    % Reentry section�Σ���һʱ�̷�������ϵλ��
	XReentryG=zeros(1,num_point3);
	YReentryG=zeros(1,num_point3);
    XReentryG(1)= XYZ_e(1);
	YReentryG(1)= XYZ_e(2);
    ZReentryG=XYZ_e(3);

    %λ��������ת����84����ϵ
	XReentryS=zeros(1,num_point3-1);%[���������ϳ�]
	YReentryS=zeros(1,num_point3-1);%[���������ϳ�]
	ZReentryS=zeros(1,num_point3-1);%[���������ϳ�]
    
    hBefore = h3_t(1);%����������µĸ߶�,qzt
    num3=num_point3;%����num_point3
    for i = 2:num_point3  % Reentry section �κ���qzt
        kk11=-Cx_ma2*S_m*rou0*exp(-beta*h3_t(i-1))*(V3_t(i-1)^2)/2/mTarget-g0*sin(theta3_t(i-1));
		kk12=(V3_t(i-1)/(h3_t(i-1)+R_R)-g0/V3_t(i-1))*cos(theta3_t(i-1));
		kk13=V3_t(i-1)*sin(theta3_t(i-1));
      
		kk21=-Cx_ma2*S_m*rou0*exp(-beta*(h3_t(i-1)+0.5*delta_t*kk13))*((V3_t(i-1)+0.5*delta_t*kk11)^2)/2/mTarget-g0*sin((theta3_t(i-1)+0.5*delta_t*kk12));
		kk22=((V3_t(i-1)+0.5*delta_t*kk11)/((h3_t(i-1)+0.5*delta_t*kk13)+R_R)-g0/(V3_t(i-1)+0.5*delta_t*kk11))*cos((theta3_t(i-1)+0.5*delta_t*kk12));
		kk23=(V3_t(i-1)+0.5*delta_t*kk11)*sin((theta3_t(i-1)+0.5*delta_t*kk12));

		kk31=-Cx_ma2*S_m*rou0*exp(-beta*(h3_t(i-1)+0.5*delta_t*kk23))*((V3_t(i-1)+0.5*delta_t*kk21)^2)/2/mTarget-g0*sin((theta3_t(i-1)+0.5*delta_t*kk22));
		kk32=((V3_t(i-1)+0.5*delta_t*kk21)/((h3_t(i-1)+0.5*delta_t*kk23)+R_R)-g0/(V3_t(i-1)+0.5*delta_t*kk21))*cos((theta3_t(i-1)+0.5*delta_t*kk22));
		kk33=(V3_t(i-1)+0.5*delta_t*kk21)*sin((theta3_t(i-1)+0.5*delta_t*kk22));
        
		kk41=-Cx_ma2*S_m*rou0*exp(-beta*(h3_t(i-1)+delta_t*kk33))*((V3_t(i-1)+delta_t*kk31)^2)/2/mTarget-g0*sin((theta3_t(i-1)+delta_t*kk32));
		kk42=((V3_t(i-1)+delta_t*kk31)/((h3_t(i-1)+delta_t*kk33)+R_R)-g0/(V3_t(i-1)+delta_t*kk31))*cos((theta3_t(i-1)+delta_t*kk32));
        kk43=(V3_t(i-1)+delta_t*kk31)*sin((theta3_t(i-1)+delta_t*kk32));
            
		V3_t(i)=V3_t(i-1)+delta_t/6*(kk11+2*kk21+2*kk31+kk41);	%V3_t(i)
		theta3_t(i)=theta3_t(i-1)+delta_t/6*(kk12+2*kk22+2*kk32+kk42);	%theta3_t(i)
		h3_t(i)=h3_t(i-1)+delta_t/6*(kk13+2*kk23+2*kk33+kk43);	%h3_t(i)
			
		L3_t = L3_t+R_R*V3_t(i-1)*cos(theta3_t(i-1))/(R_R+h3_t(i-1))*delta_t;
		f3(i) = L3_t/R_R;
        
        [~,sonic,~] = calculateRSP(h3_t(i),environment);%����߶ȣ����Ӧ����qzt
		Ma_2=V3_t(i)/sonic;
		if Ma_2<0.8 && Ma_2>=0 
			Cx_ma2=0.029;
        elseif Ma_2<=1.068
			Cx_ma2=0.1*Ma_2-0.051;
        else
            Cx_ma2=0.09+0.05/Ma_2;
        end
			
		XReentryG(i)=XReentryG(i-1)+V3_t(i-1)*cos(theta3_t(i-1)-(f3(i-1)+fFreeEnd-fFreeStart))*delta_t;
		YReentryG(i)=YReentryG(i-1)+V3_t(i-1)*sin(theta3_t(i-1)-(f3(i-1)+fFreeEnd-fFreeStart))*delta_t;

		coordS=G2S(launchBLA,launchLandLatiLong,[XReentryG(i),YReentryG(i),ZReentryG],environment);%����ת������launch����ϵ������ֱ������ϵ,qzt
		rTmp=sqrt(power(coordS(1),2)+power(coordS(2),2)+power(coordS(3),2));
		XReentryS(i-1)=(R_R+h3_t(i))/rTmp*coordS(1);
		YReentryS(i-1)=(R_R+h3_t(i))/rTmp*coordS(2);
		ZReentryS(i-1)=(R_R+h3_t(i))/rTmp*coordS(3);
			
		coordBLH = WGS842BLH([XReentryS(i-1),YReentryS(i-1),ZReentryS(i-1)],environment);
        h3_temp = coordBLH(3);
            
        if hBefore>=0 && h3_temp<=0
            num3 = i-1;
            break;        
        else            
            hBefore=h3_temp;
        end
    end
    
    XReentryG=[];
    YReentryG=[];
    
    %Reentry Section ��������ϵ�ٶ�
    VXReentryG=zeros(1,num_point3);
	VYReentryG=zeros(1,num_point3);

    for i = 1:num_point3
        VXReentryG(i)=V3_t(i)*cos(theta3_t(i)-f3(i)-fFreeEnd+fFreeStart);
        VYReentryG(i)=V3_t(i)*sin(theta3_t(i)-f3(i)-fFreeEnd+fFreeStart);
    end
	
    h3_t = [];
    V3_t = [];
        
	%�ٶ�ת��Ϊ84����
	VXReentryS=zeros(1,num_point3);%[���������]  ���ٶ�x
	VYReentryS=zeros(1,num_point3);%[���������]  ���ٶ�y
	VZReentryS=zeros(1,num_point3);%[���������]  ���ٶ�z

    for i = 1:num_point3
        if i > num3
            VXReentryS(i)=0;
            VYReentryS(i)=0;
            VZReentryS(i)=0;
        else
            VCoordS = G2S_2(launchBLA,[VXReentryG(i),VYReentryG(i),0]);
            VXReentryS(i)=VCoordS(1);
            VYReentryS(i)=VCoordS(2);
            VZReentryS(i)=VCoordS(3);
        end
    end
    
    VXReentryG = [];
    VYReentryG = [];

    %Reentry section��launch����ϵ�������ָ����ĵ�λ����������
	thetaXReentryG=zeros(1,num_point3);%[���������ϳ�]
    thetaYReentryG=zeros(1,num_point3);%[���������ϳ�]
    for i = 1:num_point3
        thetaXReentryG(i)=cos(theta3_t(i)-f3(i)-fFreeEnd+fFreeStart);
        thetaYReentryG(i)=sin(theta3_t(i)-f3(i)-fFreeEnd+fFreeStart);
    end
        
    theta3_t = [];
    f3 = [];
        
	%����ת��Ϊ84����ϵ
	thetaXReentryS=zeros(1,num_point3);%[���������]  �ܷ���x
	thetaYReentryS=zeros(1,num_point3);%[���������]  �ܷ���y
	thetaZReentryS=zeros(1,num_point3);%[���������]  �ܷ���z

    for i = 1:num_point3
        if i > num3
            thetaXReentryS(i)=0;
            thetaYReentryS(i)=0;
            thetaZReentryS(i)=0;
        else
            thetaCoordS = G2S_2(launchBLA,[thetaXReentryG(i),thetaYReentryG(i),0]);
            thetaXReentryS(i)=thetaCoordS(1);
            thetaYReentryS(i)=thetaCoordS(2);
            thetaZReentryS(i)=thetaCoordS(3);
        end
    end
    
    thetaXReentryG = [];
    thetaYReentryG = [];
end
    
    