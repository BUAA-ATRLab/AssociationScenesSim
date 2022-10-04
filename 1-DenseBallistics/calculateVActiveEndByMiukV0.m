%Date��2021.1.31
%Version��calculateVActiveEndByMiukV0.m
%Description:����miuk,v0�����������յ�V,Vx,Vy

function [V,VV,Vx,Vy,thetaX,thetaY]=calculateVActiveEndByMiukV0(miuk,v0,betaPassiveSegment,settings,environment)
    Pb0=settings.Pb0;
    arfa=settings.arfa;
    PM=settings.PM;
    T0=v0*Pb0;  %����ʱ��ĸ�������
    g0=environment.g0;
    %Active Segment�յ�Ĺ����ǣ���λ����
    %������Ϊrk=R,��theta_KOPT=(pi-beta_c)/4
    theta_k=pi/4-betaPassiveSegment/4;
    delta_t=settings.delta_t;
    
    t_k=T0*(1-miuk);        %Active Segment����ʱ��
    num_point1=floor(t_k/delta_t)+1;
     
    t=zeros(1,num_point1);%[���������ϳ�]%ʱ�������
    miu=zeros(1,num_point1);%����������P264�� [������Active Segment����]

    for i=1:num_point1
        t(i)=(i-1)*delta_t;
        miu(i)=1-t(i)/T0;
    end
    
    %%%%%%%%%���%%%%%%%%%%%
    theta_miu = zeros(1,num_point1);%[�������ٶȼ������]
    I1 = zeros(1,num_point1);%[������delta_v1�������]
    
    for i1=1:num_point1
        if miu(i1)<=1 && miu(i1)>=0.95
            theta_miu(i1)=pi/2;
            I1(i1)=1-miu(i1);
        elseif miu(i1)>=0.45
            theta_miu(i1)=4*(pi/2-theta_k)*power((miu(i1)-0.45),2)+theta_k;
            I1(i1)=0.05+integral1(0.95,miu(i1));
        else
            theta_miu(i1)=theta_k;
            I1(i1)=0.05+integral1(0.95,0.45)+(0.45-miu(i1))*sin(theta_k);	
        end
    end
    
     %launch����ϵ�������ָ����ĵ�λ����������   
     thetaX_1 = zeros(1,num_point1);%[���������ϳ�]
     thetaY_1 = zeros(1,num_point1);%[���������ϳ�]

     for i1=1:num_point1
         thetaX_1(i1)=cos(theta_miu(i1));
         thetaY_1(i1)=sin(theta_miu(i1));
     end
    
    %%%%%%%%%%%%%%%�ٶ�%%%%%%%%%%%%%%/

    %%%����Vu
    Vu = zeros(1,num_point1);%[�������ٶȼ������]
    Vu = -g0*Pb0*arfa*log(miu);
    
    %%%����delta_v1
    delta_v1 = zeros(1,num_point1);%[�������ٶȼ������]
    delta_v1 = T0*g0*I1;

    I1=[];

    %%%����delta_v2
    midVarV = zeros(1,num_point1);%[������I2�������]
    h = zeros(1,num_point1);%[������atmos������]

    %һ�׽����ٶ�Vu-deta_v1
    midVarV = Vu-delta_v1;

    for i1=1:num_point1
        if miu(i1)==1
            h(i1)=0;        
        elseif miu(i1)<1 && miu(i1)>=0.95
            h(i1)=h(i1-1)+T0*midVarV(i1)*(miu(i1-1)-miu(i1));
        elseif miu(i1)>=0.45
            h(i1)=h(i1-1)+T0*midVarV(i1)*integral1(miu(i1-1),miu(i1));
        else
            h(i1)=h(i1-1)+T0*sin(theta_k)*midVarV(i1)*(miu(i1-1)-miu(i1));   
        end
    end              
            
    rou = zeros(1,num_point1);%�����洢����õ��Ķ�Ӧ��ͬ�߶ȵ��ܶȡ����١�ѹǿ��%[������I2�������]
    sonic = zeros(1,num_point1);%[������Ma�������]
    p_p0 = zeros(1,num_point1);%[������I3�������]
    
    for i=1:num_point1
        [rou(i),sonic(i),p_p0(i)] = calculateRSP(h(i),environment);
    end
    h=[];

    Ma = zeros(1,num_point1);%�����%[������Cx_ma�������]
    Ma = midVarV./sonic;

    sonic=[];

    Cx_ma = zeros(1,num_point1);%[������I2�������]
    %����Cxֻ������������Ĺ�ϵ
    for i=1:num_point1
        if Ma(i)<0.8 && Ma(i)>=0
            Cx_ma(i)=0.029;
        elseif Ma(i)<=1.068
            Cx_ma(i)=0.1*Ma(i)-0.051;
        else
            Cx_ma(i)=0.09+0.05/Ma(i); 
        end
    end

    Ma=[];

    I2 = zeros(1,num_point1);%[������delta_v2�������]

    I2(1)=0;
    for i=2:num_point1
        I2(i)=I2(i-1)+0.5*Cx_ma(i)*rou(i)*power(midVarV(i),2)*(log(miu(i-1))-log(miu(i)));
    end

    midVarV=[];
    rou=[];
    Cx_ma=[];

    delta_v2 = zeros(1,num_point1);%[�������ٶȼ������]
    delta_v2 = I2*T0*g0/PM;
    
    I2=[];

    %����delta_v3
    I3 = zeros(1,num_point1);%[������delta_v3�������]
    
    I3(1)=0;
    for i=2:num_point1
        I3(i)=I3(i-1)+p_p0(i)*(log(miu(i-1))-log(miu(i)));
    end

    p_p0=[];

    delta_v3 = zeros(1,num_point1);%[�������ٶȼ������]
	delta_v3 = I3*Pb0*g0*(arfa-1.0);
    
    I3=[];

    V_total = zeros(1,num_point1);%[������Active Segment�յ��ٶȼ������]
 	VX_1 = zeros(1,num_point1);%[���������ϳ�]
 	VY_1 = zeros(1,num_point1);%[���������ϳ�]


	for i=1:num_point1
        V_total(i)=Vu(i)-delta_v1(i)-delta_v2(i)-delta_v3(i);
        VX_1(i)=V_total(i)*cos(theta_miu(i));
        VY_1(i)=V_total(i)*sin(theta_miu(i));
    end
    
    V=V_total;
    VV=V_total(num_point1);
    Vx=VX_1;
    Vy=VY_1;
    thetaX=thetaX_1;
    thetaY=thetaY_1;
    
    theta_miu=[];
    Vu=[];
    delta_v1=[];
    delta_v2=[];
    delta_v3=[];
    V_total=[];
end


