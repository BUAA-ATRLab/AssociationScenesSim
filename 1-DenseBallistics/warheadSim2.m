%Date：2021.3.1
%Version：warheadSim2.m
%Description：二级、三级弹头轨迹仿真
%第二或第三个弹头的处理，仅有自由段再入段，与第一个弹头处理完全相同
%相比改变了关机点参数，速度、轨道倾角与第一个弹头相同，仅位置改变
function []=warheadSim2(launchBLA,launchLandLatiLong,numGroup,settings,environment)
    numTrajectory=settings.iMisNum;
    
    if numGroup==2
        numLaunch=settings.numSecondLaunchTime;
    else
        numLaunch=settings.numThirdLaunchTime;
    end
    
    %%%读取各阶段数据点数目
    str3 = '.\mid\Missiles_Track\PointNumber';
    ch5=num2str(numTrajectory);
    str4=[ch5,'.txt'];
    str3A = [str3, str4];

    fp_help1 = fopen(str3A,'r');
    if fp_help1 == -1
        error(['cannot open ',str3A, '\n']);
    end
    
    read_res = fscanf(fp_help1, '%d %d %d %d %f %f %f',[1 7]);
    num_point1 = read_res(1);
    num_point2 = read_res(2);
    num_point3 = read_res(3); 
    num_total = read_res(4);
    delta_t = read_res(5);
    fFreeStart= read_res(6);
    fFreeEnd = read_res(7);
    fclose(fp_help1);
    
    if numGroup== 2
         % 第二个弹头的发射时的真近点角和轨道偏角
        str5 = '.\mid\Missiles_Track\param2_mid';
        ch6 = num2str(numTrajectory);
        str6 = [ch6, '.txt'];
        str5A = [str5, str6];

        fp_help2=fopen(str5A,'r');
        if fp_help2 == -1
            error(['cannot open ',str5A, '\n']);
        end
        read_res = fscanf(fp_help2,'%f %f',[1 2]);
        f_N = read_res(1);
        theta_N = read_res(2);
        fclose(fp_help2);
    else
         % 第三个弹头的发射时的真近点角和轨道偏角
        str5 = '.\mid\Missiles_Track\param3_mid';
        ch6 = num2str(numTrajectory);
        str6 = [ch6, '.txt'];
        str5A = [str5, str6];
        fp_help23=fopen(str5A,'r');
        if fp_help23 == -1
            error(['cannot open ',str5A, '\n']);
        end
        read_res = fscanf(fp_help23,'%f %f',[1,2]);
        f_N = read_res(1);
        theta_N = read_res(2);
        fclose(fp_help23);
    end
    
     if numGroup == 2
        %第二个warhead起始时刻，第一个弹头的位置，速度，trajectory倾角（launch坐标系下）
        str5 = '.\mid\Missiles_Track\warhead2_start';
        ch6 = num2str(numTrajectory);
        str6 = [ch6, '.txt'];
        str5A = [str5, str6];

        fp_help3=fopen(str5A,'r');
        if fp_help3 == -1
            error(['cannot open ',str5A, '\n']);
        end
        read_res = fscanf(fp_help3,'%f %f %f %f %f %f %f %f %f %f',[1 10]);
        tStart = read_res(1);
        XStart = read_res(2);
        YStart = read_res(3);
        ZStart = read_res(4);
        VXStart = read_res(5);
        VYStart = read_res(6);
        VZStart = read_res(7);
        thetaXStart = read_res(8);
        thetaYStart = read_res(9);
        thetaZStart = read_res(10);
        fclose(fp_help3);
     else
         %第三个warhead起始时刻，第一个弹头的位置，速度，trajectory倾角（launch坐标系下）
        str5 = '.\mid\Missiles_Track\warhead3_start';
        ch6 = num2str(numTrajectory);
        str6 = [ch6, '.txt'];
        str5A = [str5, str6];

        fp_help33=fopen(str5A,'r');
        if fp_help33 == -1
            error(['cannot open ',str5A, '\n']);
        end
        read_res = fscanf(fp_help33,'%f %f %f %f %f %f %f %f %f %f',[1,10]);
        tStart = read_res(1);
        XStart = read_res(2);
        YStart = read_res(3);
        ZStart = read_res(4);
        VXStart = read_res(5);
        VYStart = read_res(6);
        VZStart = read_res(7);
        thetaXStart = read_res(8);
        thetaYStart = read_res(9);
        thetaZStart = read_res(10);
        fclose(fp_help33);
     end
    
    numStart=num_point1+numLaunch-1;        % 第二个warhead的起始在第一个warhead航迹里的点数
    num_total2=num_total-numStart+1;         % 第二个warhead的航迹总点数
    num_total2=floor(num_total2*1.5);         %【L-2020/12/24】*1.5 防止落点不在地面
    
    t_total = zeros(1,num_total2);           % 存放第二个warhead航迹的时间点 [保留到最后]

    str7 = '.\mid\Missiles_Track\time';
    ch7 = num2str(numTrajectory);
    str8 = [ch7, '.txt'];
    str7A = [str7, str8];

    fp_help4=fopen(str7A,'r');
    if fp_help4 == -1
        error(['cannot open ',str7A, '\n']);
    end
    
    read_res = fscanf(fp_help4,' %f', [1 inf]);

    seqUsed = [numStart:num_total];
    t_total(seqUsed-numStart+1) = read_res(1,seqUsed);
    for i=num_total-numStart+2:num_total2
        t_total(i)=t_total(num_total-numStart+1)+(i-(num_total-numStart+1))*delta_t;
    end
    
    fclose(fp_help4);
    
    %二三级弹头起始时 第一个弹头的位置
    PosiStartWarheadFirst = zeros(1, 3);            
    PosiStartWarheadFirst(1)=XStart;
	PosiStartWarheadFirst(2)=YStart;
	PosiStartWarheadFirst(3)=ZStart;
    %第二个warhead的起始速度(与第一个弹头此时速度相同)
	V_N= sqrt(power(VXStart,2)+power(VYStart,2)+power(VZStart,2)); 
    
    %%%%%%%%%%Free section%%%%%%%%%%%%%%%%%%%%%
    %第二个warhead的launch时间
    TLaunch=t_total(1);
    if numGroup == 2
        % 第二个warhead起点所处的位置。在第一个warhead的missile body坐标系下的坐标,单位：米
        PosiStartB=[settings.XOffsetSecondWarheadB,settings.YOffsetSecondWarheadB,settings.ZOffsetSecondWarheadB];
    else
        PosiStartB=[settings.XOffsetThirdWarheadB,settings.YOffsetThirdWarheadB,settings.ZOffsetThirdWarheadB];
    end

    thetaStart = [theta_N-f_N+fFreeStart,0.0,0.0]; %【L-2020/12/25】参考missile body的俯仰角、偏航角、滚动角
    %第二个warhead 起点发射坐标系下的位置
    PosiStartG = B2G(PosiStartWarheadFirst,thetaStart,PosiStartB);
    
     %第二个warhead的起点在地心坐标系下的坐标
    PosiStartS = G2S(launchBLA,launchLandLatiLong,PosiStartG,environment);
     %第二个warhead起始点的地心距
    R_N=sqrt(power(PosiStartS(1),2)+power(PosiStartS(2),2)+power(PosiStartS(3),2));
    
    %%%椭圆trajectory参数
    fM=environment.fM; %地心引力常数
	v_energy2=(V_N^2)*R_N/fM;%能量参数,P184（trajectory missile trajectory学P182）
	P_semi_p2=R_N*v_energy2*(cos(theta_N)^2);%半通径,P183
	e_e2=sqrt(1+v_energy2*(v_energy2-2)*(cos(theta_N)^2));%偏心率e,P185
	a_a2=P_semi_p2/(1-(e_e2^2));%椭圆长半轴a,P188
	b_b2=P_semi_p2/sqrt(1-(e_e2^2));%椭圆短半轴b,P188
	E_k2=acos((1-R_N/a_a2)/e_e2);%偏近点角Ek,P199,P203
	T_cycle2=2*pi*(a_a2^1.5)/sqrt(fM);%质点绕椭圆运行一周的时间,P197
	t_p2=TLaunch-T_cycle2*(E_k2-e_e2*sin(E_k2))/(2*pi);%missile飞经近地点的时间tp,P200

    num_point_22=num_point2-numLaunch+1;%第二个warheadFree section的点数
    
    t_22 = t_total(1:num_point_22);%第二个目标在Free section的时间点[保留到Free section结束]

    E2_t0=0;
    E2_t1 = zeros(1,num_point_22); % [保留到任一时刻trajectory倾角计算出来]
    for i2=1:num_point_22
        E2_t0=E_k2;%初始化Et
        E2_t1(i2)=E2_t0+((t_22(i2)-t_p2)*2*pi/T_cycle2-E2_t0+e_e2*sin(E2_t0))/(1-e_e2*cos(E2_t0));
        while (abs(E2_t1(i2)-E2_t0)/abs(E2_t1(i2))>1e-8)
            E2_t0=E2_t1(i2);
            E2_t1(i2)=E2_t0+((t_22(i2)-t_p2)*2*pi/T_cycle2-E2_t0+e_e2*sin(E2_t0))/(1-e_e2*cos(E2_t0));
        end
    end

    %任一时刻地心距
	%任一时刻速度
	%任一时刻trajectory倾角
    rFree2=a_a2.*(1-e_e2.*cos(E2_t1(1:num_point_22))); % 地心距
    VFree2=sqrt(fM/a_a2).*sqrt(1-(e_e2^2).*(cos(E2_t1(1:num_point_22)).^2))./(1-e_e2.*cos(E2_t1(1:num_point_22)));
    thetaFree2=atan(e_e2.*sin(E2_t1(1:num_point_22))./sqrt(1-(e_e2^2)));
    
    fFree2=acos((P_semi_p2./rFree2-1)./e_e2); %每一点的真近点角，P185
    fFree2End=fFree2(num_point_22);%Free section最后一点的真近点角值（Reentry section用）
    
    E2_t1 = [];
    
    %%%发射坐标系位置坐标
    XFree2G=zeros(1, num_point_22);%[保留到最后合成]
    YFree2G=zeros(1, num_point_22);%[保留到最后合成]
    ZFree2G=PosiStartG(3); %z方向上一直保持此值，因为z方向上没速度
    XFree2G(1)=PosiStartG(1);
	YFree2G(1)=PosiStartG(2);

    for i2=2:num_point_22
		XFree2G(i2)=XFree2G(i2-1)+VFree2(i2-1)*cos(thetaFree2(i2-1)-(fFree2(i2-1)-fFreeStart))*(t_22(i2)-t_22(i2-1));
		YFree2G(i2)=YFree2G(i2-1)+VFree2(i2-1)*sin(thetaFree2(i2-1)-(fFree2(i2-1)-fFreeStart))*(t_22(i2)-t_22(i2-1));
    end
    
    % 位置转换为84坐标系
    XFree2S = zeros(1,num_point_22); % [保留到最后合成]
    YFree2S = zeros(1,num_point_22); % [保留到最后合成]
    ZFree2S = zeros(1,num_point_22); % [保留到最后合成]
    
    for i2 = 1:num_point_22
		coordS = G2S(launchBLA,launchLandLatiLong,[XFree2G(i2),YFree2G(i2),ZFree2G],environment);
		rTmp=sqrt(coordS(1)^2 + coordS(2)^2 + coordS(3)^2); 
		XFree2S(i2)=rFree2(i2)/rTmp*coordS(1);
		YFree2S(i2)=rFree2(i2)/rTmp*coordS(2);
		ZFree2S(i2)=rFree2(i2)/rTmp*coordS(3);
    end

    %launch坐标系下速度
    VXFree2G = VFree2(1:num_point_22).*cos(thetaFree2(1:num_point_22)-fFree2(1:num_point_22)+fFreeStart);
    VYFree2G = VFree2(1:num_point_22).*sin(thetaFree2(1:num_point_22)-fFree2(1:num_point_22)+fFreeStart);    
    %速度转换到84坐标下
    VXFree2S=zeros(1, num_point_22); 
    VYFree2S=zeros(1, num_point_22); 
    VZFree2S=zeros(1, num_point_22);

    for i2=1:num_point_22
        VCoordS = G2S_2(launchBLA,[VXFree2G(i2),VYFree2G(i2),0]);
        VXFree2S(i2)=VCoordS(1);
        VYFree2S(i2)=VCoordS(2);
        VZFree2S(i2)=VCoordS(3);
    end
    VXFree2G = [];
    VYFree2G = [];
    
    %launch坐标系下倾角所指方向的单位向量的坐标
    thetaXFree2G = cos(thetaFree2(1:num_point_22)-fFree2(1:num_point_22)+fFreeStart);
    thetaYFree2G = sin(thetaFree2(1:num_point_22)-fFree2(1:num_point_22)+fFreeStart);	
    %倾角转换到84坐标下
    thetaXFree2S=zeros(1, num_point_22); 
    thetaYFree2S=zeros(1, num_point_22); 
    thetaZFree2S=zeros(1, num_point_22);

    for i2=1:num_point_22
        VCoordS = G2S_2(launchBLA,[thetaXFree2G(i2),thetaYFree2G(i2),0]);
        thetaXFree2S(i2)=VCoordS(1);
        thetaYFree2S(i2)=VCoordS(2);
        thetaZFree2S(i2)=VCoordS(3);
    end
    thetaXFree2G = [];
    thetaYFree2G = [];
    
    fFree2 = [];

    %%%%%%%%%%Reentry section%%%%%%%%%%%%%%%%%%%%%
    % 第二个warheadReentry section采样点个数
    num_point_32=num_total2-num_point_22+1;
    % 第二个warhead的Reentry section采样时间点
    t_32 = t_total(num_point_22:num_point_22+num_point_32-1);
    
    XReentry2S = zeros(1,num_point_32-1);%[保留到最后合成]
    YReentry2S = zeros(1,num_point_32-1);%[保留到最后合成]
    ZReentry2S = zeros(1,num_point_32-1);%[保留到最后合成]
    VXReentry2S = zeros(1,num_point_32);%[保留到最后合成]
    VYReentry2S = zeros(1,num_point_32);%[保留到最后合成]
    VZReentry2S = zeros(1,num_point_32);%[保留到最后合成]
    thetaXReentry2S = zeros(1,num_point_32);%[保留到最后合成]
    thetaYReentry2S = zeros(1,num_point_32);%[保留到最后合成]
    thetaZReentry2S = zeros(1,num_point_32);%[保留到最后合成]
    
    %再入点地心距，速度，轨道倾角
    R_e=rFree2(num_point_22);
    V_e=VFree2(num_point_22);
    theta_e=thetaFree2(num_point_22);
    
    %再入点地心直角坐标
    BLH_e=WGS842BLH([XFree2S(num_point_22),YFree2S(num_point_22),ZFree2S(num_point_22)],environment);
    h_e=BLH_e(3);
    XYZ_e=[XFree2G(num_point_22),YFree2G(num_point_22),ZFree2G];
    
    rFree2=[];
	VFree2=[];
	thetaFree2=[];
    
    mTarget=settings.mWarhead2;
    
    [XReentry2S,YReentry2S,ZReentry2S,VXReentry2S,VYReentry2S,VZReentry2S,thetaXReentry2S,thetaYReentry2S,thetaZReentry2S,num_point_32]=...
            ReentrySectionSim(R_e,V_e,theta_e,h_e,XYZ_e,num_point_32,mTarget,fFreeStart,fFree2End,launchBLA,launchLandLatiLong,settings,environment);

    
    %////////////////////最终生成/////////////////////////////////
    num_total2=num_point_22+num_point_32-1;

    %位置(大地直角坐标系)
    coordX2_84 = zeros(1,num_total2);%[保留到最后] 总坐标x   
    coordY2_84 = zeros(1,num_total2);%[保留到最后] 总坐标y    
    coordZ2_84 = zeros(1,num_total2);%[保留到最后] 总坐标z
    for i2=1:num_point_22
		coordX2_84(i2)=XFree2S(i2);
		coordY2_84(i2)=YFree2S(i2);
		coordZ2_84(i2)=ZFree2S(i2);
    end
    
	XFree2S=[];
	YFree2S=[];
	ZFree2S=[];

	for i2=2:num_point_32
		coordX2_84(num_point_22-1+i2)=XReentry2S(i2-1);
		coordY2_84(num_point_22-1+i2)=YReentry2S(i2-1);
		coordZ2_84(num_point_22-1+i2)=ZReentry2S(i2-1);
    end
    
	XReentry2S=[];
	YReentry2S=[];
	ZReentry2S=[];
    
	%速度
    VX2_84 = zeros(1,num_total2);%[保留到最后] 总速度x
    VY2_84 = zeros(1,num_total2);%[保留到最后] 总速度y
    VZ2_84 = zeros(1,num_total2);%[保留到最后] 总速度z
	
	for i2=1:num_point_22
		VX2_84(i2)=VXFree2S(i2);
		VY2_84(i2)=VYFree2S(i2);
		VZ2_84(i2)=VZFree2S(i2);
    end
    
	VXFree2S=[];
	VYFree2S=[];
    VZFree2S=[];

	for i2=2:num_point_32
		VX2_84(num_point_22-1+i2)=VXReentry2S(i2);
		VY2_84(num_point_22-1+i2)=VYReentry2S(i2);
		VZ2_84(num_point_22-1+i2)=VZReentry2S(i2);
    end
    
	VXReentry2S=[];
	VYReentry2S=[];
	VZReentry2S=[];

    %方向(倾角所指方向的单位向量的坐标)
    directX2_84 = zeros(1,num_total2);%[保留到最后] 总方向x   
    directY2_84 = zeros(1,num_total2);%[保留到最后] 总方向y    
    directZ2_84 = zeros(1,num_total2);%[保留到最后] 总方向z

	for i2=1:num_point_22
		directX2_84(i2)=thetaXFree2S(i2);
		directY2_84(i2)=thetaYFree2S(i2);
		directZ2_84(i2)=thetaZFree2S(i2);
    end
    thetaXFree2S=[];
    thetaYFree2S=[];
    thetaZFree2S=[];
	
	for i2=2:num_point_32
		directX2_84(num_point_22-1+i2)=thetaXReentry2S(i2);
		directY2_84(num_point_22-1+i2)=thetaYReentry2S(i2);
		directZ2_84(num_point_22-1+i2)=thetaZReentry2S(i2);
    end
    thetaXReentry2S=[];
    thetaYReentry2S=[];
    thetaZReentry2S=[];

     %////////////////////文件存储/////////////////////////////////
    id=numTrajectory*1000+numGroup*100+1*10+1;
    str9='.\mid\Missiles_Track\warhead';
    ch1 = num2str(numTrajectory);
    ch2 = num2str(numGroup);
    ch3 = '11';
    str10 = [ ch1, ch2, ch3, '_84.txt'];
    str9A = [str9, str10];

    fp=fopen(str9A,'w');
    if fp == -1
        error(['cannot open ',str9A, '\n']);
    end
    fprintf(fp,'%10s','t');
	fprintf(fp,'%5s','');
	fprintf(fp,'%10s','idBGTN');
	fprintf(fp,'%5s','');fprintf(fp,'%15s','coordX_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','coordY_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','coordZ_84');
	fprintf(fp,'%5s','');fprintf(fp,'%15s','VX_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','VY_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','VZ_84');
	fprintf(fp,'%5s','');fprintf(fp,'%15s','directX_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','directY_84');fprintf(fp,'%5s','');fprintf(fp,'%15s','directZ_84');
    fprintf(fp,'\n');
    
    for i2=1:num_total2        
        fprintf(fp,'%10.4f',t_total(i2));
        fprintf(fp,'%5s','');
        fprintf(fp,'%10.4d',id);
        fprintf(fp,'%5s','');fprintf(fp,'%15.5f',coordX2_84(i2));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',coordY2_84(i2));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',coordZ2_84(i2));
        fprintf(fp,'%5s','');fprintf(fp,'%15.5f',VX2_84(i2));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',VY2_84(i2));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',VZ2_84(i2));
        fprintf(fp,'%5s','');fprintf(fp,'%15.5f',directX2_84(i2));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',directY2_84(i2));fprintf(fp,'%5s','');fprintf(fp,'%15.5f',directZ2_84(i2));
        fprintf(fp,'\n');
    end
    fclose(fp);
    
    t_total=[];
	coordX2_84=[];
	coordY2_84=[];
	coordZ2_84=[];
	VX2_84=[];
	VY2_84=[];
	VZ2_84=[];
	directX2_84=[];
	directY2_84=[];
	directZ2_84=[];	

end


    

     