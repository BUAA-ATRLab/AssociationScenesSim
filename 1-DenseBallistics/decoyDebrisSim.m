%Date：2021.2.1
%Version：decoyDebrisSim.m
%Description: 生成重诱饵、轻诱饵以及碎片轨迹数据
             %重诱饵在自由段和再入段，轻诱饵、碎片仅在自由段
%launchBLA:发射点经纬度以及瞄准方位角
%launchLandLatiLong：发射点以及落点经纬度
%launchWGS84:发射点WGS84坐标系坐标
%numGroup：团编号

function [] = decoyDebrisSim(launchBLA,launchLandLatiLong,launchWGS84,numGroup,settings,environment)
    numTrajectory=settings.iMisNum;
    
    if numGroup==1
        numHeavyDecoy=settings.iSumHeavyDecoyFirst;
        numLightDecoy=settings.iSumLightDecoyFirst;
        numDebris=settings.iSumDebrisFirst;
    elseif numGroup==2
        numHeavyDecoy=settings.iSumHeavyDecoySecond;
        numLightDecoy=settings.iSumLightDecoySecond;
        numDebris=settings.iSumDebrisSecond;
    else
        numHeavyDecoy=settings.iSumHeavyDecoyThird;
        numLightDecoy=settings.iSumLightDecoyThird;
        numDebris=settings.iSumDebrisThird;
    end
        
    BT=launchBLA(1);%天文维度
    LT=launchBLA(2);%天文经度
    AT=launchBLA(3);%天文瞄准方位角
        
    releaseTime=settings.releaseTime; %Decoy释放时刻（Free section开始后的多少秒）单位：s 
        
    str3 = '.\mid\Missiles_Track\PointNumber';
	ch5 = num2str(numTrajectory);
	str4 = [ch5,'.txt'];
    str3A = [str3,str4];
   
    fp_help1 = fopen(str3A, 'r');
    if (fp_help1==-1)
        error(['cannot open ',str3A, '\n']);
    end
    
    read_res = fscanf(fp_help1, '%d %d %d %d %f %f %f', [1, 7]);
    
	num_point1 = read_res(1);
    num_point2 = read_res(2);
    num_point3 = read_res(3);
    numTotal = read_res(4);
    delta_t = read_res(5);
    fFreeStart = read_res(6);
    fFreeEnd = read_res(7);
	fclose(fp_help1);
   
    num_temp=ceil(releaseTime/delta_t);
    numDecoyStart=num_point1+num_temp;	%Decoy释放时航迹点数
	numDecoySum=numTotal-numDecoyStart; %Decoy航迹点数总量(分成两段)，Free section和Reentry section;
	numDecoyFreeSum=numDecoySum-num_point3+1; %Heavy Decoy Free section点数;
    numLightDecoyFreeSum=numDecoyFreeSum; %轻重诱饵同时释放
    
    %1、读入warhead的所有坐标
	t1=zeros(1, numDecoyFreeSum);%Free section时间
    
	%位置
	coordX_84=zeros(1, numDecoyFreeSum); % 总方向x
	coordY_84=zeros(1, numDecoyFreeSum); % 总方向y
	coordZ_84=zeros(1, numDecoyFreeSum); % 总方向z
	
    %速度
	VX_84=zeros(1, numDecoyFreeSum); % 总方向x
	VY_84=zeros(1, numDecoyFreeSum); % 总方向y
	VZ_84=zeros(1, numDecoyFreeSum); % 总方向z

    str7='.\mid\Missiles_Track\warhead';
	ch7=num2str(numTrajectory);
	ch8=num2str(numGroup);%【L-2020/12/25】不同弹头读取文件不同
	ch9='11';
    str8=[ch7,ch8,ch9,'_84.txt'];
    str7A = [str7, str8];
	
	fp=fopen(str7A,'r');
	if fp==-1
		error(['cannot open ',str7A, '\n']);
    end

	if numGroup==1
        read_res1 = fscanf(fp, '%s %s %s %s %s %s %s %s %s %s %s', [1 11]);%滤除描述字符
        read_res = fscanf(fp,' %f %d %f %f %f %f %f %f %f %f %f', [11 inf]);
        
        seqDecoyFree = [numDecoyStart+1:numDecoyStart+numDecoyFreeSum];%重诱饵释放到自由段结束
        t1 = read_res(1,seqDecoyFree);
        coordX_84 = read_res(3,seqDecoyFree);
        coordY_84 = read_res(4,seqDecoyFree);
        coordZ_84 = read_res(5,seqDecoyFree);
        VX_84 = read_res(6,seqDecoyFree);
        VY_84 = read_res(7,seqDecoyFree);
        VZ_84 = read_res(8,seqDecoyFree);        
    else
        %此时为第二第三级弹头释放诱饵过程\
        %ATTENTION:各弹头同时释放诱饵，实际同时！
		tTmp1=numDecoyStart*delta_t;%弹头从地面发射到第一颗弹头释放诱饵经过时间

		read_res1 = fscanf(fp, '%s %s %s %s %s %s %s %s %s %s %s', [1 11]);
        read_res = fscanf(fp,' %f %d %f %f %f %f %f %f %f %f %f', [11 inf]);
        
        tTmp2 = read_res(1,1);%弹头从地面发射到第二个弹头发射经过时间
        numTmp=round((tTmp1-tTmp2)/delta_t);%诱饵释放在第二颗弹头时间序列的编号
        
        seqDecoyFree = [numTmp+1:numTmp+numDecoyFreeSum];
        t1 = read_res(1,seqDecoyFree);
        coordX_84 = read_res(3,seqDecoyFree);
        coordY_84 = read_res(4,seqDecoyFree);
        coordZ_84 = read_res(5,seqDecoyFree);
        VX_84 = read_res(6,seqDecoyFree);
        VY_84 = read_res(7,seqDecoyFree);
        VZ_84 = read_res(8,seqDecoyFree);     
    end
    fclose (fp);
    
    %2.生成 Decoy在相对坐标系的坐标

	wz=0; %warhead绕地心转动角速度
	e=0; %偏心率

    str9='.\mid\Missiles_Track\yer_help';
	ch10=num2str(numTrajectory);
    str10=[ch10,'.txt'];
    str9A = [str9, str10];
    
	fp_help3=fopen(str9A,'r');
	if (fp_help3==-1)
		error(['cannot open ', str9A,'\n']);
    end

    read_res = fscanf(fp_help3,'%f %f',[1 2]);
    wz = read_res(1); 
    e = read_res(2);
	fclose(fp_help3);
	%wz=4.2225945e-4;
    %e=0.8582;
    
    %%%坐标转换：相对坐标系->发射坐标系->大地直角坐标系
    %%%相对坐标实际上是随着弹头运动的发射坐标系，转换为发射坐标系的坐标转换矩阵D(-wt),式（2.3-7）
    %D(-wt)=[-sin a,-cos a,0;-cos a,-sin a,0;0,0,1]
    %【Need to be addressed】 矩阵相乘的顺序？？

	XDecoyFreeS=zeros(1, numDecoyFreeSum);%Decoy的84坐标x
	YDecoyFreeS=zeros(1, numDecoyFreeSum);%Decoy的84坐标Y
	ZDecoyFreeS=zeros(1, numDecoyFreeSum);%Decoy的84坐标Z
	VXDecoyFreeS=zeros(1, numDecoyFreeSum);%Decoy的84速度vx
	VYDecoyFreeS=zeros(1, numDecoyFreeSum);%Decoy的84速度vY
	VZDecoyFreeS=zeros(1, numDecoyFreeSum);%Decoy的84速度vZ
	thetaXDecoyFreeS=zeros(1, numDecoyFreeSum);%Decoy的84角度vx
	thetaYDecoyFreeS=zeros(1, numDecoyFreeSum);%Decoy的84角度vY
	thetaZDecoyFreeS=zeros(1, numDecoyFreeSum);%Decoy的84角度vZ
    
    XDecoyReentryS=zeros(1, num_point3-1);%Decoy的84坐标x
	YDecoyReentryS=zeros(1, num_point3-1);%Decoy的84坐标Y
	ZDecoyReentryS=zeros(1, num_point3-1);%Decoy的84坐标Z
	VXDecoyReentryS=zeros(1, num_point3);%Decoy的84速度vx
	VYDecoyReentryS=zeros(1, num_point3);%Decoy的84速度vY
	VZDecoyReentryS=zeros(1, num_point3);%Decoy的84速度vZ
	thetaXDecoyReentryS=zeros(1, num_point3);%Decoy的84角度vx
	thetaYDecoyReentryS=zeros(1, num_point3);%Decoy的84角度vY
	thetaZDecoyReentryS=zeros(1, num_point3);%Decoy的84角度vZ
    
    %launch坐标系转为84坐标的矩阵元素
    d11=-sin(LT)*sin(AT)-sin(BT)*cos(AT)*cos(LT);
    d12=cos(BT)*cos(LT);
    d13=-sin(LT)*cos(AT)+sin(BT)*sin(AT)*cos(LT);
    d21=cos(LT)*sin(AT)-sin(BT)*cos(AT)*sin(LT);
    d22=sin(LT)*cos(BT);
    d23=cos(LT)*cos(AT)+sin(BT)*sin(AT)*sin(LT);
    d31=cos(BT)*cos(AT);
    d32=sin(BT);
    d33=-sin(AT)*cos(BT);
    d=[d11,d12,d13,d21,d22,d23,d31,d32,d33];
    
    for kkk=1:numHeavyDecoy
    %Free Section
        [XDecoyFreeS,YDecoyFreeS,ZDecoyFreeS,VXDecoyFreeS,VYDecoyFreeS,VZDecoyFreeS,thetaXDecoyFreeS,thetaYDecoyFreeS,thetaZDecoyFreeS]=...
            decoyDebrisFreeSectionSim(2,d,wz,e,kkk,numDecoyFreeSum,launchWGS84,coordX_84,coordY_84,coordZ_84,VX_84,VY_84,VZ_84,settings);

        %再入点地心距
        r_e=sqrt(power(XDecoyFreeS(numDecoyFreeSum),2)+power(YDecoyFreeS(numDecoyFreeSum),2)...
                +power(ZDecoyFreeS(numDecoyFreeSum),2));
        %再入点发射坐标系速度
        [V_eG] = S2G_2(launchBLA,[VXDecoyFreeS(numDecoyFreeSum),VYDecoyFreeS(numDecoyFreeSum),VZDecoyFreeS(numDecoyFreeSum)]);
        V_e = sqrt(power(V_eG(1),2)+power(V_eG(2),2)+power(V_eG(3),2));
        %再入点轨道倾角
        theta_e=atan(V_eG(2)/V_eG(1));
        %再入点高度
        BLH_e=WGS842BLH([XDecoyFreeS(numDecoyFreeSum-1),YDecoyFreeS(numDecoyFreeSum-1),ZDecoyFreeS(numDecoyFreeSum-1)],environment);
        h_e=BLH_e(3);
        %再入点发射坐标系X,Y坐标
        [XYZ_e] =S2G(launchBLA,launchWGS84,[XDecoyFreeS(numDecoyFreeSum),YDecoyFreeS(numDecoyFreeSum),ZDecoyFreeS(numDecoyFreeSum)]); 
        mTarget=settings.mHeavyDecoy;
        
        t2=zeros(1, num_point3);
        t2(1)=t1(numDecoyFreeSum);
        for i=2:num_point3
            t2(i)=t2(1)+(i-1)*delta_t;
        end

        
        [XDecoyReentryS,YDecoyReentryS,ZDecoyReentryS,VXDecoyReentryS,VYDecoyReentryS,VZDecoyReentryS,thetaXDecoyReentryS,thetaYDecoyReentryS,thetaZDecoyReentryS,num_point3]=...
            ReentrySectionSim(r_e,V_e,theta_e,h_e,XYZ_e,num_point3,mTarget,fFreeStart,fFreeEnd,launchBLA,launchLandLatiLong,settings,environment);
    
        %生成编号		
		id=numTrajectory*1000+numGroup*100+2*10+kkk;
		str1='.\mid\Missiles_Track\HeavyDecoy';
		ch1=num2str(numTrajectory);
		ch2=num2str(numGroup);
		ch3='2';
		ch4=num2str(kkk);
        str2=[ch1,ch2,ch3,ch4,'_84.txt'];
        str1A = [str1, str2];

		fp2=fopen(str1A,'w');
		if (fp2==-1)
			error(['cannot open ',str1A,'\n']);
        end
		
		fprintf(fp2,'%10s','t');
		fprintf(fp2,'%5s','');
		fprintf(fp2,'%10s','idBGTN');
		fprintf(fp2,'%5s','');fprintf(fp2,'%15s','coordX_84');fprintf(fp2,'%5s','');fprintf(fp2,'%15s','coordY_84');fprintf(fp2,'%5s','');fprintf(fp2,'%15s','coordZ_84');
		fprintf(fp2,'%5s','');fprintf(fp2,'%15s','VX_84');fprintf(fp2,'%5s','');fprintf(fp2,'%15s','VY_84');fprintf(fp2,'%5s','');fprintf(fp2,'%15s','VZ_84');
		fprintf(fp2,'%5s','');fprintf(fp2,'%15s','directX_84');fprintf(fp2,'%5s','');fprintf(fp2,'%15s','directY_84');fprintf(fp2,'%5s','');fprintf(fp2,'%15s','directZ_84');
		fprintf(fp2,'\n');
        
		for i=1:numDecoyFreeSum
			fprintf(fp2,'%10.4f',t1(i));
			fprintf(fp2,'%5s','');
			fprintf(fp2,'%10.4d',id);
			fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',XDecoyFreeS(i));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',YDecoyFreeS(i));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',ZDecoyFreeS(i));
			fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',VXDecoyFreeS(i));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',VYDecoyFreeS(i));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',VZDecoyFreeS(i));
			fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',thetaXDecoyFreeS(i));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',thetaYDecoyFreeS(i));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',thetaZDecoyFreeS(i));
			fprintf(fp2,'\n');
        end

		for i=2:num_point3
			fprintf(fp2,'%10.4f',t2(i));
			fprintf(fp2,'%5s','');
			fprintf(fp2,'%10.4d',id);
			fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',XDecoyReentryS(i-1));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',YDecoyReentryS(i-1));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',ZDecoyReentryS(i-1));
			fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',VXDecoyReentryS(i));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',VYDecoyReentryS(i));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',VZDecoyReentryS(i));
			fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',thetaXDecoyReentryS(i));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',thetaYDecoyReentryS(i));fprintf(fp2,'%5s','');fprintf(fp2,'%15.5f',thetaZDecoyReentryS(i));
			fprintf(fp2,'\n');
        end
		
		fclose(fp2);
        
        t2=[];
		XDecoyReentryS=[];
		YDecoyReentryS=[];
        ZDecoyReentryS=[];
        VXDecoyReentryS=[];
        VYDecoyReentryS=[];
        VZDecoyReentryS=[];
        thetaXDecoyReentryS=[];
        thetaYDecoyReentryS=[];
        thetaZDecoyReentryS=[];
        
    end
    
    %Light Decoy%【L-2020/12/24】轻诱饵仅有自由段航程
    for kkk=1:numLightDecoy
        [XDecoyFreeS,YDecoyFreeS,ZDecoyFreeS,VXDecoyFreeS,VYDecoyFreeS,VZDecoyFreeS,thetaXDecoyFreeS,thetaYDecoyFreeS,thetaZDecoyFreeS]=...
            decoyDebrisFreeSectionSim(3,d,wz,e,kkk,numLightDecoyFreeSum,launchWGS84,coordX_84,coordY_84,coordZ_84,VX_84,VY_84,VZ_84,settings);
    
        id=numTrajectory*1000+numGroup*100+3*10+kkk;
		str1='.\mid\Missiles_Track\LightDecoy';
		ch1=num2str(numTrajectory);
		ch2=num2str(numGroup);
		ch3='3';
		ch4=num2str(kkk);
        str2=[ch1,ch2,ch3,ch4,'_84.txt'];
        str1A = [str1, str2];

        fp3 = fopen(str1A, 'w');
		if fp3==-1
			error(['cannot open ',str1A,'\n']);
        end
				
		fprintf(fp3,'%10s','t');
		fprintf(fp3,'%5s','');
		fprintf(fp3,'%10s','idBGTN');
		fprintf(fp3,'%5s','');fprintf(fp3,'%15s','coordX_84');fprintf(fp3,'%5s','');fprintf(fp3,'%15s','coordY_84');fprintf(fp3,'%5s','');fprintf(fp3,'%15s','coordZ_84');
		fprintf(fp3,'%5s','');fprintf(fp3,'%15s','VX_84');fprintf(fp3,'%5s','');fprintf(fp3,'%15s','VY_84');fprintf(fp3,'%5s','');fprintf(fp3,'%15s','VZ_84');
		fprintf(fp3,'%5s','');fprintf(fp3,'%15s','directX_84');fprintf(fp3,'%5s','');fprintf(fp3,'%15s','directY_84');fprintf(fp3,'%5s','');fprintf(fp3,'%15s','directZ_84');
		fprintf(fp3,'\n');
		for i=1:numLightDecoyFreeSum
			fprintf(fp3,'%10.4f',t1(i));
			fprintf(fp3,'%5s','');
			fprintf(fp3,'%10.4d',id);
			fprintf(fp3,'%5s','');fprintf(fp3,'%15.5f',XDecoyFreeS(i));fprintf(fp3,'%5s','');fprintf(fp3,'%15.5f',YDecoyFreeS(i));fprintf(fp3,'%5s','');fprintf(fp3,'%15.5f',ZDecoyFreeS(i));
			fprintf(fp3,'%5s','');fprintf(fp3,'%15.5f',VXDecoyFreeS(i));fprintf(fp3,'%5s','');fprintf(fp3,'%15.5f',VYDecoyFreeS(i));fprintf(fp3,'%5s','');fprintf(fp3,'%15.5f',VZDecoyFreeS(i));
			fprintf(fp3,'%5s','');fprintf(fp3,'%15.5f',thetaXDecoyFreeS(i));fprintf(fp3,'%5s','');fprintf(fp3,'%15.5f',thetaYDecoyFreeS(i));fprintf(fp3,'%5s','');fprintf(fp3,'%15.5f',thetaZDecoyFreeS(i));
			fprintf(fp3,'\n');
        end	
		
		fclose(fp3);
    end

    %debris%%【L-2020/12/24】跟轻诱饵相同处理，仅有自由段~~~代码除初值设置不同外与重诱饵完全相同
    for kkk=1:numDebris
         [XDecoyFreeS,YDecoyFreeS,ZDecoyFreeS,VXDecoyFreeS,VYDecoyFreeS,VZDecoyFreeS,thetaXDecoyFreeS,thetaYDecoyFreeS,thetaZDecoyFreeS]=...
            decoyDebrisFreeSectionSim(4,d,wz,e,kkk,numLightDecoyFreeSum,launchWGS84,coordX_84,coordY_84,coordZ_84,VX_84,VY_84,VZ_84,settings);
        
        id=numTrajectory*1000+numGroup*100+4*10+kkk;
		str1='.\mid\Missiles_Track\Debris';
		ch1=num2str(numTrajectory);
		ch2=num2str(numGroup);
		ch3='4';
		ch4=num2str(kkk);
        str2=[ch1,ch2,ch3,ch4,'_84.txt'];
        str1A = [str1, str2];
       
        fp4=fopen(str1A,'w');
        if (fp4==-1)
            error(['cannot open ', str1A,'\n']);
        end
        
        fprintf(fp4,'%10s','t');
        fprintf(fp4,'%5s','');
        fprintf(fp4,'%10s','idBGTN');
        fprintf(fp4,'%5s','');fprintf(fp4,'%15s','coordX_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','coordY_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','coordZ_84');
        fprintf(fp4,'%5s','');fprintf(fp4,'%15s','VX_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','VY_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','VZ_84');
        fprintf(fp4,'%5s','');fprintf(fp4,'%15s','directX_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','directY_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','directZ_84');
        fprintf(fp4,'\n');
        for i=1:numLightDecoyFreeSum
            fprintf(fp4,'%10.4f',t1(i));
            fprintf(fp4,'%5s','');
            fprintf(fp4,'%10.4d',id);
            fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',XDecoyFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',YDecoyFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',ZDecoyFreeS(i));
			fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',VXDecoyFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',VYDecoyFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',VZDecoyFreeS(i));
			fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',thetaXDecoyFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',thetaYDecoyFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',thetaZDecoyFreeS(i));
			fprintf(fp4,'\n');
        end
        
        fclose(fp4);
    end
    
    
    
    
   