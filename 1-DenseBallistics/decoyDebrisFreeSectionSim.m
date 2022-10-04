%Date：2021.2.28
%Version：decoyDebrisFreeSectionSim.m
%Description: 生成单个重诱饵、轻诱饵以及碎片自由段轨迹数据，整合成函数提高代码重复利用率
%输出84坐标系下X,Y,Z以及VX，VY，VZ，thetaX,thetaY,thetaZ
function [XDecoyS,YDecoyS,ZDecoyS,VXDecoyS,VYDecoyS,VZDecoyS,thetaXDecoyS,thetaYDecoyS,thetaZDecoyS]=...
        decoyDebrisFreeSectionSim(markstring,d,wz,e,kkk,numDecoyFreeSum,launchWGS84,coordX_84,coordY_84,coordZ_84,VX_84,VY_84,VZ_84,settings)
    
    delta_t=settings.delta_t;
    
    d11=d(1);d12=d(2);d13=d(3);
    d21=d(4);d22=d(5);d23=d(6);
    d31=d(7);d32=d(8);d33=d(9);
    
    XDecoyReleative=zeros(1, numDecoyFreeSum);%Decoy的相对坐标x
	YDecoyReleative=zeros(1, numDecoyFreeSum);%Decoy的相对坐标Y
	ZDecoyReleative=zeros(1, numDecoyFreeSum);%Decoy的相对坐标Z
	VXDecoyReleative=zeros(1, numDecoyFreeSum);%Decoy的相对速度vx
	VYDecoyReleative=zeros(1, numDecoyFreeSum);%Decoy的相对速度vY
	VZDecoyReleative=zeros(1, numDecoyFreeSum);%Decoy的相对速度vZ
    
    XDecoyS=zeros(1, numDecoyFreeSum);%Decoy的84坐标x
	YDecoyS=zeros(1, numDecoyFreeSum);%Decoy的84坐标Y
	ZDecoyS=zeros(1, numDecoyFreeSum);%Decoy的84坐标Z
	VXDecoyS=zeros(1, numDecoyFreeSum);%Decoy的84速度vx
	VYDecoyS=zeros(1, numDecoyFreeSum);%Decoy的84速度vY
	VZDecoyS=zeros(1, numDecoyFreeSum);%Decoy的84速度vZ
	thetaXDecoyS=zeros(1, numDecoyFreeSum);%Decoy的84角度vx
	thetaYDecoyS=zeros(1, numDecoyFreeSum);%Decoy的84角度vY
	thetaZDecoyS=zeros(1, numDecoyFreeSum);%Decoy的84角度vZ
   
    k11=0;k12=0;k13=0;k14=0;k15=0;k16=0;
    k21=0;k22=0;k23=0;k24=0;k25=0;k26=0;
    k31=0;k32=0;k33=0;k34=0;k35=0;k36=0;
    k41=0;k42=0;k43=0;k44=0;k45=0;k46=0;
        
    arfa = 0; %发射点地心连线与warhead地心连线的夹角
        
    zh11=0;zh12=0;zh13=0;
    zh21=0;zh22=0;zh23=0;
    zh31=0;zh32=0;zh33=0; %转换矩阵的元素
        
    %坐标转换矩阵
	arfa=acos((launchWGS84(1)*coordX_84(1)+launchWGS84(2)*coordY_84(1)+launchWGS84(3)*coordZ_84(1))/...
        sqrt(power(launchWGS84(1),2)+power(launchWGS84(2),2)+power(launchWGS84(3),2))/...
        sqrt(power(coordX_84(1),2)+power(coordY_84(1),2)+power(coordZ_84(1),2)));
        
    zh11=sin(arfa)*d11-cos(arfa)*d12;
    zh12=-cos(arfa)*d11-sin(arfa)*d12;
    zh13=-d13;
    zh21=sin(arfa)*d21-cos(arfa)*d22;
    zh22=-cos(arfa)*d21-sin(arfa)*d22;
    zh23=-d23;
    zh31=sin(arfa)*d31-cos(arfa)*d32;
    zh32=-cos(arfa)*d31-sin(arfa)*d32;
    zh33=-d33;
    if markstring==2
        switch kkk
            %相对坐标——Decoy
            %【L-2020/12/23】设置初值【位置偏移为0，且垂直方向无速度偏移】
            case 1  %第一个Heavy Decoy
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=settings.VxOffsetHeavyDecoyFirst;
                VYDecoyReleative(1)=0;
                VZDecoyReleative(1)=settings.VzOffsetHeavyDecoyFirst;
            case 2  %第二个Heavy Decoy
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=settings.VxOffsetHeavyDecoySecond;
                VYDecoyReleative(1)=0;
                VZDecoyReleative(1)=settings.VzOffsetHeavyDecoySecond;
            case 3  %第三个Heavy Decoy
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=settings.VxOffsetHeavyDecoyThird;
                VYDecoyReleative(1)=0;
                VZDecoyReleative(1)=settings.VzOffsetHeavyDecoyThird;
            case 4  %第四个Heavy Decoy
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=settings.VxOffsetHeavyDecoyFourth;
                VYDecoyReleative(1)=0;
                VZDecoyReleative(1)=settings.VxOffsetHeavyDecoyFourth;
        end
    elseif markstring==3
        switch kkk
            case 1  
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=settings.VxOffsetLightDecoyFirst;
                VYDecoyReleative(1)=0;
                VZDecoyReleative(1)=settings.VzOffsetLightDecoyFirst;
            case 2
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=settings.VxOffsetLightDecoySecond;
                VYDecoyReleative(1)=0;
                VZDecoyReleative(1)=settings.VzOffsetLightDecoySecond;
            case 3 
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=settings.VxOffsetLightDecoyThird;
                VYDecoyReleative(1)=0;
                VZDecoyReleative(1)=settings.VzOffsetLightDecoyThird;
            case 4
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=settings.VxOffsetLightDecoyFourth;
                VYDecoyReleative(1)=0;
                VZDecoyReleative(1)=settings.VxOffsetLightDecoyFourth;
        end
    elseif markstring==4
        low = settings.VOffsetDebrisLower;
        up = settings.VOffsetDebrisUpper;
        switch kkk
            case 1  
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=rand_num(low,up);
                VYDecoyReleative(1)=rand_num(low,up);
                VZDecoyReleative(1)=rand_num(low,up);
            case 2
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=rand_num(low,up);
                VYDecoyReleative(1)=rand_num(low,up);
                VZDecoyReleative(1)=rand_num(low,up);
            case 3 
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=rand_num(low,up);
                VYDecoyReleative(1)=rand_num(low,up);
                VZDecoyReleative(1)=rand_num(low,up);
            case 4
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=rand_num(low,up);
                VYDecoyReleative(1)=rand_num(low,up);
                VZDecoyReleative(1)=rand_num(low,up);
        end
    elseif markstring==5
        low = settings.VOffsetPiecesLower;
        up = settings.VOffsetPiecesUpper;
        switch kkk
            case 1  
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=rand_num(low,up);
                VYDecoyReleative(1)=rand_num(low,up);
                VZDecoyReleative(1)=rand_num(low,up);
            case 2
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=rand_num(low,up);
                VYDecoyReleative(1)=rand_num(low,up);
                VZDecoyReleative(1)=rand_num(low,up);
            case 3 
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=rand_num(low,up);
                VYDecoyReleative(1)=rand_num(low,up);
                VZDecoyReleative(1)=rand_num(low,up);
            case 4
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=rand_num(low,up);
                VYDecoyReleative(1)=rand_num(low,up);
                VZDecoyReleative(1)=rand_num(low,up);
            case 5
                XDecoyReleative(1)=0;
                YDecoyReleative(1)=0;
                ZDecoyReleative(1)=0;
                VXDecoyReleative(1)=rand_num(low,up);
                VYDecoyReleative(1)=rand_num(low,up);
                VZDecoyReleative(1)=rand_num(low,up);
        end
    end

    %84坐标系——Decoy
    XDecoyS(1)=zh11*XDecoyReleative(1)+zh12*YDecoyReleative(1)+zh13*ZDecoyReleative(1)+coordX_84(1);
    YDecoyS(1)=zh21*XDecoyReleative(1)+zh22*YDecoyReleative(1)+zh23*ZDecoyReleative(1)+coordY_84(1);
    ZDecoyS(1)=zh31*XDecoyReleative(1)+zh32*YDecoyReleative(1)+zh33*ZDecoyReleative(1)+coordZ_84(1);
    VXDecoyS(1)=zh11*VXDecoyReleative(1)+zh12*VYDecoyReleative(1)+zh13*VZDecoyReleative(1)+VX_84(1);
    VYDecoyS(1)=zh21*VXDecoyReleative(1)+zh22*VYDecoyReleative(1)+zh23*VZDecoyReleative(1)+VY_84(1);
    VZDecoyS(1)=zh31*VXDecoyReleative(1)+zh32*VYDecoyReleative(1)+zh33*VZDecoyReleative(1)+VZ_84(1);
    VDecoyS=sqrt(power(VXDecoyS(1),2)+power(VYDecoyS(1),2)+power(VZDecoyS(1),2));
    thetaXDecoyS(1)=VXDecoyS(1)/VDecoyS;
    thetaYDecoyS(1)=VYDecoyS(1)/VDecoyS;
    thetaZDecoyS(1)=VZDecoyS(1)/VDecoyS;

    for i = 2:numDecoyFreeSum
        %坐标转换矩阵
        arfa=acos((launchWGS84(1)*coordX_84(i)+launchWGS84(2)*coordY_84(i)+launchWGS84(3)*coordZ_84(i))/...
            sqrt(power(launchWGS84(1),2)+power(launchWGS84(2),2)+power(launchWGS84(3),2))/...
            sqrt(power(coordX_84(i),2)+power(coordY_84(i),2)+power(coordZ_84(i),2)));

        zh11=sin(arfa)*d11-cos(arfa)*d12;
        zh12=-cos(arfa)*d11-sin(arfa)*d12;
        zh13=-d13;
        zh21=sin(arfa)*d21-cos(arfa)*d22;
        zh22=-cos(arfa)*d21-sin(arfa)*d22;
        zh23=-d23;
        zh31=sin(arfa)*d31-cos(arfa)*d32;
        zh32=-cos(arfa)*d31-sin(arfa)*d32;
        zh33=-d33;

        %相对坐标——Decoy
        %【L-2020/12/23】四阶龙格库塔法迭代求解
        k11=VXDecoyReleative(i-1);
        k12=VYDecoyReleative(i-1);
        k13=VZDecoyReleative(i-1);
        k14=2*wz*VYDecoyReleative(i-1)+(3-e)/(1-e)*(wz^2)*XDecoyReleative(i-1);
        k15=-2*wz*VXDecoyReleative(i-1)-e/(1-e)*(wz^2)*YDecoyReleative(i-1);
        k16=-1/(1-e)*(wz^2)*ZDecoyReleative(i-1);

        k21=VXDecoyReleative(i-1)+0.5*delta_t*k14;
        k22=VYDecoyReleative(i-1)+0.5*delta_t*k15;
        k23=VZDecoyReleative(i-1)+0.5*delta_t*k16;
        k24=2*wz*(VYDecoyReleative(i-1)+0.5*delta_t*k15)+(3-e)/(1-e)*(wz^2)*(XDecoyReleative(i-1)+0.5*delta_t*k11);
        k25=-2*wz*(VXDecoyReleative(i-1)+0.5*delta_t*k14)-e/(1-e)*(wz^2)*(YDecoyReleative(i-1)+0.5*delta_t*k12);
        k26=-1/(1-e)*(wz^2)*(ZDecoyReleative(i-1)+0.5*delta_t*k13);

        k31=VXDecoyReleative(i-1)+0.5*delta_t*k24;
        k32=VYDecoyReleative(i-1)+0.5*delta_t*k25;
        k33=VZDecoyReleative(i-1)+0.5*delta_t*k26;
        k34=2*wz*(VYDecoyReleative(i-1)+0.5*delta_t*k25)+(3-e)/(1-e)*(wz^2)*(XDecoyReleative(i-1)+0.5*delta_t*k21);
        k35=-2*wz*(VXDecoyReleative(i-1)+0.5*delta_t*k24)-e/(1-e)*(wz^2)*(YDecoyReleative(i-1)+0.5*delta_t*k22);
        k36=-1/(1-e)*(wz^2)*(ZDecoyReleative(i-1)+0.5*delta_t*k23);

        k41=VXDecoyReleative(i-1)+delta_t*k34;
        k42=VYDecoyReleative(i-1)+delta_t*k35;
        k43=VZDecoyReleative(i-1)+delta_t*k36;
        k44=2*wz*(VYDecoyReleative(i-1)+delta_t*k35)+(3-e)/(1-e)*(wz^2)*(XDecoyReleative(i-1)+delta_t*k31);
        k45=-2*wz*(VXDecoyReleative(i-1)+delta_t*k34)-e/(1-e)*(wz^2)*(YDecoyReleative(i-1)+delta_t*k32);
        k46=-1/(1-e)*(wz^2)*(ZDecoyReleative(i-1)+delta_t*k33);

        %当前航迹点的Decoy的相对坐标qzt
        XDecoyReleative(i)=XDecoyReleative(i-1)+delta_t/6*(k11+2*k21+2*k31+k41);
        YDecoyReleative(i)=YDecoyReleative(i-1)+delta_t/6*(k12+2*k22+2*k32+k42);
        ZDecoyReleative(i)=ZDecoyReleative(i-1)+delta_t/6*(k13+2*k23+2*k33+k43);
        VXDecoyReleative(i)=VXDecoyReleative(i-1)+delta_t/6*(k14+2*k24+2*k34+k44);
        VYDecoyReleative(i)=VYDecoyReleative(i-1)+delta_t/6*(k15+2*k25+2*k35+k45);
        VZDecoyReleative(i)=VZDecoyReleative(i-1)+delta_t/6*(k16+2*k26+2*k36+k46);

        %84坐标系——Decoy
        XDecoyS(i)=zh11*XDecoyReleative(i)+zh12*YDecoyReleative(i)+zh13*ZDecoyReleative(i)+coordX_84(i);
        YDecoyS(i)=zh21*XDecoyReleative(i)+zh22*YDecoyReleative(i)+zh23*ZDecoyReleative(i)+coordY_84(i);
        ZDecoyS(i)=zh31*XDecoyReleative(i)+zh32*YDecoyReleative(i)+zh33*ZDecoyReleative(i)+coordZ_84(i);
        VXDecoyS(i)=zh11*VXDecoyReleative(i)+zh12*VYDecoyReleative(i)+zh13*VZDecoyReleative(i)+VX_84(i);
        VYDecoyS(i)=zh21*VXDecoyReleative(i)+zh22*VYDecoyReleative(i)+zh23*VZDecoyReleative(i)+VY_84(i);
        VZDecoyS(i)=zh31*VXDecoyReleative(i)+zh32*VYDecoyReleative(i)+zh33*VZDecoyReleative(i)+VZ_84(i);
        VDecoyS=sqrt(VXDecoyS(i)^2+VYDecoyS(i)^2+VZDecoyS(i)^2);
        thetaXDecoyS(i)=VXDecoyS(i)/VDecoyS;
        thetaYDecoyS(i)=VYDecoyS(i)/VDecoyS;
        thetaZDecoyS(i)=VZDecoyS(i)/VDecoyS;
    end
    XDecoyReleative=[];
	YDecoyReleative=[];
	ZDecoyReleative=[];
	VXDecoyReleative=[];
	VYDecoyReleative=[];
	VZDecoyReleative=[];
end
    
    
        
        