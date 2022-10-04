%Date：2021.3.1
%Version：pieces3Sim.m
%Description:生成第三级碎片的轨迹数据
%与debris有不同，第三级碎片起始时间为自由段开始，比debris早,且二级三级弹头没有此过程
function []=pieces3Sim(launchBLA,launchWGS84,settings)
    numTrajectory=settings.iMisNum;
    numPieces3=settings.iNumPieces3;
    
    BT=launchBLA(1); %天文维度
	LT=launchBLA(2); %天文经度
	AT=launchBLA(3); %天文瞄准方位角
    
    str3 = '.\mid\Missiles_Track\PointNumber';
	ch5 = num2str(numTrajectory);
    str4 = [ch5,'.txt'];
    str3A = [str3,str4];
	fp_help1=fopen(str3A,'r');
	if fp_help1==-1
        error(['cannot open ',str3A, '\n']);
    end

    read_fp1 = fscanf(fp_help1,'%d %d %d %d %f %f %f',[1,7]);
 
    num_point1 = read_fp1(1);
    num_point2 = read_fp1(2);
    num_point3 = read_fp1(3);
    numTotal = read_fp1(4);
    delta_t = read_fp1(5);
    fFreeStart = read_fp1(6);
    fFreeEnd = read_fp1(7);
	fclose(fp_help1);

    % 读入warhead的所有坐标
	t1 = zeros(1,num_point2); % Free section时间
	% 位置
	coordX_84 = zeros(1,num_point2);
	coordY_84 = zeros(1,num_point2); 
	coordZ_84 = zeros(1,num_point2); 
	% 速度
	VX_84 = zeros(1,num_point2); 
	VY_84 = zeros(1,num_point2);
	VZ_84 = zeros(1,num_point2); 

    str7 = '.\mid\Missiles_Track\warhead';
    str8 = [num2str(numTrajectory),'111_84','.txt'];
    str7A = [str7,str8];
    
	fp=fopen(str7A,'r');
	if (fp==-1)
        error(['cannot open ',str7A, '\n']);
    end
    
    read_res1 = fscanf(fp, '%s %s %s %s %s %s %s %s %s %s %s', [1 11]);
    read_fp1 = fscanf(fp,' %f %d %f %f %f %f %f %f %f %f %f', [11 inf]);

    seqPiecesFree = [num_point1+1:num_point1+num_point2];
    t1 = read_fp1(1,seqPiecesFree);
    coordX_84 = read_fp1(3,seqPiecesFree);
    coordY_84 = read_fp1(4,seqPiecesFree);
    coordZ_84 = read_fp1(5,seqPiecesFree);
    VX_84 = read_fp1(6,seqPiecesFree);
    VY_84 = read_fp1(7,seqPiecesFree);
    VZ_84 = read_fp1(8,seqPiecesFree);        
    fclose(fp);

    % 2.生成debris相对坐标系的坐标
    wz=0; %warhead绕地心转动角速度
    e=0; %偏心率

	str9 = '.\mid\Missiles_Track\yer_help';
	ch10 = num2str(numTrajectory);
	str10 = [ch10,'.txt'];
    str9A = [str9,str10];

	fp_help3=fopen(str9A,'r');
	if (fp_help3==-1)
        error(['cannot open ',str9A, '\n']);
	end
    read_fp3 = fscanf(fp_help3,'%f %f',[1,2]);
    wz = read_fp3(1);
    e = read_fp3(2);
	fclose(fp_help3);

    XPiecesFreeS=zeros(1, num_point2);
	YPiecesFreeS=zeros(1, num_point2);
	ZPiecesFreeS=zeros(1, num_point2);
	VXPiecesFreeS=zeros(1, num_point2);
	VYPiecesFreeS=zeros(1, num_point2);
	VZPiecesFreeS=zeros(1, num_point2);
	thetaXPiecesFreeS=zeros(1, num_point2);
	thetaYPiecesFreeS=zeros(1, num_point2);
	thetaZPiecesFreeS=zeros(1, num_point2);
    
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
    
    for kkk=1:numPieces3
        [XPiecesFreeS,YPiecesFreeS,ZPiecesFreeS,VXPiecesFreeS,VYPiecesFreeS,VZPiecesFreeS,thetaXPiecesFreeS,thetaYPiecesFreeS,thetaZPiecesFreeS]=...
            decoyDebrisFreeSectionSim(5,d,wz,e,kkk,num_point2,launchWGS84,coordX_84,coordY_84,coordZ_84,VX_84,VY_84,VZ_84,settings);
 
        id=numTrajectory*1000+1*100+5*10+kkk;
		str1='.\mid\Missiles_Track\Pieces';
		ch1=num2str(numTrajectory);
		ch4=num2str(kkk);
		str2=[ch1,'15',ch4,'_84.txt'];
		str1A=[str1,str2];
    

		fp4=fopen(str1A,'w');
		if (fp4==-1)
            error(['cannot open ',str1A, '\n']);
        end

		fprintf(fp4,'%10s','t');
		fprintf(fp4,'%5s','');
		fprintf(fp4,'%10s','idBGTN');
		fprintf(fp4,'%5s','');fprintf(fp4,'%15s','coordX_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','coordY_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','coordZ_84');
		fprintf(fp4,'%5s','');fprintf(fp4,'%15s','VX_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','VY_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','VZ_84');
		fprintf(fp4,'%5s','');fprintf(fp4,'%15s','directX_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','directY_84');fprintf(fp4,'%5s','');fprintf(fp4,'%15s','directZ_84');
		fprintf(fp4,'\n');
        
        for i=1:1:num_point2
            fprintf(fp4,'%10.4f',t1(i));
            fprintf(fp4,'%5s','');
            fprintf(fp4,'%10.4d',id);
            fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',XPiecesFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',YPiecesFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',ZPiecesFreeS(i));
            fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',VXPiecesFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',VYPiecesFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',VZPiecesFreeS(i));
            fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',thetaXPiecesFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',thetaYPiecesFreeS(i));fprintf(fp4,'%5s','');fprintf(fp4,'%15.5f',thetaZPiecesFreeS(i));
            fprintf(fp4,'\n');
        end
        fclose(fp4);
        
    end
        
        
    