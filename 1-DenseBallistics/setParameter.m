%Version：setParameter.m
%Description: 用于设置界面输入的参数

function result0 = setParameter()

    result0.iMisNum=1;                     %[含义：trajectory编号][单位：1][来源：界面]
    result0.iTypeChos=1;                   %[含义：采用哪种方式设置launch point和landing point（1：经纬高；2：雷达球坐标系）][单位：1][来源：界面]
		
    result0.dLatiLaunch=38.2362;            %[含义：launch point纬度][单位：°][来源：界面]
	result0.dLongLaunch=156.3867;          %[含义：launch point经度][单位：°][来源：界面]
	result0.dAltiLaunch=0;                  %[含义：launch point高度] [单位：m][来源：界面]
	result0.dLatiLand=39.2362;              %[含义：landing point纬度][单位：°][来源：界面]
	result0.dLongLand=115.7546;            %[含义：landing point经度][单位：°][来源：界面]
	result0.dAltiLand=100;                  %[含义：landing point高度][单位：m][来源：界面]
		
	result0.dLatiRadar=39.2362;			   %[含义：雷达位置的纬度][单位：°][来源：界面]+
	result0.dLongRadar=115.7546;              %[含义：雷达位置的经度][单位：°][来源：界面]+
	result0.dAltiRadar=10;                    %[含义：雷达位置的高度][单位：m][来源：界面]+
		
	result0.dAzLaunch=0;                    %[含义：launch point在雷达球坐标系下的方位角][单位：°][来源：界面]
	result0.dRnLaunch=0;                    %[含义：launch point在雷达球坐标系下的距离][单位：m][来源：界面]
	result0.dAlLaunch=0;                    %[含义：launch point在雷达球坐标系下的仰角][单位：°][来源：界面]
	result0.dAzLand=0;                      %[含义：landing point在雷达球坐标系下的方位角][单位：°][来源：界面]	
	result0.dRnLand=0;                      %[含义：landing point在雷达球坐标系下的距离][单位：m][来源：界面]
	result0.dAlLand=0;                      %[含义：landing point在雷达球坐标系下的仰角][单位：°][来源：界面]
		
	result0.iNumPieces3 = 2;                %[含义：第三级破片数目][单位：1][来源：界面]
		
	result0.iSumGroup=1;                     %几个warhead，不超过三个
% 	result0.numSecondLaunchTime=501;            %第二个warhead发射时刻（用Free section的点数表示）（0表示没有第二个warhead）
%     result0.numThirdLaunchTime=601;
    result0.numSecondLaunchTime=0;
    result0.numThirdLaunchTime=0;            %第三个warhead发射时刻（用Free section的点数表示）（0表示没有第三个warhead）

    
    result0.iSumHeavyDecoyFirst  = 2;% 第一个团中Heavy Decoy的个数
    result0.iSumHeavyDecoySecond = 0;% 第二个团中Heavy Decoy的个数
    result0.iSumHeavyDecoyThird  = 0;% 第三个团中Heavy Decoy的个数
        
    result0.iSumLightDecoyFirst  = 2;% 团中Light Decoy的个数【轻Heavy Decoy加起来不超过四】
    result0.iSumLightDecoySecond = 0;
    result0.iSumLightDecoyThird  = 0;
        
    result0.iSumDebrisFirst      = 2;  % 团中debris个数，每个团不超过五个debris
    result0.iSumDebrisSecond     = 0;
    result0.iSumDebrisThird      = 0;
        
    result0.iSumHeavyDecoy=[result0.iSumHeavyDecoyFirst,result0.iSumHeavyDecoySecond,result0.iSumHeavyDecoyThird];%团中Heavy Decoy的个数
    result0.iSumLightDecoy=[result0.iSumLightDecoyFirst ,result0.iSumLightDecoySecond,result0.iSumLightDecoyThird];%团中Light Decoy的个数
	result0.iSumDebris=[result0.iSumDebrisFirst,result0.iSumDebrisSecond,result0.iSumDebrisThird];        %团中debris个数
    
    %雷达散射截面(Radar Cross section,缩写RCS)
    result0.dDDRCS=0.1;
    result0.dHeavyDecoyRCS=0.05;
    result0.dLightDecoyRCS=0.03;
    result0.dDebrisRCS=0.01;
    result0.dPiecesRCS=0;
    
    result0.miukInitial=0.15;           %质量比迭代初值
    result0.v0Initial=0.577;            %地面重推比迭代初值
    result0.Pb0=240;                    %地面比推力
	result0.arfa=288/result0.Pb0;       %发动机高空特性系数
	result0.PM=10000;                   %起飞截面载荷
   
    result0.delta_t=0.01;               %仿真时间步长
    
    result0.mWarhead=500;               %warhead质量，单位，kg
    result0.mWarhead2=500;              %二级warhead质量，单位，kg
    result0.mHeavyDecoy=450;            %Heavy Decoy质量，单位：kg
    result0.S_m=3;                      %飞行器特性面积,单位：平方米

    result0.releaseTime = 5;              %Decoy释放时刻（Free section开始后的多少秒）单位：s
    
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
    
    result0.VOffsetDebrisLower=-100;%Debris偏移速度下界
    result0.VOffsetDebrisUpper=100;%Debris偏移速度上界
    result0.VOffsetPiecesLower=-100;%Pieces偏移速度下界
    result0.VOffsetPiecesUpper=100;%Pieces偏移速度上界
    
    result0.XOffsetSecondWarheadB=0.0;
    result0.YOffsetSecondWarheadB=2.0e4;
    result0.ZOffsetSecondWarheadB=0.0;
    result0.XOffsetThirdWarheadB=0.0;
    result0.YOffsetThirdWarheadB=1.0e4;
    result0.ZOffsetThirdWarheadB=0.0;
    
    %设置量测误差
    result0.radar_noise_r = 10;                      %雷达测距离误差，单位：m
    result0.radar_noise_theta = 0.2 *pi/180;         %雷达测俯仰角误差，单位：弧度
    result0.radar_noise_phi = 0.2 *pi/180;           %雷达测方位角误差，单位：弧度

    result0.telescope_noise_theta = 20/3600 *pi/180;       %光学望远镜测俯仰角误差，单位：弧度,20角秒
    result0.telescope_noise_phi = 20/3600 *pi/180;         %光学望远镜测方位角误差，单位：弧度,20角秒
    
end
    
    