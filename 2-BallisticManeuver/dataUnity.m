%Description：将之前的所有文件整合在一起地址：.\Final\datan.mat,储存弹道目标
% WGS84坐标，经纬高坐标，以及雷达直角坐标系坐标
function dataUnity(settings,environment)
numTrajectory=settings.iMisNum;

%% // 读取数据 //
dataWGS84 = cell(1,2);
dataBLH = cell(1,2);
dataRadar = cell(1,2);

str1='.\mid\Missiles_Track\';
ch1 = num2str(numTrajectory);
str2=['warhead',ch1,'111_84.txt'];
str = [str1,str2];

fid = fopen(str,'r');
if (fid == -1)
    error(['Cannot open ',str, '\n']);
end
read_res = fscanf(fid,'%s%s%s%s%s%s%s%s%s%s%s',[1 11]);
Warhead_data = fscanf(fid,'%g',[11,inf]);
fclose(fid);
dataWGS84{1} = Warhead_data';

str1='.\mid\Missiles_Track\';
ch1 = num2str(numTrajectory);
str2=['warheadManeuver',ch1,'111_84.txt'];
str = [str1,str2];

fid = fopen(str,'r');
if (fid == -1)
    error(['Cannot open ',str, '\n']);
end
read_res = fscanf(fid,'%s%s%s%s%s%s%s%s%s%s%s',[1 11]);
Warhead_data = fscanf(fid,'%g',[11,inf]);
fclose(fid);
dataWGS84{2} = Warhead_data';


%% // 得到导弹BLH坐标系数据和雷达坐标系数据 //
for i = 1:2
    length = size(dataWGS84{i},1);
    dataBLH{i} = zeros(length,3);
    for j = 1:length
        coordWGS84=dataWGS84{i}(j,3:5);
        dataBLH{i}(j,:)=WGS842BLH(coordWGS84,environment);
    end
    
    dataRadar{i} = zeros(length,6);
    dataRadarX = S2R(dataWGS84{i}(:,3:5),settings,environment);
    dataRadarv = S2R_2(dataWGS84{i}(:,6:8),settings);
    dataRadar{i}=[dataRadarX',dataRadarv'];
end
%% // save //
filename = ['.\Final\truth',num2str(numTrajectory),'.mat'];
save(filename,'dataWGS84','dataBLH','dataRadar');


