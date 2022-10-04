%Description����֮ǰ�������ļ�������һ���ַ��.\Final\datan.mat,���浯��Ŀ��
% WGS84���꣬��γ�����꣬�Լ��״�ֱ������ϵ����
function dataUnity(settings,environment)
numTrajectory=settings.iMisNum;
numSumWarhead=settings.iSumGroup;
numSumHeavyDecoy=settings.iSumHeavyDecoy;
numSumLightDecoy=settings.iSumLightDecoy;
numSumDebris=settings.iSumDebris;
numSumPieces=settings.iNumPieces3;
numSumTarget=numSumWarhead+numSumPieces+sum(numSumHeavyDecoy)+sum(numSumLightDecoy)+sum(numSumDebris);

%% // ��ȡ���� //
dataWGS84 = cell(1,numSumTarget);
dataBLH = cell(1,numSumTarget);
dataRadar = cell(1,numSumTarget);
typeTarget = zeros(1,numSumTarget); %��ʶĿ�����࣬1-warhead,2-HeavyDecoy,3-LightDecoy,4-debrid,5-pieces
k1=1;  %�����ܹ���Ŀ����
for k2 = 1:numSumWarhead %ABCD��BΪGroup��ţ�%����Group
    str1='.\mid\Missiles_Track\';
    ch1 = num2str(numTrajectory);
    ch2 = num2str(k2);
    ch3='1';%ABCD(C=1��ʾ��ͷ)
    ch4='1';%ABCD(DΪ���)
    str2=['warhead',ch1,ch2,ch3,ch4,'_84.txt'];
    str = [str1,str2];
    
    fid = fopen(str,'r');
    if (fid == -1)
        error(['Cannot open ',str, '\n']);
    end
    read_res = fscanf(fid,'%s%s%s%s%s%s%s%s%s%s%s',[1 11]);
    Warhead_data = fscanf(fid,'%g',[11,inf]);
    fclose(fid);
    dataWGS84{k1} = Warhead_data';
    typeTarget(k1) = 1;
    k1 = k1 + 1;
    
    for k3 = 1:numSumHeavyDecoy(k2)%����Heavy Decoy
        str1='.\mid\Missiles_Track\';
        ch1 = num2str(numTrajectory);
        ch2 = num2str(k2);
        ch3='2';%ABCD(C=2��ʾHeavy Decoy)
        ch4 = num2str(k3);%ABCD(DΪ���)
        str2=['HeavyDecoy',ch1,ch2,ch3,ch4,'_84.txt'];
        str = [str1,str2];
        
        fid = fopen(str,'r');
        if (fid == -1)
            error(['Cannot open ',str, '\n']);
        end
        read_res = fscanf(fid,'%s%s%s%s%s%s%s%s%s%s%s',[1 11]);
        HeavyDecoy_data = fscanf(fid,'%g',[11,inf]);
        fclose(fid);
        dataWGS84{k1} = HeavyDecoy_data';
        typeTarget(k1) = 2;
        k1 = k1 + 1;
    end
    
    for k4 = 1:numSumLightDecoy(k2)%����Light Decoy
        str1='.\mid\Missiles_Track\';
        ch1 = num2str(numTrajectory);
        ch2 = num2str(k2);
        ch3='3';%ABCD(C=3��ʾLight Decoy)
        ch4 = num2str(k4);%ABCD(DΪ���)
        str2=['LightDecoy',ch1,ch2,ch3,ch4,'_84.txt'];
        str = [str1,str2];
        
        fid = fopen(str,'r');
        if (fid == -1)
            error(['Cannot open ',str, '\n']);
        end
        read_res = fscanf(fid,'%s%s%s%s%s%s%s%s%s%s%s',[1 11]);
        LightDecoy_data = fscanf(fid,'%g',[11,inf]);
        fclose(fid);
        dataWGS84{k1} = LightDecoy_data';
        typeTarget(k1) = 3;
        k1 = k1 + 1;
    end
    
    for k5 = 1:numSumDebris(k2)%����debris
        str1='.\mid\Missiles_Track\';
        ch1 = num2str(numTrajectory);
        ch2 = num2str(k2);
        ch3 ='4';%ABCD(C=4��ʾDebris)
        ch4 = num2str(k5);%ABCD(DΪ���)
        str2=['Debris',ch1,ch2,ch3,ch4,'_84.txt'];
        str = [str1,str2];
        
        fid = fopen(str,'r');
        if (fid == -1)
            error(['Cannot open ',str, '\n']);
        end
        read_res = fscanf(fid,'%s%s%s%s%s%s%s%s%s%s%s',[1 11]);
        Debris_data = fscanf(fid,'%g',[11,inf]);
        fclose(fid);
        dataWGS84{k1} = Debris_data';
        typeTarget(k1) = 4;
        k1 = k1 + 1;
    end
    
    if k2==1 %����������pieces
        for k6=1:numSumPieces
            str1='.\mid\Missiles_Track\';
            ch1=num2str(numTrajectory);
            ch2=num2str(k2);
            ch3='5';%ABCD(C=5��ʾPieces)
            ch4 = num2str(k6);%ABCD(DΪ���)
            str2=['Pieces',ch1,ch2,ch3,ch4,'_84.txt'];
            str = [str1,str2];
            
            fid = fopen(str, 'r');
            if (fid == -1)
                error(['cannot open ',str, '\n']);
            end
            read_res = fscanf(fid,'%s%s%s%s%s%s%s%s%s%s%s',[1 11]);
            Pieces_data = fscanf(fid,'%g',[11,inf]);
            fclose(fid);
            dataWGS84{k1} = Pieces_data';
            typeTarget(k1) = 5;
            k1 = k1 + 1;
        end
    end
end
%% // �õ�����BLH����ϵ���ݺ��״�����ϵ���� //
for i = 1:numSumTarget
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
save(filename,'dataWGS84','dataBLH','dataRadar','typeTarget');


