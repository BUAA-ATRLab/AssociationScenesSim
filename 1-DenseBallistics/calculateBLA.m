%Date：2021.1.31
%Version：calculateBLA.m
%Description: 由发射点落点BLH计算发射点天文纬度BT,天文经度LamT和发射点到落点的天文瞄准方位角AT
%单位：弧度
%【Need to be addressed】AT的计算原理
function [coordination] = calculateBLA(launchBLH,landBLH,environment)
    
    latiLaunch=launchBLH(1);%launch point纬度
    longLaunch=launchBLH(2);%launch point经度
    altiLaunch=launchBLH(3);%launch point高度
    latiLand=landBLH(1);%landing point纬度
    longLand=landBLH(2);%landing point经度
    altiLand=landBLH(3);%landing point高度
    
    BT=latiLaunch*pi/180;
    LamT=longLaunch*pi/180;
    AT=0;%与北极方向夹角
    
    launchWGS84=BLH2WGS84(launchBLH,environment);
    landWGS84=BLH2WGS84(landBLH,environment);
    
    aa=landWGS84(1)-launchWGS84(1);
    bb=landWGS84(2)-launchWGS84(2);
    cc=landWGS84(3)-launchWGS84(3);
    
    aaa=-aa*sin(LamT)+bb*cos(LamT);%cos(AT)的系数
    bbb=aa*sin(BT)*cos(LamT)+bb*sin(BT)*sin(LamT)-cc*cos(BT);%sin(AT)的系数

    if aaa==0%发射点落点在经度相同
        if latiLaunch>latiLand
            AT=pi;
        else
            AT=0;
        end
    elseif bbb==0%纬度相同
        if longLaunch>=0
            if (longLand<=longLaunch && longLand>=0) || (longLand>(longLaunch-180) && longLand<=0)
                AT=-pi/2;
            else
                AT=pi/2;
            end
        else
            if longLand<=longLaunch || longLand>=(longLaunch+180)
                AT=-pi/2;
            else
                AT=pi/2;
            end
        end
    else
        if longLaunch>=0
            if (longLand<=longLaunch && longLand>=0) || (longLand>(longLaunch-180) && longLand<=0)
                if (-bbb/aaa)>0
                    AT=-pi+asin(sqrt(power(aaa,2)/(power(aaa,2)+power(bbb,2))));
                else
                    AT=-asin(sqrt(power(aaa,2)/(power(aaa,2)+power(bbb,2))));
                end
            else
                if (-bbb/aaa)>0
                    AT=asin(sqrt(power(aaa,2)/(power(aaa,2)+power(bbb,2))));
                else
                    AT=pi-asin(sqrt(power(aaa,2)/(power(aaa,2)+power(bbb,2))));
                end
            end
        else
            if longLand<=longLaunch || longLand>=(longLaunch+180)
                if (-bbb/aaa)>0
                    AT=-pi+asin(sqrt(power(aaa,2)/(power(aaa,2)+power(bbb,2))));
                else
                    AT=-asin(sqrt(power(aaa,2)/(power(aaa,2)+power(bbb,2))));
                end
            else
                if (-bbb/aaa)>0
                    AT=asin(sqrt(power(aaa,2)/(power(aaa,2)+power(bbb,2))));
                else
                    AT=pi-asin(sqrt(power(aaa,2)/(power(aaa,2)+power(bbb,2))));
                end
            end
        end
    end

    coordination(1)=BT;
    coordination(2)=LamT;
    coordination(3)=AT; % 单位：弧度
end
    
    
    
    
    
    
    
