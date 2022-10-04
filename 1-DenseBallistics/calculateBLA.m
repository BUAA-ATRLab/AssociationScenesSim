%Date��2021.1.31
%Version��calculateBLA.m
%Description: �ɷ�������BLH���㷢�������γ��BT,���ľ���LamT�ͷ���㵽����������׼��λ��AT
%��λ������
%��Need to be addressed��AT�ļ���ԭ��
function [coordination] = calculateBLA(launchBLH,landBLH,environment)
    
    latiLaunch=launchBLH(1);%launch pointγ��
    longLaunch=launchBLH(2);%launch point����
    altiLaunch=launchBLH(3);%launch point�߶�
    latiLand=landBLH(1);%landing pointγ��
    longLand=landBLH(2);%landing point����
    altiLand=landBLH(3);%landing point�߶�
    
    BT=latiLaunch*pi/180;
    LamT=longLaunch*pi/180;
    AT=0;%�뱱������н�
    
    launchWGS84=BLH2WGS84(launchBLH,environment);
    landWGS84=BLH2WGS84(landBLH,environment);
    
    aa=landWGS84(1)-launchWGS84(1);
    bb=landWGS84(2)-launchWGS84(2);
    cc=landWGS84(3)-launchWGS84(3);
    
    aaa=-aa*sin(LamT)+bb*cos(LamT);%cos(AT)��ϵ��
    bbb=aa*sin(BT)*cos(LamT)+bb*sin(BT)*sin(LamT)-cc*cos(BT);%sin(AT)��ϵ��

    if aaa==0%���������ھ�����ͬ
        if latiLaunch>latiLand
            AT=pi;
        else
            AT=0;
        end
    elseif bbb==0%γ����ͬ
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
    coordination(3)=AT; % ��λ������
end
    
    
    
    
    
    
    
