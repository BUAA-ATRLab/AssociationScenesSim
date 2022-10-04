%Date：2021.2.1
%Version：calculateMiukV0.m
%Description: 输入被动段航程角,主动段终点预计速度，输出质量比和地面重推比的值

function [miukOPT,v0OPT]=calculateMiukV0(betaPassiveSegment,VActiveSegmentEnd,settings,environment)
    
    miuk=settings.miukInitial;
    v0=settings.v0Initial;
    Pb0=settings.Pb0;
    %arfa=settings.arfa;
    %PM=settings.PM;
    T0=v0*Pb0;  %理想时间的辅助参数
    %g0=environment.g0;
    %Active Segment终点的轨道倾角，单位：度
    %近似认为rk=R,则theta_KOPT=(pi-beta_c)/4
    %theta_k=pi/4-betaPassiveSegment/4;
    
    miuk1=0;%上一次迭代用的miuk
    miuk2=0;%上上一次迭代用的miuk
    v01=0;
    v02=0;
    
    count=0;
    biggerMark=0;%记上次计算得到的Active Segment终点速度大于还是小于要达到的v值，大于为1，小于为0
    boundaryMark=0;
    VV=0;%每次迭代得到的主动段终点速度
    
    while abs(VV-VActiveSegmentEnd)>10 || count==0
        count = count+1;
        
        [~,VV,~,~,~,~]=calculateVActiveEndByMiukV0(miuk,v0,betaPassiveSegment,settings,environment);
       
        if count==1%第一次迭代的情况
            if VV>VActiveSegmentEnd
                biggerMark=1;
                miuk1=miuk;
                miuk=miuk+0.01;%miuk越大，主动段终点速度越小
            else
                biggerMark=0;
                miuk1=miuk;
                miuk=miuk1-0.01;
                if miuk<0.08
                    miuk=0.08;
                end
            end
        else
            if miuk>0.05%如果满足条件可以继续对miuk继续操作
                if boundaryMark==0 %boundaryMark== 0 表示miuk 正在确认上下界
                    if VV>VActiveSegmentEnd
                        if biggerMark==1
                            miuk2=miuk1;
                            miuk1=miuk;
                            miuk=miuk1+0.01;
                        elseif biggerMark==0
                            miuk2=miuk1;
                            miuk1=miuk;
                            miuk=(miuk1+miuk2)/2;
                            boundaryMark=1;
                        end
                        biggerMark=1;
                    elseif VV<VActiveSegmentEnd
                        if biggerMark==1
                            miuk2=miuk1;
                            miuk1=miuk;
                            miuk=(miuk1+miuk2)/2;
                            boundaryMark=1;
                        elseif biggerMark==0
                            miuk2=miuk1;
                            miuk1=miuk;
                            miuk=miuk1-0.01;
                            if miuk<0.05
                                miuk=0.05;
                            end
                        end
                        biggerMark=0;
                    end
                else%miukMark=1表示miuk 取值范围已经圈定
                    if VV>VActiveSegmentEnd
                        if biggerMark==1
                            miuk1=miuk;
                            miuk=(miuk1+miuk2)/2;
                        elseif biggerMark==0
                            miuk2=miuk1;
                            miuk1=miuk;
                            miuk=(miuk1+miuk2)/2;
                        end
                        biggerMark=1;
                    elseif VV<VActiveSegmentEnd
                        if biggerMark==1
                            miuk2=miuk1;
                            miuk1=miuk;
                            miuk=(miuk1+miuk2)/2;
                        elseif biggerMark==0
                            miuk1=miuk;
                            miuk=(miuk1+miuk2)/2;
                        end
                        biggerMark=0;
                    end
                end
            elseif miuk==0.05 && biggerMark==0 && VV>VActiveSegmentEnd
                miuk2=miuk1;
                miuk1=miuk;
                miuk=(miuk1+miuk2)/2;
                biggerMark=1;
            elseif v0>0.4 && v0<0.6 %对v0进行改变,此时miuk已经没有变化的余地
                if boundaryMark==0
                    if VV>VActiveSegmentEnd
                        if biggerMark==1
                            v02=v01;
                            v01=v0;
                            v0=v01+0.01;
                        elseif biggerMark==0
                            v02=v01;
                            v01=v0;
                            v0=(v01+v02)/2;
                            boundaryMark=1;
                        end
                        biggerMark=1;
                    elseif VV<VActiveSegmentEnd
                        if biggerMark==1
                            v02=v01;
                            v01=v0;
                            v0=(v01+v02)/2;
                            boundaryMark=1;
                        elseif biggerMark==0
                            v02=v01;
                            v01=v0;
                            v0=v01-0.01;
                            if v0<0.4	
                                v0=0.4;
                            elseif v0>0.6		
                                v0=0.6;
                            end
                        end
                        biggerMark=0;
                    end
                else
                    if VV>VActiveSegmentEnd
                        if biggerMark==1
                            v01=v0;
                            v0=(v01+v02)/2;						
                        elseif biggerMark==0
                            v02=v01;
                            v01=v0;
                            v0=(v01+v02)/2;					
                        end
                        biggerMark=1;	
                    elseif VV<VActiveSegmentEnd
                        if biggerMark==1
                            v02=v01;
                            v01=v0;
                            v0=(v01+v02)/2;
                        elseif biggerMark==0
                            v01=v0;
                            v0=(v01+v02)/2;
                        end
                        biggerMark=0;	
                    end
                end
            else%调节miuk和v0都达不到达不到那么远的射程
                disp('射程太远，没办法达到');
                break;
            end
        end
        
        V=[];
    end
    
    miukOPT=miuk1;
    v0OPT=v0;
end

    