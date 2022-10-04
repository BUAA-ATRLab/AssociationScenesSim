%Date��2021.2.1
%Version��calculateMiukV0.m
%Description: ���뱻���κ��̽�,�������յ�Ԥ���ٶȣ���������Ⱥ͵������Ʊȵ�ֵ

function [miukOPT,v0OPT]=calculateMiukV0(betaPassiveSegment,VActiveSegmentEnd,settings,environment)
    
    miuk=settings.miukInitial;
    v0=settings.v0Initial;
    Pb0=settings.Pb0;
    %arfa=settings.arfa;
    %PM=settings.PM;
    T0=v0*Pb0;  %����ʱ��ĸ�������
    %g0=environment.g0;
    %Active Segment�յ�Ĺ����ǣ���λ����
    %������Ϊrk=R,��theta_KOPT=(pi-beta_c)/4
    %theta_k=pi/4-betaPassiveSegment/4;
    
    miuk1=0;%��һ�ε����õ�miuk
    miuk2=0;%����һ�ε����õ�miuk
    v01=0;
    v02=0;
    
    count=0;
    biggerMark=0;%���ϴμ���õ���Active Segment�յ��ٶȴ��ڻ���С��Ҫ�ﵽ��vֵ������Ϊ1��С��Ϊ0
    boundaryMark=0;
    VV=0;%ÿ�ε����õ����������յ��ٶ�
    
    while abs(VV-VActiveSegmentEnd)>10 || count==0
        count = count+1;
        
        [~,VV,~,~,~,~]=calculateVActiveEndByMiukV0(miuk,v0,betaPassiveSegment,settings,environment);
       
        if count==1%��һ�ε��������
            if VV>VActiveSegmentEnd
                biggerMark=1;
                miuk1=miuk;
                miuk=miuk+0.01;%miukԽ���������յ��ٶ�ԽС
            else
                biggerMark=0;
                miuk1=miuk;
                miuk=miuk1-0.01;
                if miuk<0.08
                    miuk=0.08;
                end
            end
        else
            if miuk>0.05%��������������Լ�����miuk��������
                if boundaryMark==0 %boundaryMark== 0 ��ʾmiuk ����ȷ�����½�
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
                else%miukMark=1��ʾmiuk ȡֵ��Χ�Ѿ�Ȧ��
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
            elseif v0>0.4 && v0<0.6 %��v0���иı�,��ʱmiuk�Ѿ�û�б仯�����
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
            else%����miuk��v0���ﲻ���ﲻ����ôԶ�����
                disp('���̫Զ��û�취�ﵽ');
                break;
            end
        end
        
        V=[];
    end
    
    miukOPT=miuk1;
    v0OPT=v0;
end

    