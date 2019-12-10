%**************************************************************************
%˵�����ó���ʵ�ֹ��Ե�����е���ţ�ʵ���������̬���¡��ٶȸ��º�λ�ø���
%      ��̬�����㷨���ڵ�Ч��תʸ����
%      ���������WGS84����������н���
%      �����������㹫ʽ���ã�
%      ��������ϵѡ��ǰ-��-�£���������ϵѡ�ñ�-��-��
%      δ�������궨�ͳ�ʼ��׼��ʹ���Ѿ������ĳ�ʼ��̬���ٶȺ�λ��
%
%���ߣ�������
%ʱ�䣺2019/7/31
%**************************************************************************
clear;
clc;
fin=fopen('Data1.bin','r');
fout=fopen('Result.bin','wb');
%����һЩ��������
a_WGS= 6378137;                  %���򳤰��� m
b_WGS= 6356752.3142;             %����̰��� m
e_WGS= 0.0818191910428318;       %������
w_e= 7.292115e-5;                %������ת���ٶ� rad/s
GM_WGS= 3.986004418e14;             %������������ m^3/s^2
f_WGS= 1/298.257223563;             %�������

%������ʼ����Ϣ
TimeNow= 91620;
VelocityNow= [0;0;0];       
PositionNow= [23.1373950708*pi/180;113.3713651222*pi/180;2.175];
PoseEANow= [0.0107951084511778*pi/180;-2.14251290749072*pi/180;...
    -75.7498049314083*pi/180];
PoseQNow= EulerAngleToQ(PoseEANow);
PoseDCMNow= EulerAngleToDCM(PoseEANow);
%����������ʼ����Ϣ
I= eye(3);
while(1)                                   
    DataIn=fread(fin, 7, 'double');         %����IMU�������
    if(feof(fin))                           %�ļ�����β��ֹͣѭ��
        break;
    end
    if(DataIn(1,1)<91620)                   %91620������ǰ������
        continue;
    elseif(DataIn(1,1)==91620)              %��91620����ļӱ�������������ݼ�¼����
        AccelNow(1:3,1)= DataIn(5:7,1);
        GyroNow(1:3,1)= DataIn(2:4,1);
    else
        TimeLast= TimeNow;
        VelocityLast= VelocityNow;
        PositionLast= PositionNow;
        PoseEALast= PoseEANow;
        PoseQLast= PoseQNow;
        PoseDCMLast= PoseDCMNow;
        GyroLast= GyroNow;
        AccelLast= AccelNow;
        TimeNow= DataIn(1,1);
        GyroNow(1:3,1)= DataIn(2:4,1);
        AccelNow(1:3,1)= DataIn(5:7,1);
        
        %��ʼ��̬����
        %����bϵ��̬��Ԫ����ÿ��Ԫ��Ҫ���£�
        PhiBody= GyroNow+(1/12)*cross(GyroLast,GyroNow);        %bϵ��תʸ��
        modphi= sqrt(PhiBody(1,1)^2+PhiBody(2,1)^2+PhiBody(3,1)^2);
        QBody(1,1)= cos(0.5*modphi);
        QBody(2:4,1)= (sin(0.5*modphi)/modphi)*PhiBody;         %bϵ�仯��̬��Ԫ��
        
        %����nϵ��̬��Ԫ��
        w_ie=[w_e*cos(PositionLast(1,1));0;-1*w_e*sin(PositionLast(1,1))];
        RM= a_WGS*(1-e_WGS^2)/sqrt((1-e_WGS^2*sin(PositionLast(1,1))^2)^3);
        RN= a_WGS/sqrt(1-e_WGS^2*sin(PositionLast(1,1))^2);
        w_en= [VelocityLast(2,1)/(RN+PositionLast(3,1));...
            -VelocityLast(1,1)/(RM+PositionLast(3,1));...
            -VelocityLast(2,1)*tan(PositionLast(1,1))/(RN+PositionLast(3,1))];

        ZetaNav= (w_en+w_ie)*(TimeNow-TimeLast);
        modzeta= sqrt(ZetaNav(1,1)^2+ZetaNav(2,1)^2+ZetaNav(3,1)^2);
        QNav(1,1)=cos(0.5*modzeta);
        QNav(2:4,1)=(-1*sin(0.5*modzeta)/modzeta)*ZetaNav;
%       QNav(1,1)=1-(1/8)*modzeta^2;  %�򻯰�
%       QNav(2:4,1)=(-1/2)*ZetaNav;


        %���㵱ǰ��Ԫ����̬��Ԫ����ע���һ��
        temp= Qmultiply(QNav, PoseQLast);
        PoseQNow= Qmultiply(temp, QBody);
        modqnow= sqrt(PoseQNow(1,1)^2+PoseQNow(2,1)^2+PoseQNow(3,1)^2+...
            PoseQNow(4,1)^2);
        PoseQNow= PoseQNow./modqnow;
        PoseDCMNow= QToDCM(PoseQNow);
        PoseEANow= DCMToEulerAngle(PoseDCMNow);
        DataOut(8:10,1)=PoseEANow.*(180/pi);            %�洢��ǰ��Ԫ����̬�ǣ��ȣ�
        %������̬����
        
        %��ʼ�ٶȸ���
        %��������/���ϻ�����
        g1= 9.7803267715*(1.0+0.0052790414*sin(PositionLast(1,1))^2+0.0000232718*sin(PositionLast(1,1))^4);
        g2= (-0.000003087691089+0.000000004397731*sin(PositionLast(1,1))^2)*PositionLast(3,1);
        g3= 0.000000000000721*PositionLast(3,1)^2;
        gl= g1+g2+g3;
        glLast=[0;0;gl];
        Vgcork=(glLast-cross((2*w_ie+w_en),VelocityLast))*(TimeNow-TimeLast);
        
        %�������������
        Vfb= AccelNow+(1/2)*cross(GyroNow,AccelNow)+...
            (1/12)*(cross(GyroLast,AccelNow)+cross(AccelLast,GyroNow));
        zeta= (w_en+w_ie)*(TimeNow-TimeLast);
        antizeta=AntisymMatrix(zeta);
        Vfn= (I-(0.5*antizeta))*PoseDCMLast*Vfb;
        
        VelocityNow= VelocityLast+Vfn+Vgcork;
        DataOut(5:7,1)=VelocityNow;                 %�洢��ǰ��Ԫ���ٶ�
        %�����ٶȸ���
        
        %��ʼλ�ø���
        %���¸߳�
        PositionNow(3,1)=PositionLast(3,1)-0.5*(VelocityNow(3,1)+VelocityLast(3,1))...
            *(TimeNow-TimeLast);
        meanh= (PositionNow(3,1)+PositionLast(3,1))/2;
        
        %����γ��
        PositionNow(1,1)=PositionLast(1,1)+0.5*(VelocityNow(1,1)+VelocityLast(1,1))...
            *(TimeNow-TimeLast)/(RM+meanh);
        meanphi= (PositionNow(1,1)+PositionLast(1,1))/2;
        RN= a_WGS/sqrt(1-e_WGS^2*sin(meanphi)^2);
        
        %���¾���
        PositionNow(2,1)= PositionLast(2,1)+0.5*(VelocityNow(2,1)+VelocityLast(2,1))...
            *(TimeNow-TimeLast)/((RN+meanh)*cos(meanphi));
        DataOut(2:3,1)=PositionNow(1:2,1).*(180/pi);
        DataOut(4,1)=PositionNow(3,1);              %�洢��ǰ��Ԫ��λ��
        %����λ�ø���
        
        DataOut(1,1)=TimeNow;
        
        fwrite(fout, DataOut, 'double');

    end
end    

fclose(fin);
fclose(fout);