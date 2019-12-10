%**************************************************************************
%说明：该程序实现惯性导航机械编排，实现载体的姿态更新、速度更新和位置更新
%      姿态更新算法基于等效旋转矢量法
%      本程序基于WGS84椭球参数进行解算
%      当地重力计算公式采用：
%      载体坐标系选用前-右-下，导航坐标系选用北-东-地
%      未进行误差标定和初始对准，使用已经给定的初始姿态、速度和位置
%
%作者：王雅仪
%时间：2019/7/31
%**************************************************************************
clear;
clc;
fin=fopen('Data1.bin','r');
fout=fopen('Result.bin','wb');
%声明一些基本参数
a_WGS= 6378137;                  %地球长半轴 m
b_WGS= 6356752.3142;             %地球短半轴 m
e_WGS= 0.0818191910428318;       %离心率
w_e= 7.292115e-5;                %地球自转角速度 rad/s
GM_WGS= 3.986004418e14;             %地球引力常数 m^3/s^2
f_WGS= 1/298.257223563;             %地球扁率

%声明初始化信息
TimeNow= 91620;
VelocityNow= [0;0;0];       
PositionNow= [23.1373950708*pi/180;113.3713651222*pi/180;2.175];
PoseEANow= [0.0107951084511778*pi/180;-2.14251290749072*pi/180;...
    -75.7498049314083*pi/180];
PoseQNow= EulerAngleToQ(PoseEANow);
PoseDCMNow= EulerAngleToDCM(PoseEANow);
%结束声明初始化信息
I= eye(3);
while(1)                                   
    DataIn=fread(fin, 7, 'double');         %读入IMU输出数据
    if(feof(fin))                           %文件读到尾，停止循环
        break;
    end
    if(DataIn(1,1)<91620)                   %91620周秒以前不计算
        continue;
    elseif(DataIn(1,1)==91620)              %把91620周秒的加表、陀螺输出的数据记录下来
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
        
        %开始姿态更新
        %更新b系姿态四元数（每历元都要更新）
        PhiBody= GyroNow+(1/12)*cross(GyroLast,GyroNow);        %b系旋转矢量
        modphi= sqrt(PhiBody(1,1)^2+PhiBody(2,1)^2+PhiBody(3,1)^2);
        QBody(1,1)= cos(0.5*modphi);
        QBody(2:4,1)= (sin(0.5*modphi)/modphi)*PhiBody;         %b系变化姿态四元数
        
        %更新n系姿态四元数
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
%       QNav(1,1)=1-(1/8)*modzeta^2;  %简化版
%       QNav(2:4,1)=(-1/2)*ZetaNav;


        %计算当前历元的姿态四元数，注意归一化
        temp= Qmultiply(QNav, PoseQLast);
        PoseQNow= Qmultiply(temp, QBody);
        modqnow= sqrt(PoseQNow(1,1)^2+PoseQNow(2,1)^2+PoseQNow(3,1)^2+...
            PoseQNow(4,1)^2);
        PoseQNow= PoseQNow./modqnow;
        PoseDCMNow= QToDCM(PoseQNow);
        PoseEANow= DCMToEulerAngle(PoseDCMNow);
        DataOut(8:10,1)=PoseEANow.*(180/pi);            %存储当前历元的姿态角（度）
        %结束姿态更新
        
        %开始速度更新
        %计算重力/哥氏积分项
        g1= 9.7803267715*(1.0+0.0052790414*sin(PositionLast(1,1))^2+0.0000232718*sin(PositionLast(1,1))^4);
        g2= (-0.000003087691089+0.000000004397731*sin(PositionLast(1,1))^2)*PositionLast(3,1);
        g3= 0.000000000000721*PositionLast(3,1)^2;
        gl= g1+g2+g3;
        glLast=[0;0;gl];
        Vgcork=(glLast-cross((2*w_ie+w_en),VelocityLast))*(TimeNow-TimeLast);
        
        %计算比力积分项
        Vfb= AccelNow+(1/2)*cross(GyroNow,AccelNow)+...
            (1/12)*(cross(GyroLast,AccelNow)+cross(AccelLast,GyroNow));
        zeta= (w_en+w_ie)*(TimeNow-TimeLast);
        antizeta=AntisymMatrix(zeta);
        Vfn= (I-(0.5*antizeta))*PoseDCMLast*Vfb;
        
        VelocityNow= VelocityLast+Vfn+Vgcork;
        DataOut(5:7,1)=VelocityNow;                 %存储当前历元的速度
        %结束速度更新
        
        %开始位置更新
        %更新高程
        PositionNow(3,1)=PositionLast(3,1)-0.5*(VelocityNow(3,1)+VelocityLast(3,1))...
            *(TimeNow-TimeLast);
        meanh= (PositionNow(3,1)+PositionLast(3,1))/2;
        
        %更新纬度
        PositionNow(1,1)=PositionLast(1,1)+0.5*(VelocityNow(1,1)+VelocityLast(1,1))...
            *(TimeNow-TimeLast)/(RM+meanh);
        meanphi= (PositionNow(1,1)+PositionLast(1,1))/2;
        RN= a_WGS/sqrt(1-e_WGS^2*sin(meanphi)^2);
        
        %更新经度
        PositionNow(2,1)= PositionLast(2,1)+0.5*(VelocityNow(2,1)+VelocityLast(2,1))...
            *(TimeNow-TimeLast)/((RN+meanh)*cos(meanphi));
        DataOut(2:3,1)=PositionNow(1:2,1).*(180/pi);
        DataOut(4,1)=PositionNow(3,1);              %存储当前历元的位置
        %结束位置更新
        
        DataOut(1,1)=TimeNow;
        
        fwrite(fout, DataOut, 'double');

    end
end    

fclose(fin);
fclose(fout);