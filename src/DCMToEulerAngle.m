function [ EA ] = DCMToEulerAngle( Cbn )
%	���������Ҿ���תΪŷ����
%   Cbn��3��3
%   EA��3��1
EA(1,1)= atan2(Cbn(3,2),Cbn(3,3));
EA(2,1)= atan(-1*Cbn(3,1)/sqrt(Cbn(3,2)^2+Cbn(3,3)^2));
EA(3,1)= atan2(Cbn(2,1),Cbn(1,1));
end

