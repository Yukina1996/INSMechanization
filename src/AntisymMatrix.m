function [ AntisymmetricMatrix ] = AntisymMatrix( Matrix )
%   将一个列矩阵转换成它的反对称阵
%   Matrix:3行1列；AntisymmetricMatrix：3行3列
AntisymmetricMatrix(1,1)= 0;
AntisymmetricMatrix(1,2)= -Matrix(3,1);
AntisymmetricMatrix(1,3)= Matrix(2,1);
AntisymmetricMatrix(2,1)= Matrix(3,1);
AntisymmetricMatrix(2,2)= 0;
AntisymmetricMatrix(2,3)= -Matrix(1,1);
AntisymmetricMatrix(3,1)= -Matrix(2,1);
AntisymmetricMatrix(3,2)= Matrix(1,1);
AntisymmetricMatrix(3,3)= 0;
end

