function [ AntisymmetricMatrix ] = AntisymMatrix( Matrix )
%   ��һ���о���ת�������ķ��Գ���
%   Matrix:3��1�У�AntisymmetricMatrix��3��3��
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

