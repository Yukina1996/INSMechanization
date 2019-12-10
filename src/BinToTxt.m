%**************************************************************************
%函数功能：读入bin二进制文件输出txt文本文件
%		 文本格式：精确到小数后9位
%
%作者：王雅仪
%时间：2019/7/31
%**************************************************************************
clear;
clc;
fin= fopen('Result.bin','r');
fout=fopen('Result.txt','wt');	
while(1)
    ins=fread(fin, 10, 'double');
    if(feof(fin))
        break;
    end
    fprintf(fout, '%10.3f\t%12.9f\t%12.9f\t%12.9f\t%12.9f\t%12.9f\t%12.9f\t%12.9f\t%12.9f\t%12.9f\n',...
        ins(1,1),ins(2,1),ins(3,1),ins(4,1),ins(5,1),ins(6,1),ins(7,1),...
        ins(8,1),ins(9,1),ins(10,1));
end
fclose(fin);
fclose(fout);