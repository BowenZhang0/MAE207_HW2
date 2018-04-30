x_1=3:0.5:7;
x_11=-7:0.5:-3;
%x1=[x_11,x_1];
y_1=3*sqrt(-(x_1/7).^2+1)+15;
y_11=3*sqrt(-(x_11/7).^2+1)+15;
x_2=7:-0.5:4;
x_22=-4:-0.5:-7;
%x2=[x_22,x_2];
y_2=-3*sqrt(-(x_2/7).^2+1)+15;
y_22=-3*sqrt(-(x_22/7).^2+1)+15;
x3=4:-0.5:-4;
y3=abs(x3/2)-((3*sqrt(33)-7)/112)*x3.^2+sqrt(1-(abs(abs(x3)-2)-1).^2)-3+15;
x4=[-1,-0.75,0.75,1];
y4=9-8*abs(x4)+15;
x5=[-0.75,-0.5,0.5,0.75];
y5=3*abs(x5)+0.75+15;
%x6=[-0.5,0.5];
%y6=2.25+8;
x_7=1:0.5:3;
x_77=-3:0.5:-1;
y_7=1.5-0.5*abs(x_7)-3/7*sqrt(10)*(sqrt(3-x_7.^2+2*abs(x_7))-2)+15;
y_77=1.5-0.5*abs(x_77)-3/7*sqrt(10)*(sqrt(3-x_77.^2+2*abs(x_77))-2)+15;
plot(x_1,y_1)
hold on
plot(x_11,y_11)
hold on
plot(x_2,y_2)
hold on
plot(x_22,y_22)
hold on
plot(x3,y3)
hold on
plot(x4,y4)
hold on
plot(x5,y5)
%hold on
%plot(x6,y6)
hold on
plot(x_7,y_7)
hold on
plot(x_77,y_77)
axis equal
%axes('Xlim', [-10 10], 'Ylim', [-5 5]);
X_total=[x_11,x_77(1,2:length(x_77)),x4(1,2),x5(1,2:4),x_7,x_1(1,2:length(x_1)),x_2(1,2:length(x_2)),x3(1,2:length(x3)),x_22(1,2:length(x_22))];
Y_total=[y_11,y_77(1,2:length(y_77)),y4(1,2),y5(1,2:4),y_7,y_1(1,2:length(y_1)),y_2(1,2:length(y_2)),y3(1,2:length(y3)),y_22(1,2:length(y_22))];
figure
plot(X_total,Y_total)
XY=[X_total;Y_total];
axis equal
fid=fopen('C:\Users\Z1969\Desktop\ODrive-master\tools\test_batman.txt','wt');%写入文件路径
matrix=XY  ;                    %input_matrix为待输出矩阵
[m,n]=size(matrix);
 for i=1:1:m
   for j=1:1:n
      if j==n
        fprintf(fid,'%g\n',matrix(i,j));
     else
       fprintf(fid,'%g\t',matrix(i,j));
      end
   end
end
fclose(fid);