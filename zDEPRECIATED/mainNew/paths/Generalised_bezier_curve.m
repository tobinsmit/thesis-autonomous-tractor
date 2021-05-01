%Hello! this will plot Bezier curve for n control points 
%This is a replacement of the program 'Parametic Cubic Bezier Curve'
%submitted before ...Bezier for any number of points ...enjoy 
clear all 
clc
n=input('Enter no. of points  ');
w=input('Press 1 for entry through mouse or 2 for keyboard repectively-->');
n1=n-1;

if w==1
    axis([0 100 0 100])
    [rawPoints]=ginput(n);
end

if w==2
    [rawPoints]=input('Enter co-odinates of points within brackets ->[x1 y1;x2 y2;x3 y3;...;xn yn] ');
end
    
for i=0:1:n1
    sigma(i+1)=factorial(n1)/(factorial(i)*factorial(n1-i));  % for calculating (x!/(y!(x-y)!)) values 
end
lines=[];
UB=[];

for u=0:0.002:1
    for d=1:n
        UB(d)=sigma(d)*((1-u)^(n-d))*(u^(d-1));
    end
    lines = [lines; UB];
end

P=lines*rawPoints;
line(P(:,1),P(:,2))
line(rawPoints(:,1),rawPoints(:,2))

% books for reference on the subject of cad/cam author Roger Adams  ,
% Ibrahim Zeid 
%Prepared by Mechanical engg. student NIT Allahabad , India
% for any questions feel free to mail me at slnarasimhan89@gmail.com