t = tic();
x = linspace(-pi,pi,100000);
y = zeros(100000,1);
for i = 1:1000
    % 0.389s
    y = cos(x);

    % 1.8s
%     y = -1 + x - x.^2./2 + x.^4./24;
    
    % 7.1s
%     for j = 1:length(x)
%         xj = x(j);
%         y(j) = -1 + xj - xj.^2./2 + xj.^4./24;
%     end
end
toc(t)