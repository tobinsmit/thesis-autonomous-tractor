clear 
syms L_t L_h L_i
syms x y theta_t phi
syms v delta_t delta_i

x_dot = v*cos(theta_t);
y_dot = v*sin(theta_t);

r1 = L_t / tan(delta_t);
r2 = sqrt((r1*tan(delta_t))^2+(L_h*tan(delta_t))^2) / tan(delta_t);
% alpha1 = atan2(L_h, r1);
alpha1 = atan(L_h/r1);
alpha2 = pi/2 - alpha1 - phi;
alpha3 = pi/2 - delta_i;
alpha4 = pi - alpha2 - alpha3;
r3 = L_i * sin(alpha3) / sin(alpha4);
theta_t_dot = v / r1;
vh = theta_t_dot * r2;
if isnan(vh)
    vh = v;
end
theta_i_dot = vh / r3;
phi_dot = theta_i_dot - theta_t_dot;

dxDir = [
    x_dot
    y_dot
    theta_t_dot
    phi_dot
]

save('dxDir.mat','dxDir')

