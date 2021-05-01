
v = -1;
L_t = 2;
L_h = 0.5;
L_i = 1;

% phi_dot = Cs(1) + Cs(2)*delta_t + Cs(3)*phi + Cs(4)*delta_i

ms0 = [1 1 1];
ms1 = [0.3017             -0.9639             -1.0298];                                              % err = 4.6964e+04
ms2 = [0.3071             -0.9607             -1.0328];                                              % err = 4.7033e+04
ms3 = [0.2500             -1.0000             -1.0000]; % At delta_i = delta_t = 0;
ms4 = [0.301647142956157  -0.963932573640959  -1.029828831798468]; % Optimum everywhere err=4.696397568270913e+04
ms5 = [0.816987298107780  -0.024519052838329  -1.482050807568877]; % Optimum at delta_t=-pi/4, phi=-pi/8, delta_i=-pi/4, err=4.200841087235793e+05 around point
ms6 = [0.250021436402425  -0.040851383488567  -0.999986026290518]; % Optimum around 0. err=0.172646851544345 around 0;
getErrFromMs = @(ms) findErr(v,L_t,L_h,L_i,ms);
getErrFromMs(ms5)
[optimumC, err] = fminsearch(getErrFromMs, ms0)

function totalError = findErr(v,L_t,L_h,L_i,ms)
    % Sample space is [-pi/4 pi/4]
    delta_ts = linspace(-pi/4,pi/4,100);
    phis = linspace(-pi/4,pi/4,100);
    delta_is = linspace(-pi/4,pi/4,100);

    % Sample space is around 0
%     delta_ts = [-0.01 0 0.01];
%     phis = [-0.01 0 0.01];
%     delta_is = [-0.01 0 0.01];

    % Sample space is around a pointt
%     delta_t = -0.785398163397448;
%     phi = -0.523598775598299;
%     delta_i = -0.785398163397448;
%     delta_ts = [delta_t - 0.01, delta_t, delta_t + 0.01];
%     phis = [phi - 0.1, phi, phi + 0.1];
%     delta_is = [delta_i - 0.01, delta_i, delta_i + 0.01];
    
    totalError = 0;
    for t = 1:length(delta_ts)
        delta_t = delta_ts(t);
        for p = 1:length(phis)
            phi = phis(p);
            for i = 1:length(delta_is)
                delta_i = delta_is(i);
                
                estPhiDot = calcEstPhiDot(delta_t,phi,delta_i,ms);
                actPhiDot = calcActPhiDot(v,L_t,L_h,L_i,delta_t,phi,delta_i);
                thisError = abs(actPhiDot - estPhiDot);
                totalError = totalError + thisError;
            end
        end
    end
end

function phi_dot = calcEstPhiDot(delta_t,phi,delta_i,ms)
    phi_dot = sum([
        ms(1)*delta_t
        ms(2)*phi
        ms(3)*delta_i
%         Ks(1)
    ]);
end


function phi_dot = calcActPhiDot(v,L_t,L_h,L_i,delta_t,phi,delta_i)
    % Attempt 1
    r1 = (L_t / tan(delta_t));
    r2 = sqrt(r1^2+L_h^2);
    % alpha1 = atan2(L_h, r1)
    alpha1 = atan(L_h/r1);
    alpha4 = alpha1 + phi + delta_i;
    r3 = L_i * cos(delta_i) / sin(alpha4);
    theta_t_dot = v / r1;
    vh = v*sqrt(1+L_h^2*tan(delta_t)^2/L_t^2);
    theta_i_dot = vh / r3;
    phi_dot = theta_i_dot - theta_t_dot;
    
    % Attempt 2
%     r1 = L_t / tan(delta_t);
%     r2 = sqrt(r1^2+L_h^2);
%     r3 = L_i * cos(delta_i) / sin(atan2(L_h,r1) + phi + delta_i);
%     theta_t_dot = (v / r1);
%     vh = (theta_t_dot) * r2;
%     if isnan(vh)
%         vh = v;
%     end
%     theta_i_dot = -(vh / r3);
%     phi_dot = theta_i_dot - theta_t_dot;

end