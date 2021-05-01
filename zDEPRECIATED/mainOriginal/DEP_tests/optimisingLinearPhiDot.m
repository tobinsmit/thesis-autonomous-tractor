v = -1;
L_t = 2;
L_h = 0.5;
L_i = 1;

Cs0 = [1 1 1 1];
Cs1 = [-0.0000, 0.3017, -0.9639, -1.0298]; % err = 4.6964e+04
Cs2 = [ 0.0000, 0.3071, -0.9607, -1.0328]; % err = 4.7033e+04
getErrFromCs = @(Cs) findErr(v,L_t,L_h,L_i,Cs);
fminsearch(getErrFromCs, Cs1)

function totalError = findErr(v,L_t,L_h,L_i,Cs)
%     delta_ts = linspace(-pi/4,pi/4,100);
%     phis = linspace(-pi/4,pi/4,100);
%     delta_is = linspace(-pi/4,pi/4,100);
    delta_ts = [-0.01 0 0.01];
    phis = [-0.01 0 0.01];
    delta_is = [-0.01 0 0.01];
    
    totalError = 0;
    for t = 1:length(delta_ts)
        delta_t = delta_ts(t);
        for p = 1:length(phis)
            phi = phis(p);
            for i = 1:length(delta_is)
                delta_i = delta_is(i);
                
                estPhiDot = calcEstPhiDot(delta_t,phi,delta_i,Cs);
                actPhiDot = calcActPhiDot(v,L_t,L_h,L_i,delta_t,phi,delta_i);
                thisError = abs(actPhiDot - estPhiDot);
                totalError = totalError + thisError;
            end
        end
    end
end

function phi_dot = calcEstPhiDot(delta_t,phi,delta_i,Cs)
    phi_dot = sum([
        Cs(1)
        Cs(2)*delta_t
        Cs(3)*phi
        Cs(4)*delta_i
    ]);
end


function phi_dot = calcActPhiDot(v,L_t,L_h,L_i,delta_t,phi,delta_i)
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
end