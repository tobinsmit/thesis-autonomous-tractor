clear

v = -1;
L_t = 2;
L_h = 0.5;
L_i = 1;

ms0 = [1 1 1];
ms1 = [0.836333406488711   0.959793260515307   1.030616988384399];
ms2 = [0.595176377829280   0.376100697426611   0.023561521953149];

msInit = ms2;
getErrFromMs = @(ms) findErr(v,L_t,L_h,L_i,ms);
[bestMs, bestErr] = fminsearch(getErrFromMs, msInit)

function totalError = findErr(v,L_t,L_h,L_i,ms)
    % Sample space is [-pi/4 pi/4]
    delta_ts = linspace(-pi/4,pi/4,100);
    phis = linspace(-pi,pi,100);
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
%         ms(4)
    ]);
end


function phi_dot = calcActPhiDot(v,L_t,L_h,L_i,delta_t,phi,delta_i)
    r1 = L_t / tan(delta_t);
    r2 = sqrt(r1^2+L_h^2);
    r3 = L_i * cos(delta_i) / sin(atan2(L_h,r1) + phi + delta_i);
    theta_t_dot = (v / r1);
    vh = (theta_t_dot) * r2;
    if isnan(vh)
        vh = v;
    end
    theta_i_dot = -(vh / r3);
    phi_dot = theta_i_dot - theta_t_dot;
end