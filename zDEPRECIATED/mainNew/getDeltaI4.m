function delta_i = getDeltaI4(phi,k1,s2,c2,k3,phi_des,phi_dot_des,phi_dot_dot_des)
    delta_is = deg2rad(linspace(-60,60,1000));
    t = tan(delta_is);

    phi_dot = -k1*(s2+c2*t) - k3;
    phi_dot_dot =  -k1*phi_dot.*(c2-s2*t);

    e = phi * ones(size(delta_is)) - phi_des;
    ed = phi_dot - phi_dot_des;
    edd = phi_dot_dot - phi_dot_dot_des;
    s = e + ed;
    sd = ed + edd;
    ssd = s.*sd;
    
%     isneg = ssd < 0;
%     if ~any(isneg)
%         [~,idx] = min(ssd);
%         delta_i = delta_is(idx);
%     else
%         [~,idx] = min(ssd);
%         delta_i = delta_is(idx);
% %         delta_i = min(abs(delta_is(isneg)));     
%     end

    [~,idx] = min(ssd);
    delta_i = delta_is(idx);

    if isempty(delta_i)
        error("Uh oh")
    end
end