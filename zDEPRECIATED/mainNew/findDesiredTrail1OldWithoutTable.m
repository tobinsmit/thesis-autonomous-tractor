function trail = findPerfectTrail(config, path)
    x = path(:,1)';
    y = path(:,2)';
    x1 = x([1 1:end-1]);
    y1 = y([1 1:end-1]);
    x2 = x([2:end end]);
    y2 = y([2:end end]);
    theta = atan2(y2-y1,x2-x1)+ pi;
    
    xh = x - config.L_h*cos(theta);
    yh = y - config.L_h*sin(theta);
    
    phi = zeros(size(theta));
%     figure(1), clf, hold on
%     plot(theta)
%     h = plot(phi);
    for i = 1:length(path)
        f = @(phi) getImplErr(config, path, xh(i), yh(i), theta(i), phi);
        phi(i) = fminbnd(f,-pi/4,pi/4);
        
        phi_tests = linspace(-pi/4,pi/4,100);
        err = getImplErr(config, path, xh(i), yh(i), theta(i), phi_tests);

%         set(h,'YData',phi);
%         figure(2), clf, hold on
%         plot(phi_tests, err)
%         plot(phi(i),0,'o')
%         drawnow 
%         pause(0.1)
    end
    
    xi = xh - config.L_i*cos(theta + phi);
    yi = yh - config.L_i*sin(theta + phi);
    
    trail(1,:) = x;
    trail(2,:) = y;
    trail(3,:) = theta;
    trail(4,:) = phi;
    trail(5,:) = xi;
    trail(6,:) = yi;
    
end

function err = getImplErr(config, path, xh, yh, theta, phi)
    xi = xh - config.L_i*cos(theta + phi); 
    yi = yh - config.L_i*sin(theta + phi);
    [~,~,err] = findClosestSplinePointMat(path,[xi;yi]);
end




