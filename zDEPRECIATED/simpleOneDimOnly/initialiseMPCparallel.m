function config = initialiseControllerConstants(config)

    % Cubic spread
%     horizon = 5;
%     count = 11;
%     max = 10;
%     options = linspace(-nthroot(max,3),nthroot(max,3),count)'.^3;
    
    % Linear spread, max=1, O=11^5 
%     horizon = 5;
%     count = 11;
%     max = 10;
%     options = linspace(-max,max,count)'; % linear spread

    % Linear spread, max=10, O=11^5 
%     horizon = 5;
%     count = 11;
%     max = 1;
%     options = linspace(-max,max,count)'; % linear spread

    % Binary test
%     horizon = 5;
%     max = 8;
%     count = 2*log2(max)+1;
%     options = [-2.^(log2(max):-1:0) 0 2.^(0:log2(max))]';
    
    % Linear spread, max=20, O=2^8
    horizon = 8;
    count = 4;
    max = 20;
    options = linspace(-max,max,count)'; % linear spread

    
    config.U_options = options;
    config.horizon = horizon;
end

