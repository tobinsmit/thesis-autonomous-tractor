function config = initialiseMPC(config)

    horizon = 5;
    set = {
        [-1]
        [-30 -5 5 30]
        [0]
    };

    lenU = length(set);
    len1 = length(set{1});
    len2 = length(set{2});
    len3 = length(set{3});
    numOptions = len1 * len2 * len3;
    
    inputOptions = zeros(numOptions, lenU);
    o = 1;
    for i = 1:len1
        for j = 1:len2
            for k = 1:len3
                inputOptions(o,1) = set{1}(i);
                inputOptions(o,2) = set{2}(j);
                inputOptions(o,3) = set{3}(k);
                o = o + 1;
            end
        end
    end
    

    
    config.inputOptions = inputOptions;
    config.horizon = horizon;
end

