function config = initialiseMPCseries(config)

    % Linear spread, max=20, O=2^8
    horizon = 8;
    count = 4;
    max = 20;
    options = linspace(-max,max,count)'; % linear spread
    numOptions = length(options);
    numSeries = numOptions^horizon;
    seqs = zeros(numSeries, config.lenU, horizon);
    
    for s = 1:numSeries
        for o = 1:horizon
            idx = mod(floor((s-1)/(numOptions^(o-1))),numOptions) + 1;
            seqs(s,:,o) = options(idx);
        end
    end

    config.inputSeqs = seqs;
    config.horizon = horizon;
end

