options = [1 2 3];
horizon = 3;
plans = zeros(length(options).^3,3);
i = 1;
for a = 1:3
    for b = 1:3
        for c = 1:3
            plans(i,:) = [options(a) options(b) options(c)];
            i = i+1;
        end
    end
end
plans

m = [];
m = addPlans(m,[0 1]);
m = addPlans(m,[1 2]);
m = addPlans(m,[2 3 4])

function new = addPlans(orig,options)
    if isempty(orig)
        new = options';
        return
    end
    new = zeros(0, size(orig,2) + 1);
    for p = 1:size(orig,1)
        for o = 1:length(options)
            new(end+1,:) = [orig(p,:) options(o)];
        end
    end
end