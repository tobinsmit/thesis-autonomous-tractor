clear
A = zeros(3000,2,6);
B = rand(3000,2,5);
init = [0 1];
A(:,:,2:end) = B;

repmat1loop0 = 1;


t = tic();
for i = 1:1000
    if repmat1loop0
        A(:,:,1) = repmat(init,[3000 1]);
    else
        for j = 1:3000
            A(j,:,1) = init;
        end
    end
end
toc(t)
