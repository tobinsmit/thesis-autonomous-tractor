function dX = plantWithAB(config,X,U)
    [A,B] = getAB(config.dims,X,U);
    dX = A*X + B*U;
end

