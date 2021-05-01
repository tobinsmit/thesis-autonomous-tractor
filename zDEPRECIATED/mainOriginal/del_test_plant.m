clear

config.Lt = 2;
config.Lh = 0.75;
config.Li = 1;
config.dims = [2;0.75;1];

x = 0;
y = 0;
theta_t = pi;
phi = 0;

v = 1;
delta_t = 0;
delta_i = 0;

X = [x
    y
    theta_t
    phi];

U = [v
    delta_t
    delta_i];

dX1 = plantRad(config, X, U)

dX2 = plantRadWithABSym(config, X, U)
