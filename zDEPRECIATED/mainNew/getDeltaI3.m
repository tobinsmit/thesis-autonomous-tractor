function rts = getDeltaI3(phi,k1,s2,c2,k3)
s = s2;
c = c2;
A = -k1*c;
B = -k1*s - k3;
C = k1*s;
D = B + phi;
X = A*C;
Y = A^2 + B*C;
Z = A*B;
E = Y+A;
F = Z+B;
G = A*X;
H = A*E+D*X;
I = A*F+D*E;
rts = roots([G H I F]);
rts = rts(imag(rts) == 0);
end