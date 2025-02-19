clc; clear; close all;
syms theta1m theta2m theta3m

DH_params = [ 0  -sym(pi)/2  0.254  theta1m;
             0.254   0      0  theta2m-sym(pi)/2;
             0.254   0      0  -theta2m+theta3m+sym(pi)/2];

T = @(a, alpha, d, theta) [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                           sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                           0,           sin(alpha),            cos(alpha),           d;
                           0,           0,                     0,                    1];

T01 = T(DH_params(1,1), DH_params(1,2), DH_params(1,3), DH_params(1,4));
T12 = T(DH_params(2,1), DH_params(2,2), DH_params(2,3), DH_params(2,4));
T23 = T(DH_params(3,1), DH_params(3,2), DH_params(3,3), DH_params(3,4));

T02 = T01 * T12;
T03 = (T02 * T23);


disp('T01 = '); disp(T01);
disp('T02 = '); disp(T02);
disp('T03 = '); disp(T03);

T03_vpa = vpa(simplify(T03));
disp('T03 (numeric) = '); disp(T03_vpa);
