close all

k0 = 1e6;
L1 = 30;
L2 = 30;
%Position
x = 0.2;
y = 0.1;
z = 0.2;

%Inverse Kinematics
q2 = acos((x^2 + y^2 - L1^2 - L2^2)/L1*L2);
q1 = -atan(l2 * cos(q2)/(L1 + L2*sin(q2)))+atan(y/x);

%Link z
T = Tz(1)* Tz(0)*Rz(q1)*Ty(L1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(L2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);

%Find Jacobians for Theta param
T1 = Tz(1)* Tzd()*Rz(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T2 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rxd(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T3 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rx(0)*Ryd(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T4 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rx(0)*Ry(0)*Rzd(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T5 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Txd()*Ty(0)*Tz(0)*Rz(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T6 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Tyd()*Tz(0)*Rz(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T7 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tzd()*Rz(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T8 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(l2)*Rxd(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T9 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(l2)*Rx(0)*Ryd(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T10 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(l2)*Rx(0)*Ry(0)*Rzd(0)*Tx(0)*Ty(0)*Tz(0);
T11 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Txd()*Ty(0)*Tz(0);
T12 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Tyd()*Tz(0);
T13 = Tz(1)* Tz(0)*Rz(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tzd();

J = zeros(6, 13);

J(1:3, 1) = T1(1:3,4);
a = T1/T;
J(4, 1) = a(3,2);
J(5, 1) = a(1,3);
J(6, 1) = a(2,1);

J(1:3, 2) = T2(1:3,4);
a = T2/T;
J(4, 2) = a(3,2);
J(5, 2) = a(1,3);
J(6, 2) = a(2,1);

J(1:3, 3) = T3(1:3,4);
a = T1/T;
J(4, 3) = a(3,2);
J(5, 3) = a(1,3);
J(6, 3) = a(2,1);

J(1:3, 4) = T4(1:3,4);
a = T4/T;
J(4, 4) = a(3,2);
J(5, 4) = a(1,3);
J(6, 4) = a(2,1);

J(1:3, 5) = T5(1:3,4);
a = T5/T;
J(4, 5) = a(3,2);
J(5, 5) = a(1,3);
J(6, 5) = a(2,1);

J(1:3, 6) = T6(1:3,4);
a = T6/T;
J(4, 6) = a(3,2);
J(5, 6) = a(1,3);
J(6, 6) = a(2,1);

J(1:3, 7) = T7(1:3,4);
a = T7/T;
J(4, 7) = a(3,2);
J(5, 7) = a(1,3);
J(6, 7) = a(2,1);

J(1:3, 8) = T8(1:3,4);
a = T8/T;
J(4, 8) = a(3,2);
J(5, 8) = a(1,3);
J(6, 8) = a(2,1);

J(1:3, 9) = T9(1:3,4);
a = T9/T;
J(4, 9) = a(3,2);
J(5, 9) = a(1,3);
J(6, 9) = a(2,1);

J(1:3, 10) = T10(1:3,4);
a = T10/T;
J(4, 10) = a(3,2);
J(5, 10) = a(1,3);
J(6, 10) = a(2,1);

J(1:3, 11) = T11(1:3,4);
a = T11/T;
J(4, 11) = a(3,2);
J(5, 11) = a(1,3);
J(6, 11) = a(2,1);

J(1:3, 12) = T12(1:3,4);
a = T12/T;
J(4, 12) = a(3,2);
J(5, 12) = a(1,3);
J(6, 12) = a(2,1);

J(1:3, 13) = T13(1:3,4);
a = T13/T;
J(4, 13) = a(3,2);
J(5, 13) = a(1,3);
J(6, 13) = a(2,1);

%Find jacobians for q
Tq1 = Tz(1)* Tz(0)*Rzd(q1)*Ty(L1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(L2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
Tq2 = Tz(1)* Tz(0)*Rz(q1)*Ty(L1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rzd(q2)*Ty(L2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);

Jq = zeros(6,2);

Jq(1:3, 1) = Tq1(1:3, 4);
a = Tq1/T;
Jq(4, 1) = a(3,2);
Jq(5, 1) = a(1,3);
Jq(6, 1) = a(2,1);

Jq(1:3, 2) = Tq2(1:3, 4);
a = Tq2/T;
Jq(4, 2) = a(3,2);
Jq(5, 2) = a(1,3);
Jq(6, 2) = a(2,1);

%LINK1
E_1 = 70 *10e9; %Young's modulus
G_1 = 25.5*10e9; %shear modulus
d_1 = 10*10e-3;
L_1 = L1;

%for cylinder
S_1 = pi*d_1^2/4;
Iy_1 = pi*d_1^4/64;
Iz_1 = pi*d_1^4/64;
J_1 = Iy_1 + Iz_1;


k1 = [E_1*S_1/L_1 0                  0                 0           0                 0;
    0           12*E_1*Iz_1/L_1^3  0                 0           0                 6*E_1*Iy_1/L_1^2;
    0           0                  12*E_1*Iy_1/L_1^3 0           -6*E_1*Iy_1/L_1^2 0;
    0           0                  0                 G_1*J_1/L_1 0                 0;
    0           0                  -6*E_1*Iy_1/L_1^2 0           4*E_1*Iy_1/L_1      0;
    0           6*E_1*Iy_1/L_1^2   0                 0           0                 4*E_1*Iz_1/L_1];




%LINK2
E_2 = 70 *10e9; %Young's modulus
G_2 = 25.5*10e9; %shear modulusS
d_2 = 10*10e-3;
L_2 = L2;

%for cylinder
S_2 = pi*d_2^2/4;
Iy_2 = pi*d_2^4/64;
Iz_2 = pi*d_2^4/64;
J_2 = Iy_2 + Iz_2;


k2 = [E_2*S_2/L_2 0                  0                 0           0                 0;
    0           12*E_2*Iz_2/L_2^3  0                 0           0                 6*E_2*Iy_2/L_2^2;
    0           0                  12*E_2*Iy_2/L_2^3 0           -6*E_2*Iy_2/L_2^2 0;
    0           0                  0                 G_2*J_2/L_2 0                 0;
    0           0                  -6*E_2*Iy_2/L_2^2 0           4*E_2*Iy_2/L_2      0;
    0           6*E_2*Iy_2/L_2^2   0                 0           0                 4*E_2*Iz_2/L_2];


Kt = [k0 zeros(1,12)
    zeros(6,1) k1 zeros(6,6)
    zeros(6,1) zeros(6,6) k2];

%Making general stiffness matrix for z
K_stiffness = zeros(21, 21);
K_stiffness(1:6, 7:19) = J;
K_stiffness(1:6, 20:21) = Jq;
K_stiffness(7:19, 1:6) = J';
K_stiffness(20:21, 1:6) = Jq';
K_stiffness(7:19, 7:19) = -Kt;

Kc_Z = zeros(6, 6);
b = inv(K_stiffness);

%Result for z
Kc_Z = b(1:6, 1:6);

%Inverse Kinematics
q2 = acos((z^2 + y^2 - L1^2 - L2^2)/L1*L2);
q1 = -atan(l2 * cos(q2)/(L1 + L2*sin(q2)))+atan(y/z);

%Link x
T = Tx(1)* Tz(0)*Rz(q1)*Ty(L1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rz(q2)*Ty(L2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);

%Find Jacobians for Theta param
T1 = Tx(1)* Txd()*Rx(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rx(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T2 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rxd(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rx(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T3 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rx(0)*Ryd(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rx(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T4 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rx(0)*Ry(0)*Rzd(0)*Tx(0)*Ty(0)*Tz(0)*Rx(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T5 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Txd()*Ty(0)*Tz(0)*Rx(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T6 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Tyd()*Tz(0)*Rx(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T7 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tzd()*Rx(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T8 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rx(q2)*Ty(l2)*Rxd(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T9 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rx(q2)*Ty(l2)*Rx(0)*Ryd(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T10 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rx(q2)*Ty(l2)*Rx(0)*Ry(0)*Rzd(0)*Tx(0)*Ty(0)*Tz(0);
T11 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rx(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Txd()*Ty(0)*Tz(0);
T12 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rx(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Tyd()*Tz(0);
T13 = Tx(1)* Tx(0)*Rx(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rx(q2)*Ty(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tzd();

J = zeros(6, 13);

J(1:3, 1) = T1(1:3,4);
a = T1/T;
J(4, 1) = a(3,2);
J(5, 1) = a(1,3);
J(6, 1) = a(2,1);

J(1:3, 2) = T2(1:3,4);
a = T2/T;
J(4, 2) = a(3,2);
J(5, 2) = a(1,3);
J(6, 2) = a(2,1);

J(1:3, 3) = T3(1:3,4);
a = T1/T;
J(4, 3) = a(3,2);
J(5, 3) = a(1,3);
J(6, 3) = a(2,1);

J(1:3, 4) = T4(1:3,4);
a = T4/T;
J(4, 4) = a(3,2);
J(5, 4) = a(1,3);
J(6, 4) = a(2,1);

J(1:3, 5) = T5(1:3,4);
a = T5/T;
J(4, 5) = a(3,2);
J(5, 5) = a(1,3);
J(6, 5) = a(2,1);

J(1:3, 6) = T6(1:3,4);
a = T6/T;
J(4, 6) = a(3,2);
J(5, 6) = a(1,3);
J(6, 6) = a(2,1);

J(1:3, 7) = T7(1:3,4);
a = T7/T;
J(4, 7) = a(3,2);
J(5, 7) = a(1,3);
J(6, 7) = a(2,1);

J(1:3, 8) = T8(1:3,4);
a = T8/T;
J(4, 8) = a(3,2);
J(5, 8) = a(1,3);
J(6, 8) = a(2,1);

J(1:3, 9) = T9(1:3,4);
a = T9/T;
J(4, 9) = a(3,2);
J(5, 9) = a(1,3);
J(6, 9) = a(2,1);

J(1:3, 10) = T10(1:3,4);
a = T10/T;
J(4, 10) = a(3,2);
J(5, 10) = a(1,3);
J(6, 10) = a(2,1);

J(1:3, 11) = T11(1:3,4);
a = T11/T;
J(4, 11) = a(3,2);
J(5, 11) = a(1,3);
J(6, 11) = a(2,1);

J(1:3, 12) = T12(1:3,4);
a = T12/T;
J(4, 12) = a(3,2);
J(5, 12) = a(1,3);
J(6, 12) = a(2,1);

J(1:3, 13) = T13(1:3,4);
a = T13/T;
J(4, 13) = a(3,2);
J(5, 13) = a(1,3);
J(6, 13) = a(2,1);

%Find jacobians for q
Tq1 = Tx(1)* Tx(0)*Rxd(q1)*Ty(L1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rx(q2)*Ty(L2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
Tq2 = Tx(1)* Tx(0)*Rx(q1)*Ty(L1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Rxd(q2)*Ty(L2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);

Jq = zeros(6,2);

Jq(1:3, 1) = Tq1(1:3, 4);
a = Tq1/T;
Jq(4, 1) = a(3,2);
Jq(5, 1) = a(1,3);
Jq(6, 1) = a(2,1);

Jq(1:3, 2) = Tq2(1:3, 4);
a = Tq2/T;
Jq(4, 2) = a(3,2);
Jq(5, 2) = a(1,3);
Jq(6, 2) = a(2,1);

%LINK1
E_1 = 70 *10e9; %Young's modulus
G_1 = 25.5*10e9; %shear modulus
d_1 = 10*10e-3;
L_1 = L1;

%for cylinder
S_1 = pi*d_1^2/4;
Iy_1 = pi*d_1^4/64;
Iz_1 = pi*d_1^4/64;
J_1 = Iy_1 + Iz_1;


k1 = [E_1*S_1/L_1 0                  0                 0           0                 0;
    0           12*E_1*Iz_1/L_1^3  0                 0           0                 6*E_1*Iy_1/L_1^2;
    0           0                  12*E_1*Iy_1/L_1^3 0           -6*E_1*Iy_1/L_1^2 0;
    0           0                  0                 G_1*J_1/L_1 0                 0;
    0           0                  -6*E_1*Iy_1/L_1^2 0           4*E_1*Iy_1/L_1      0;
    0           6*E_1*Iy_1/L_1^2   0                 0           0                 4*E_1*Iz_1/L_1];




%LINK2
E_2 = 70 *10e9; %Young's modulus
G_2 = 25.5*10e9; %shear modulusS
d_2 = 10*10e-3;
L_2 = L2;

%for cylinder
S_2 = pi*d_2^2/4;
Iy_2 = pi*d_2^4/64;
Iz_2 = pi*d_2^4/64;
J_2 = Iy_2 + Iz_2;


k2 = [E_2*S_2/L_2 0                  0                 0           0                 0;
    0           12*E_2*Iz_2/L_2^3  0                 0           0                 6*E_2*Iy_2/L_2^2;
    0           0                  12*E_2*Iy_2/L_2^3 0           -6*E_2*Iy_2/L_2^2 0;
    0           0                  0                 G_2*J_2/L_2 0                 0;
    0           0                  -6*E_2*Iy_2/L_2^2 0           4*E_2*Iy_2/L_2      0;
    0           6*E_2*Iy_2/L_2^2   0                 0           0                 4*E_2*Iz_2/L_2];


Kt = [k0 zeros(1,12)
    zeros(6,1) k1 zeros(6,6)
    zeros(6,1) zeros(6,6) k2];

%Making general stiffness matrix for x
K_stiffness = zeros(21, 21);
K_stiffness(1:6, 7:19) = J;
K_stiffness(1:6, 20:21) = Jq;
K_stiffness(7:19, 1:6) = J';
K_stiffness(20:21, 1:6) = Jq';
K_stiffness(7:19, 7:19) = -Kt;

Kc_X = zeros(6, 6);
b = inv(K_stiffness);

%Result for x
Kc_X = b(1:6, 1:6);

%Inverse Kinematics 
q2 = acos((z^2 + x^2 - L1^2 - L2^2)/L1*L2);
q1 = -atan(l2 * cos(q2)/(L1 + L2*sin(q2)))+atan(x/z);

%Link z
T = Ty(1)* Ty(0)*Ry(q1)*Tx(L1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(L2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);

%Find Jacobians for Theta param
T1 = Ty(1)* Tyd()*Ry(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T2 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rxd(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T3 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rx(0)*Ryd(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T4 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rx(0)*Ry(0)*Rzd(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T5 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Txd()*Ty(0)*Tz(0)*Ry(q2)*Tx(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T6 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Tyd()*Tz(0)*Ry(q2)*Tx(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T7 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tzd()*Ry(q2)*Tx(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T8 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(l2)*Rxd(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T9 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(l2)*Rx(0)*Ryd(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
T10 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(l2)*Rx(0)*Ry(0)*Rzd(0)*Tx(0)*Ty(0)*Tz(0);
T11 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(l2)*Rx(0)*Ry(0)*Rz(0)*Txd()*Ty(0)*Tz(0);
T12 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Tyd()*Tz(0);
T13 = Ty(1)* Ty(0)*Ry(q1)*Ty(l1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(l2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tzd();

J = zeros(6, 13);

J(1:3, 1) = T1(1:3,4);
a = T1/T;
J(4, 1) = a(3,2);
J(5, 1) = a(1,3);
J(6, 1) = a(2,1);

J(1:3, 2) = T2(1:3,4);
a = T2/T;
J(4, 2) = a(3,2);
J(5, 2) = a(1,3);
J(6, 2) = a(2,1);

J(1:3, 3) = T3(1:3,4);
a = T1/T;
J(4, 3) = a(3,2);
J(5, 3) = a(1,3);
J(6, 3) = a(2,1);

J(1:3, 4) = T4(1:3,4);
a = T4/T;
J(4, 4) = a(3,2);
J(5, 4) = a(1,3);
J(6, 4) = a(2,1);

J(1:3, 5) = T5(1:3,4);
a = T5/T;
J(4, 5) = a(3,2);
J(5, 5) = a(1,3);
J(6, 5) = a(2,1);

J(1:3, 6) = T6(1:3,4);
a = T6/T;
J(4, 6) = a(3,2);
J(5, 6) = a(1,3);
J(6, 6) = a(2,1);

J(1:3, 7) = T7(1:3,4);
a = T7/T;
J(4, 7) = a(3,2);
J(5, 7) = a(1,3);
J(6, 7) = a(2,1);

J(1:3, 8) = T8(1:3,4);
a = T8/T;
J(4, 8) = a(3,2);
J(5, 8) = a(1,3);
J(6, 8) = a(2,1);

J(1:3, 9) = T9(1:3,4);
a = T9/T;
J(4, 9) = a(3,2);
J(5, 9) = a(1,3);
J(6, 9) = a(2,1);

J(1:3, 10) = T10(1:3,4);
a = T10/T;
J(4, 10) = a(3,2);
J(5, 10) = a(1,3);
J(6, 10) = a(2,1);

J(1:3, 11) = T11(1:3,4);
a = T11/T;
J(4, 11) = a(3,2);
J(5, 11) = a(1,3);
J(6, 11) = a(2,1);

J(1:3, 12) = T12(1:3,4);
a = T12/T;
J(4, 12) = a(3,2);
J(5, 12) = a(1,3);
J(6, 12) = a(2,1);

J(1:3, 13) = T13(1:3,4);
a = T13/T;
J(4, 13) = a(3,2);
J(5, 13) = a(1,3);
J(6, 13) = a(2,1);

%Find jacobians for q
Tq1 = Ty(1)* Ty(0)*Ryd(q1)*Tx(L1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ry(q2)*Tx(L2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);
Tq2 = Ty(1)* Ty(0)*Ry(q1)*Tx(L1)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0)*Ryd(q2)*Tx(L2)*Rx(0)*Ry(0)*Rz(0)*Tx(0)*Ty(0)*Tz(0);

Jq = zeros(6,2);

Jq(1:3, 1) = Tq1(1:3, 4);
a = Tq1/T;
Jq(4, 1) = a(3,2);
Jq(5, 1) = a(1,3);
Jq(6, 1) = a(2,1);

Jq(1:3, 2) = Tq2(1:3, 4);
a = Tq2/T;
Jq(4, 2) = a(3,2);
Jq(5, 2) = a(1,3);
Jq(6, 2) = a(2,1);

%LINK1
E_1 = 70 *10e9; %Young's modulus
G_1 = 25.5*10e9; %shear modulus
d_1 = 10*10e-3;
L_1 = L1;

%for cylinder
S_1 = pi*d_1^2/4;
Iy_1 = pi*d_1^4/64;
Iz_1 = pi*d_1^4/64;
J_1 = Iy_1 + Iz_1;


k1 = [E_1*S_1/L_1 0                  0                 0           0                 0;
    0           12*E_1*Iz_1/L_1^3  0                 0           0                 6*E_1*Iy_1/L_1^2;
    0           0                  12*E_1*Iy_1/L_1^3 0           -6*E_1*Iy_1/L_1^2 0;
    0           0                  0                 G_1*J_1/L_1 0                 0;
    0           0                  -6*E_1*Iy_1/L_1^2 0           4*E_1*Iy_1/L_1      0;
    0           6*E_1*Iy_1/L_1^2   0                 0           0                 4*E_1*Iz_1/L_1];




%LINK2
E_2 = 70 *10e9; %Young's modulus
G_2 = 25.5*10e9; %shear modulusS
d_2 = 10*10e-3;
L_2 = L2;

%for cylinder
S_2 = pi*d_2^2/4;
Iy_2 = pi*d_2^4/64;
Iz_2 = pi*d_2^4/64;
J_2 = Iy_2 + Iz_2;


k2 = [E_2*S_2/L_2 0                  0                 0           0                 0;
    0           12*E_2*Iz_2/L_2^3  0                 0           0                 6*E_2*Iy_2/L_2^2;
    0           0                  12*E_2*Iy_2/L_2^3 0           -6*E_2*Iy_2/L_2^2 0;
    0           0                  0                 G_2*J_2/L_2 0                 0;
    0           0                  -6*E_2*Iy_2/L_2^2 0           4*E_2*Iy_2/L_2      0;
    0           6*E_2*Iy_2/L_2^2   0                 0           0                 4*E_2*Iz_2/L_2];


Kt = [k0 zeros(1,12)
    zeros(6,1) k1 zeros(6,6)
    zeros(6,1) zeros(6,6) k2];

%Making general stiffness matrix for x
K_stiffness = zeros(21, 21);
K_stiffness(1:6, 7:19) = J;
K_stiffness(1:6, 20:21) = Jq;
K_stiffness(7:19, 1:6) = J';
K_stiffness(20:21, 1:6) = Jq';
K_stiffness(7:19, 7:19) = -Kt;

Kc_Y = zeros(6, 6);
b = inv(K_stiffness);

%Result for Y
Kc_Y = b(1:6, 1:6);

%Final res stiffness matrix assume that platform like one point we can sum
%all our stiffness matrises

Kc = Kc_X + Kc_Y + Kc_Z;

function rx = Rx(phi)
rx = [1 0 0 0
    0 cos(phi) sin(phi) 0
    0 -sin(phi) cos(phi) 0
    0 0 0 1];
end

function rx = Rxd(phi)
rx = [ 0,         0,         0, 0
 0, -sin(phi),  cos(phi), 0
 0, -cos(phi), -sin(phi), 0
 0,         0,         0, 0];
 
end

function rx = Ry(phi)
rx = [
    cos(phi) 0 sin(phi) 0
    0 1 0 0
    -sin(phi) 0 cos(phi) 0
    0 0 0 1];
end

function rx = Ryd(phi)
rx = [ -sin(phi), 0,  cos(phi), 0
        0, 0,         0, 0
 -cos(phi), 0, -sin(phi), 0
         0, 0,         0, 0];
 
end

function rx = Rz(phi)
rx = [
    cos(phi) sin(phi) 0 0
    -sin(phi) cos(phi) 0 0
    0 0 1 0
    0 0 0 1];
end

function rx = Rzd(phi)
rx = [ -sin(phi),  cos(phi), 0, 0
 -cos(phi), -sin(phi), 0, 0
         0,         0, 0, 0
         0,         0, 0, 0];
 
end

function tx = Tx(x)
tx = [1 0 0 x
    0 1 0 0
    0 0 1 0
    0 0 0 1];
end

function tx = Ty(x)
tx = [1 0 0 0
    0 1 0 x
    0 0 1 0
    0 0 0 1];
end

function tx = Tz(x)
tx = [1 0 0 0
    0 1 0 0
    0 0 1 x
    0 0 0 1];
end
function tx = Txd()
tx = [0 0 0 1
    0 0 0 0
    0 0 0 0
    0 0 0 0];
end
function tx = Tyd()
tx = [0 0 0 0
    0 0 0 1
    0 0 0 0
    0 0 0 0];
end
function tx = Tzd()
tx = [0 0 0 0
    0 0 0 0
    0 0 0 1
    0 0 0 0];
end