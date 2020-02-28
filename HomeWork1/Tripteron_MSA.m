close all

F = [100;0;0;0;00];

%Parameters of links
L=300e-3;
l=100e-3;

%Links on Z 
E = 70 *10e9; 
G = 25.5*10e9;
d = 10*10e-3;

S = pi*d^2/4;
Iy = pi*d^4/64;
Iz = pi*d^4/64;
J = Iy + Iz;

% Get angels from IK 
q1 = 0.6;
q2 = 0.3;
q3 = 0.4;



K11 = [E*S/L 0                  0                 0           0                 0;
      0           12*E*Iz/L^3  0                 0           0                 6*E*Iy/L^2;
      0           0                  12*E*Iy/L^3 0           -6*E*Iy/L^2 0;
      0           0                  0                 G*J/L 0                 0;
      0           0                  -6*E*Iy/L^2 0           4*E*Iy/L      0;
      0           6*E*Iy/L^2   0                 0           0                 4*E*Iz/L];
  
  
K12 = [-E*S/L 0                  0                 0           0                 0;
      0           -12*E*Iz/L^3  0                 0           0                 6*E*Iz/L^2;
      0           0                  -12*E*Iy/L^3 0           -6*E*Iy/L^2 0;
      0           0                  0                 -G*J/L 0                 0;
      0           0                  6*E*Iy/L^2 0           2*E*Iy/L      0;
      0           -6*E*Iz/L^2   0                 0           0                 2*E*Iz/L];

K21 = K12';

K22 = [E*S/L 0                  0                 0           0                 0;
      0           12*E*Iz/L^3  0                 0           0                 -6*E*Iy/L^2;
      0           0                  12*E*Iy/L^3 0           6*E*Iy/L^2 0;
      0           0                  0                 G*J/L 0                 0;
      0           0                  6*E*Iy/L^2 0           4*E*Iy/L      0;
      0           -6*E*Iz/L^2   0                 0           0                 4*E*Iz/L];

Qz1 = zeros(6,6);
Qz1(1:3, 1:3) = Rz(q1);
Qz1(4:6, 4:6) = Rz(q1);

KLz = zeros(24, 24);
KLz(1:6, 1:6) = K11;
KLz(1:6, 7:12) = K12;
KLz(7:12, 1:6) = K21;
KLz(7:12, 7:12) = K22;
KLz(13:18, 13:18) = Qz1 * K11 * Qz1';
KLz(13:18, 19:24) = Qz1 * K12 * Qz1';
KLz(19:24, 13:18) = Qz1 * K21 * Qz1';
KLz(19:24, 19:24) = Qz1 * K22 * Qz1';

function rz = Rz(phi)
rz = [
    cos(phi) sin(phi) 0
    -sin(phi) cos(phi) 0
    0 0 1];
end
function rx = Rx(phi)
rx = [1 0 0 
    0 cos(phi) sin(phi) 
    0 -sin(phi) cos(phi)];
end
function ry = Ry(phi)
ry = [1 0 0 
    0 cos(phi) sin(phi) 
    0 -sin(phi) cos(phi)];
end