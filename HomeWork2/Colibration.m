close all

% Init parameters
iter_num = 30;
joint_no = 3;
theta_no = 3;
tool = eye(4);

theta = zeros(theta_no, 1);
Kq = 1e6 *[1 2 0.5];
S1 = zeros(3, 3);
S2 = zeros(3, 1);
L1=0.65;
L2=0.35;

tool(1:3,4) = [0;0.1;0.1];

% Iterations through cofigurations 
for i=1:iter_num

    force = zeros(6,1);
    force(1:3) = 2*rand(3,1)-1;    
    force = (force / norm(force)) * 1000;
    
    % Generate q for joints
    q0 = (pi/2)*(2*rand(joint_no, 1) - 1);
    
    % displacement    
    [dt,dq] = delta(q0,theta, force, theta_no, Kq, tool); 
    
    % use only cartesian displacement
    dt = dt(1:3);
    A = get_A(q0, theta, force, theta_no, tool);  
    
    % accumulate sum    
    S1 = S1 + A' * A;
    S2 = S2 + A' * dt;
    
end    

% results
ks = S1 \ S2;          % compliance
stiffness = 1 ./ ks;  % stiffness

% Generate desired points, calculate deflected points and colebrated 
points_amount = 50;

% Generate desired - green
angles = linspace(0, 2*pi, points_amount); 
radius = 20;
xCenter = 10;
yCenter = 20;
zCenter = 30;
x = radius * cos(angles) + xCenter; 
y = radius * sin(angles) + yCenter;
z = radius * cos(angles) + zCenter;
points = [x;y;z;];

% Calculate inverse kinematics q   
q = zeros(3, points_amount);

for i=1:points_amount
   x = points(1, i);
   y = points(2, i);
   z = points(3, i);
   
   q2 = get_Q2(L1, z);
   q3 = get_Q3(x,y,L2);
   q1 = get_Q1(L2, q3, x);
   
   q(1,i) = q1;
   q(2,i) = q2;
   q(3,i) = q3;
end

force_1 = [1000;100;-10;200;-2000;200];

% Points deflected
points_def = points + get_A(q, theta, force_1,3,tool)*(1 ./stiffness);

% Calculate inverse kinematics for points deflected
new_new_q = zeros(3,20);

for i=1:points_amount
   x = points_def(1, i);
   y = points_def(2, i);
   z = points_def(3, i);
   
   q2 = get_Q2(L1, z);
   q3 = get_Q3(x,y,L2);
   q1 = get_Q1(L2, q3, x);
   
   new_new_q(1,i) = q1;
   new_new_q(2,i) = q2;
   new_new_q(3,i) = q3;
end

% Points colibrated
points_col = points_def - get_A(new_new_q, theta, force_1, 3, tool)*(1 ./stiffness);

% Show results
figure(1)
plot3(points(1,:), points(2,:), points(3,:), 'green');
hold on;
plot3(points_def(1,:), points_def(2,:), points_def(3,:), 'red');
hold on;
plot3(points_col(1,:), points_col(2,:), points_col(3,:), 'blue');
legend('desired','deflected','colibrated')
hold off;

% Inverse Kinematics getting angels
function q2 = get_Q2(L1, z)
    q2 = z - L1;
end

function q3 = get_Q3(x,y,L2)
    q3 = sqrt(x^2 + y+2) - L2;
end

function q1 = get_Q1(L2, q3, x)
    q1 = acos(x/(L2 + q3));
end

function a = get_A(q, theta, force, theta_no, tool)
% Matrix of displacement dt = A * pi
   J = theta_jac(q, theta, theta_no, tool);     
   
   % only for joints 
   tmp = zeros(6,3);   
   for i = 1:3
       tmp(:,i) = J(:,i);
   end   
   J = tmp;
   
   %J(3,1) = J(3,2);    
   J = J(1:3,:); % remove angles
   a = J;   
   for i = 1:size(J,2)
       Ji = J(:,i);
       a(:,i) = Ji*Ji'*force(1:3);
   end
end

function [dt,dq] = delta(q,theta, force, theta_no, Kq, tool) 
    % Find end-effector displacement  
    J = theta_jac(q, theta, theta_no, tool);
    
    K = diag(Kq);
   
   % joint displacement
   % J' * force
   dq = K \ (J' * force);
   
   % end-effector displacement
   dt = J * dq;
end

function t = theta_jac(q, theta, theta_no, tool)
   t = zeros(6, theta_no);
   
   L1 = 0.65; 
   L2 = 0.35;
   Tool = tool;
   
   drz = dRz(0);
   dty = dTy(0); 
   dtz = dTz(0);
   
   % 1
   tmp = Rz(q(1)) * drz * Tz(L1) * Tz(q(2)) * Tz(theta(2)) * Ty(L2) * Ty(q(3)) * Ty(theta(3)) * Tool;
   t(:,1) = Jcol(tmp);
   % 2
   tmp = Rz(q(1)) * Rz(theta(1)) * Tz(L1) * Tz(q(2)) * dtz * Ty(L2) * Ty(q(3)) * Ty(theta(3)) * Tool;
   t(:,2) = Jcol(tmp);
   % 3
   tmp = Rz(q(1)) * Rz(theta(1)) * Tz(L1) * Tz(q(2)) * Tz(theta(2)) * Ty(L2) * Ty(q(3)) * dty * Tool;
   t(:,3) = Jcol(tmp);  
   
end

function j = Jcol(m)
   j = [m(1:3,4);m(3,2);m(1,3);m(2,1)];
end

function t = dRz(a)
   t = zeros(4);
   t(1,1) = -sin(a);  t(1,2) = -cos(a);
   t(2,1) = -t(1,2); t(2,2) = t(1,1); 
end

function t = dTy(a)
   t = zeros(4);
   t(2,4) = 1;
end

function t = dTz(a)
   t = zeros(4);
   t(3,4) = 1;
end

function t = Ty(a)
   t = eye(4);
   t(2,4) = a;
end

function t = Tz(a)
   t = eye(4);
   t(3,4) = a;
end

function t = Rz(a)
   t = eye(4);
   t(1,1) = cos(a);  t(1,2) = -sin(a);
   t(2,1) = -t(1,2); t(2,2) = t(1,1); 
end