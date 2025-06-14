clear
close all 
clc
global prova i
% Initial state (robot in the origin, with zero velocity)
x0 = [0 0 0 0 0 0 0 0 0 0 0 0]; %(x y z xdot ydot zdot phi theta psi
prova =[0]
% Simulation time vector
tspan=0:0.05:30;
i= 0
% Start simulation
[t, x]=ode45(@(t,x)simulation(t,x),tspan,x0);

% Ottieni le posizioni del drone dalla simulazione
drone_x = x(:, 1);
drone_y = x(:, 2);
drone_z = x(:, 3);

% Posizione dell'ostacolo (fisso)
obstacle_x = 10;
obstacle_y = 0.5;
obstacle_z = 0;

% Creazione del plot 3D
figure; % Apre una nuova figura
plot3(drone_x, drone_y, drone_z, 'b-', 'LineWidth', 1.5); % Traiettoria del drone (linea blu)
hold on; % Mantiene il plot corrente per aggiungere altri elementi
scatter3(obstacle_x, obstacle_y, obstacle_z, 100, 'r*', 'LineWidth', 2); % Posizione dell'ostacolo (stella rossa)
plot3([0, drone_x(end)], [0, drone_y(end)], [0, drone_z(end)], 'k--'); % Linea dal drone all'ostacolo

% Etichette degli assi e titolo
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Traiettoria del Drone e Posizione dell''Ostacolo');
grid on; % Attiva la griglia
axis equal; % Assicura che le proporzioni degli assi siano uguali
legend('Traiettoria Drone', 'Ostacolo', 'Linea Drone-Destinazione'); % Aggiunge una legenda
view(3); % Imposta la vista 3D iniziale

function [dx,prova] =simulation(t,x,phi1,theta1,psi1,phi_dot1,theta_dot1,psi_dot1)
global prova i
yref = [1*t; 0;0];

yref_dot  = [1; 0;0];

yref_dd = [0; 0;0];

yref_ddd = [0; 0;0];

% yref = [20*cos(t*0.1); 20*sin(t*0.1);0];
% 
% yref_dot  = [-20*0.1*sin(t); 20*0.1*cos(0.1*t);0];
% 
% yref_dd = [-20*0.1^2*cos(0.1*t); -20*0.1^2*sin(0.1*t);0];
% 
% yref_ddd = [20*0.1^3*sin(0.1*t); -20*0.1^3*cos(0.1*t);0];

M = diag([1/0.5^2 1/0.5^2 1/0.25^2]);
% if(t >20 && t< 20.5)
%              x(7) = -80;
%              x(8) = 50;
% end

Ob1 = [10; 0.5; 0];
Ob1Dot = [0; 0;0];
Ob1DotDot = [0; 0;0];
prova=[prova,10];
Pi = x(1:3);
Pi_dot = x(4:6);
phi = x(7);
theta = x(8);
psi = x(9);
phi_dot = x(10);
theta_dot = x(11);
psi_dot = x(12);
% phi = phi1(end);
% theta = theta1(end); 
% psi = psi1(end);
% phi_dot = phi_dot1(end);
% theta_dot = theta_dot1(end);
% psi_dot = psi_dot1(end);

R_x = [1 0 0;
       0 cos(phi) -sin(phi);
       0 sin(phi) cos(phi)];

R_y = [cos(theta) 0 sin(theta);
       0 1 0;
       -sin(theta) 0 cos(theta)];

R_z = [cos(psi) -sin(psi) 0;
       sin(psi) cos(psi) 0;
       0 0 1];

R = R_z * R_y * R_x;
S = [0 -psi_dot theta_dot; psi_dot 0 -phi_dot; -theta_dot psi_dot 0];
Rdot = R*S;

uNominal = yref_dd + 30*(yref_dot - Pi_dot) + 100*(yref-Pi);
V = (Pi- Ob1);
Vdot=(Pi_dot -Ob1Dot);
po = V*((V'*V)^(-1))*V';
poPerp = eye(3) - po;

V_inv = V*((V'*V)^(-1));
mu=0.6;
%Definition of CBF
delta =20;
h = V.'*R.'*M*R*V + mu * V.'*M*Vdot -1.5; % V'*mu*uNominal + 2 V' * V_dot - mu V' * Ob1DotDot
hdot = 2*V.'*R.'*M*R*Vdot +V.'*(Rdot.'*M*R+R.'*M*Rdot)*V - mu *V.'*M* Ob1DotDot + mu *V.'*M*uNominal;

if h > delta || hdot > 0
     if(t<11.5)
        u = uNominal;
     else
         u = inv(M) * po * (-2/mu*R.'*M*R*Vdot +(Rdot.'*M*R + R.'*M*Rdot)*V)  + inv(M) * poPerp*uNominal;
     end

else
    u = inv(M) * po * (-2/mu*R.'*M*R*Vdot +(Rdot.'*M*R + R.'*M*Rdot)*V)  + inv(M) * poPerp*uNominal;
    disp(t)
end


phi_dot1 = 100*(atan((yref_dd(1)*sin(psi)-yref_dd(2)*cos(psi)/(yref_dd(3)-9.81)*cos(theta)))-phi) +30*(1/(1+tan(phi)^2)*1/(yref_dd(3)-9.81)*(sin(psi)*(yref_dd(2)*psi_dot+yref_ddd(1))*cos(theta)+(yref_dd(2)*theta_dot*sin(theta)))+cos(psi)*((yref_dd(1)*psi_dot+yref_ddd(2))*cos(theta)+(yref_dd(2)*theta_dot*sin(theta))-tan(phi))-phi_dot);
theta_dot1 =100*(atan((yref_dd(1)*cos(psi)+yref_dd(2)*sin(psi))/(yref_dd(3)-9.81))-theta)+ 30* (1/(1+tan(theta)^2)*1/(yref_dd(1)-9.81)*((yref_ddd(1)+yref_dd(2)*psi_dot)*cos(psi)+(yref_ddd(2)-yref_dd(1)*psi_dot)*sin(psi)-yref_ddd(3)*tan(psi))-theta_dot);
psi_dot1 = 0;


A =[0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0];
B=[0 0 0 0 0 0;
   0 0 0 0 0 0;
   0 0 0 0 0 0;
   1 0 0 0 0 0;
   0 1 0 0 0 0;
   0 0 1 0 0 0
   0 0 0 0 0 0
   0 0 0 0 0 0
   0 0 0 0 0 0
   0 0 0 1 0 0
   0 0 0 0 1 0
   0 0 0 0 0 1];

% A =[0 0 0 1 0 0 0 0 0 ;
%     0 0 0 0 1 0 0 0 0 ;
%     0 0 0 0 0 1 0 0 0 ;
%     0 0 0 0 0 0 0 0 0 ;
%     0 0 0 0 0 0 0 0 0 ;
%     0 0 0 0 0 0 0 0 0 ;
%     0 0 0 0 0 0 0 0 0 ;
%     0 0 0 0 0 0 0 0 0 ;
%     0 0 0 0 0 0 0 0 0];
% B=[0 0 0 0 0 0;
%    0 0 0 0 0 0;
%    0 0 0 0 0 0;
%    1 0 0 0 0 0;
%    0 1 0 0 0 0;
%    0 0 1 0 0 0
%    0 0 0 1 0 0
%    0 0 0 0 1 0
%    0 0 0 0 0 1];

dx = A*x+B*[u;phi_dot1;theta_dot1;psi_dot1];
end