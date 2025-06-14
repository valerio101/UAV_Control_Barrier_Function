  close all;
%% DRONE SIZE
% Size (WxHxD): 92x92x29mm (motor-to-motor and including motor mount feet)
WW = 0.15; %m
HH = 0.15; %m
DD = 0.05; %m

% Modify to true if you have double system
comparison = false;
%% Impostazioni grafiche                     
figure;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 1]);

% Drone Position New CBF
X_drone = squeeze(out.Position.signals.values(1,:) );
Y_drone = squeeze(out.Position.signals.values(2,:) );
Z_drone = squeeze(out.Position.signals.values(3,:) );

% Attitude New CBF
R = out.Attitude.signals.values;

if comparison
    % Drone Position Old CBF
    X_drone2 = squeeze( out.Position2.signals.values(1,:) );
    Y_drone2 = squeeze( out.Position2.signals.values(2,:) );
    Z_drone2 = squeeze( out.Position2.signals.values(3,:) );
end

% Reference Trajectory
X_ref = squeeze( out.Reference.signals.values(1,:,:) );
Y_ref = squeeze( out.Reference.signals.values(2,:,:) );
Z_ref = squeeze( out.Reference.signals.values(3,:,:) );

% Obstacles' Trajectories
X_obs = squeeze( out.Obstacles.signals.values(:,1,:) );
Y_obs = squeeze( out.Obstacles.signals.values(:,2,:) );
Z_obs = squeeze( out.Obstacles.signals.values(:,3,:) );


hold on;
grid on;

% Imposta la vista 3D
view();

% Generate Points
[x_real, y_real, z_real] = generateEllipsoid([X_drone(1), Y_drone(1), Z_drone(1)], WW,HH,DD);

if comparison
    [x_real2, y_real2, z_real2] = generateSphere([X_drone2(1), Y_drone2(1), Z_drone2(1)], WW);
end

[x_ref, y_ref, z_ref] = generateEllipsoid([X_ref(1), Y_ref(1), Z_ref(1)], WW,HH,DD);

% Generate Surfaces
surface_real = surf(x_real, y_real, z_real, ...
    'FaceColor', [0.9290 0.6940 0.1250], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

if comparison
    surface_real2 = surf(x_real2, y_real2, z_real2, ...
        'FaceColor', [0.2196, 0.9216, 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
end

surface_ref = surf(x_ref, y_ref, z_ref, ...
    'FaceColor', [0 0.5 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% Generate Spheres for each obstacle
Obs_surf = []; 
for i=1:size(X_obs,1)
    [obs_x, obs_y, obs_z] = generateSphere([X_obs(i,1), Y_obs(i,1), Z_obs(i,1)], 0.3);
    %[obs_x, obs_y, obs_z] = generateSphere([9.6017, -3.2942, 30.3], 1);
    Obs_surf(i) = surf(obs_x, obs_y, obs_z, ...
        'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
end

% Trail (tracce) per posizione reale e di riferimento
trail_real = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 3);
trail_ref = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0 0.5 1], 'LineWidth', 3);
%trail_obs = line('XData', [], 'YData', [], 'ZData', [], 'Color', [0.9 0 0], 'LineWidth', 3,'LineStyle',':');
 
%{
% Freccia della velocità
v_vector = quiver3(X_ref(1), Y_ref(1), Z_ref(1), X_dot(1), Y_dot(1), Z_dot(1), ...
                   'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

% Testo della velocità
vel = sqrt(X_dot.^2 + Y_dot.^2 + Z_dot.^2);
v_text = text(X_ref(1), Y_ref(1), Z_ref(1) + 1, num2str(vel(1)), 'FontSize', 14, 'Color', 'k');
%}
axis equal
plot3(X_drone,Y_drone,Z_drone,'c--', 'LineWidth', 2)
if comparison
    plot3(X_drone2,Y_drone2,Z_drone2,'g--', 'LineWidth', 2)
end
plot3(X_ref,Y_ref,Z_ref,'y--', 'LineWidth', 2)
h = hgtransform;
set(surface_real,"Parent",h)

% Plot drone mesh
path2stl = "./quadrotor_2.stl";
drone = generateDroneMesh("./quadrotor_2.stl");

X0_Drone = drone.XData;
Y0_Drone = drone.YData;
Z0_Drone = drone.ZData;

rotoTranslateDrone(drone, X0_Drone,Y0_Drone,Z0_Drone,[X_drone(1) Y_drone(1)  Z_drone(1)], R(:,:,1))

hold on

[x_hat, y_hat, z_hat] = generateRF([X_drone(1) Y_drone(1)  Z_drone(1)],R(:,:,1));

set(x_hat,'Color','red','LineWidth',2) 
set(y_hat,'Color','green','LineWidth',2)
set(z_hat,'Color','blue','LineWidth',2)
hold on
%% Animazione
for k = 2:length(X_drone)
    view();
    angles =  rad2deg(rotm2eul(R(:,:,k)));
    if angles(3) < 0
        angles(3) = angles(3) + 360;
    end
    RR = R(:,:,k);

    % Aggiornamento posizione robot reale
    rotoTranslateEllipsoid(surface_real,WW,HH,DD, [X_drone(k), Y_drone(k), Z_drone(k)], RR)
    rotoTranslateRF(x_hat, y_hat, z_hat,[X_drone(k) Y_drone(k)  Z_drone(k)],R(:,:,k))
    rotoTranslateDrone(drone, X0_Drone,Y0_Drone,Z0_Drone,[X_drone(k) Y_drone(k)  Z_drone(k)], RR)
    
    if comparison
        [x_real2, y_real2, z_real2] = generateSphere([X_drone2(k), Y_drone2(k), Z_drone2(k)], 0.5);
        set(surface_real2, 'XData', x_real2, 'YData', y_real2, 'ZData', z_real2);
    end
    % Aggiornamento posizione robot di riferimento
    rotoTranslateEllipsoid(surface_ref,WW,HH,DD, [X_ref(k), Y_ref(k), Z_ref(k)], eye(3));

    % Aggiornamento posizione ostacoli
    
    for i=1:length(Obs_surf)
        [obs1_x, obs1_y, obs1_z] = generateSphere([X_obs(i,k), Y_obs(i,k), Z_obs(i,k)], 1                      );
        %[obs1_x, obs1_y, obs1_z] = generateSphere([9.6017, -3.2942, 30.3], 1);
        set( Obs_surf(i), 'XData', obs1_x, 'YData', obs1_y, 'ZData', obs1_z);
    end

    % Aggiornamento tracce
    x_trail_real = [get(trail_real, 'XData'), X_drone(k)];
    y_trail_real = [get(trail_real, 'YData'), Y_drone(k)];
    z_trail_real = [get(trail_real, 'ZData'), Z_drone(k)];
    set(trail_real, 'XData', x_trail_real, 'YData', y_trail_real, 'ZData', z_trail_real);
    
    x_trail_ref = [get(trail_ref, 'XData'), X_ref(k)];
    y_trail_ref = [get(trail_ref, 'YData'), Y_ref(k)];
    z_trail_ref = [get(trail_ref, 'ZData'), Z_ref(k)];
    set(trail_ref, 'XData', x_trail_ref, 'YData', y_trail_ref, 'ZData', z_trail_ref);

     %{
    x_trail_obs = [get(trail_obs, 'XData'), Pobs1x(k)];
    y_trail_obs = [get(trail_obs, 'YData'), Pobs1y(k)];
    z_trail_obs = [get(trail_obs, 'ZData'), Pobs1z(k)];
    set(trail_obs, 'XData', x_trail_obs, 'YData', y_trail_obs, 'ZData', z_trail_obs);
    
   
    % Aggiornamento velocità
    set(v_vector, 'XData', X_ref(k), 'YData', Y_ref(k), 'ZData', Z_ref(k), ...
                  'UData', X_dot(k), 'VData', Y_dot(k), 'WData', Z_dot(k));
    
    % Aggiornamento testo della velocità
    cbf = H(k);
    clr = [0 0 0];
    if cbf <= 0
        clr = [1 0 0];
    end
    set(v_text, 'squeeze( out.Position', [X_ref(k), Y_ref(k), Z_ref(k) + 1], 'String', num2str(cbf), 'Color', clr);
    
    
    %}

    %{
    xmin = X_drone(k)-2; xmax = X_drone(k)+2;
    ymin = Y_drone(k)-2; ymax = Y_drone(k)+2;
    zmin = Z_drone(k)-2; zmax = Z_drone(k)+2;

    xlim([xmin, xmax]);
    ylim([ymin, ymax]);
    zlim([zmin, zmax]);
    %}
    
    % Genera il frame
    
        xmin = X_drone(k)-5; xmax = X_drone(k)+5                  ;
        ymin = Y_drone(k)-5; ymax = Y_drone(k)+5;
        zmin = Z_drone(k)-5; zmax = Z_drone(k)+5                ;
        
       
        xlim([xmin, xmax]);
        ylim([ymin, ymax]);
        zlim([zmin, zmax]); 
        
       %{
        xlim([-1, 1]);
        ylim([-1, 1]);
        zlim([-1, 1]); 
       %}
        

    drawnow;
     
   
end

%% Funzione per disegnare una sfera
function [X_ref, Y_ref, Z_ref] = generateSphere(center, radius)
    [sx, sy, sz] = sphere(20);
    X_ref = radius * sx + center(1);
    Y_ref = radius * sy + center(2);
    Z_ref = radius * sz + center(3);
end

%    %    %  % 
function [X_ref, Y_ref, Z_ref] = generateEllipsoid(center, a,b,c)
 
    [X_ref,Y_ref,Z_ref] = ellipsoid(center(1),center(2),center(3),a,b,c);

end

% 
function rotoTranslateEllipsoid(surface_real,a,b,c, center, R)
    [XData,YData,ZData] = ellipsoid(0,0,0,a,b,c);
    [XData,YData,ZData] = applyRotation(XData, YData, ZData, R, center);
    set(surface_real, 'XData', XData, 'YData', YData, 'ZData', ZData); 
end

function [X, Y, Z] = applyRotation(XData, YData, ZData, R, center) 
   X = zeros(size(XData));
   Y = zeros(size(XData));
   Z = zeros(size(XData));
   for i=1:size(XData,1) 
       for j=1:size(XData,2)
       v = [XData(i,j) YData(i,j) ZData(i,j)]';
       v1 = R*v + center';
       X(i,j) = v1(1);
       Y(i,j) = v1(2);
       Z(i,j) = v1(3);
       end
   end

end

function [x_hat, y_hat, z_hat] = generateRF(start, R)

    x_hat = quiver3(start(1),start(2),start(3),R(1,1),R(2,1),R(3,1));
    y_hat = quiver3(start(1),start(2),start(3),R(1,2),R(2,2),R(3,2));
    z_hat = quiver3(start(1),start(2),start(3),R(1,3),R(2,3),R(3,3));
    set(x_hat,'Color','red','LineWidth',2) 
    set(y_hat,'Color','green','LineWidth',2)
    set(z_hat,'Color','blue','LineWidth',2)

end

function rotoTranslateRF(x_hat, y_hat, z_hat, start, R)

    set(x_hat,'XData',start(1),'YData',start(2),'ZData',start(3),'UData',R(1,1),'VData',R(2,1),'WData',R(3,1))
    set(y_hat,'XData',start(1),'YData',start(2),'ZData',start(3),'UData',R(1,2),'VData',R(2,2),'WData',R(3,2))
    set(z_hat,'XData',start(1),'YData',start(2),'ZData',start(3),'UData',R(1,3),'VData',R(2,3),'WData',R(3,3))

end

function drone_mesh = generateDroneMesh(path2stl)
    stl_file = stlread(path2stl);
    geom = fegeometry(stl_file);
    pde = pdegplot(geom,CellLabels="on",FaceAlpha=0.3);
    pp = pde(1);
    drone_mesh = patch("Faces",pp.Faces, "Vertices", pp.Vertices);   
end

function rotoTranslateDrone(mesh, x0,y0,z0, position, R)
    [x0,y0,z0] = applyRotation(x0, y0, z0, R, position      );
    set(mesh, "XData", x0, "YData", y0, "ZData", z0);

end
