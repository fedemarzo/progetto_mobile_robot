clear all
close all
clc
%% NOTE
%dimensione stanza come input e anche robot size
%% Generazione del workspace e degli ostacoli 

raggio_disco = 50;
clearance = 1;
% CVD_robots = [];
stanza=ones(500,500); % Creo il workspace, inizializzando una matrice di 1 di dimensione 500x500
% hfig=figure(1); % Plot  Workspace
% imshow(stanza,[]); % Mostra the Workspace

numero_ostacoli=input('Inserire il numero di ostacoli: ');
disp('Disegnare i poligoni sulla figura. Doppio click su ogni poligono dopo aver selezionato al ROI per andare avanti: ');

    for i=1:numero_ostacoli
         ostacoli=roipoly(stanza); % seleziono la region of interest ( ROI) per ogni ostacolo
         stanza=stanza-ostacoli;
    end 
    
save('obstacle_space.mat','stanza');
stanza = load('obstacle_space.mat');
stanza = stanza.stanza;
imshow(stanza,[]); % Mostro il nuovo workspace aggiornato

hold on;
%% Definizione della start e goal pose

matrice_check=zeros(500,500);

robot_start = []; % Inizializzate 
robot_end = [];

t=1;% Istante di tempo.

% Seleziono il nodo di partenza del robot
    disp('Selezionare la posizione di partenza: ');
    start_point = ginput(1);
    stanza_dim = size(stanza);
    
    % Controllo sulla posizione corretta 
    while(~(start_point(1)+raggio_disco <= stanza_dim(1) && start_point(2)+raggio_disco <=stanza_dim(2) && start_point(1)-raggio_disco>=1 && start_point(2)-raggio_disco>=1))
        disp('Posizione di partenza non ammissibile. Selezionarne una nuova')
        start_point= ginput(1);
    end
    
    % Aggiorno la posa del robot con i dati di input 
    robot_start =start_point;
    robot_start=round(robot_start);
    
    % Il robot ha forma circolare, di raggio scelto dall'utente.
    rectangle('Position',[start_point(1)-raggio_disco, start_point(2)-raggio_disco, 2*raggio_disco, 2*raggio_disco],'Curvature',[1 1], 'EdgeColor', 'r', 'FaceColor', 'r');
    
    % Scelgo la goal pose
    disp('Selezionare la posizione di arrivo ');
    end_point = ginput(1);
    
    % Controllo la posizione corretta
    while(~(end_point(1)+raggio_disco <= stanza_dim(1) || end_point(2)+raggio_disco <=stanza_dim(2) || end_point(1)-raggio_disco>=1 || end_point(2)-raggio_disco>=1))
        disp('Posizione di arrivo non ammissibile. Selezionarne una nuova')
        end_point= ginput(1);
    end
    
    robot_end = end_point;%
    robot_end=round(robot_end);

    rectangle('Position',[end_point(1)-raggio_disco, end_point(2)-raggio_disco, 2*raggio_disco, 2*raggio_disco],'Curvature',[1 1], 'EdgeColor', 'b', 'FaceColor', 'b');

    
    % Mostro le posizioni di partenza e arrivo per l'algoritmo di planning
    plot(end_point(:,1),end_point(:,2),'g--o');
    
%  for i=robot_start(2)-raggio_disco:1:robot_start(2)+raggio_disco
%     matrice_check(robot_start(1)-raggio_disco:robot_start(1)+raggio_disco,i)=1;
%  end

Z = zeros(500); 
origin = [robot_start(1) robot_start(2)]; %centro della matrice
[xx,yy] = meshgrid((1:size(Z,2))-origin(1),(1:size(Z,1))-origin(2)); % create x and y grid
Z(sqrt(xx.^2 + yy.^2) <= raggio_disco) = 1; % punti interni alla matrice =1
 
    
%% Generazione del diagramma di Voronoi
        updated_map = algoritmo_minkowski(raggio_disco, clearance, stanza);
        updated_map_robot = updated_map(:,:,t,1);
        
        GVD_temp = updated_map_robot;
        updated_map_robot_temp = bwdist(1-GVD_temp);


        [GVD]= rescale(del2(updated_map_robot_temp))<0.5; % con questo comando quando 
        %i valori sono minori della metà del range sono settati a 0, quando
        %maggiori sono settati a 1. 
        %Remove spurious pixels
        GVD = bwmorph(GVD,'spur');
        GVD = bwmorph(GVD,'thin');
        GVD = bwmorph(GVD,'clean');
        GVD=bwareaopen(GVD,20);
        

        
%% Operazioni sul diagramma di Voronoi

[x,y]=find((GVD==1 )); %trovo le coordinate del diagramma
coordinate_diagramma=[x y];

%     for i=1:1:size(coordinate_diagramma,1)
%         for j=size(coordinate_diagramma,2):-1:1
%       distanze_da_start(i)=sqrt((((coordinate_diagramma(i,1))-(robot_start(1)))^2)+(((coordinate_diagramma(j,2))-(robot_start(2)))^2));
%         end
%     end
%       
% [distanza_start_min,indice_start_min]=min(distanze_da_start);
% GVD_start=coordinate_diagramma(indice_start_min,:);
% 
% for i=1:1:size(coordinate_diagramma,1)
%     for j=size(coordinate_diagramma,2):-1:1
%        
%       distanze_da_end(i)=sqrt((((coordinate_diagramma(i,1))-(robot_end(1)))^2)+(((coordinate_diagramma(j,2))-(robot_end(2)))^2));
%     end
% end
% [distanza_end_min,indice_end_min]=min(distanze_da_end);
% GVD_end=coordinate_diagramma(indice_end_min,:);

%% trovo nearest point
%mask = bwareafilt(GVD, 1); % Make sure there is only one blob.
boundaries = bwboundaries(GVD);
boundaries = boundaries{1}; % Extract from cell. Data is [rows, columns], not [x, y]
% Find rows
yRows = boundaries(:, 1);
% Find columns (x)
xColumns = boundaries(:, 2);
% Find distances from (x1, y1) to all other points.
distances = sqrt((xColumns - robot_start(1)).^2 + (yRows - robot_start(2)).^2);
% Find the closest
[minDistance, indexOfMin] = min(distances);
% Find the coordinates
GVD_start(1) = xColumns(indexOfMin);
GVD_start(2) = yRows(indexOfMin);

% Find distances from (x2, y2) to all other points.
distances2 = sqrt((xColumns - robot_end(1)).^2 + (yRows - robot_end(2)).^2);
% Find the closest
[minDistance2, indexOfMin2] = min(distances2);
% Find the coordinates
GVD_end(1) = xColumns(indexOfMin2);
GVD_end(2) = yRows(indexOfMin2);


figure(2)
imshow(GVD)
hold on
plot(GVD_start(1),GVD_start(2),'*','MarkerSize',8)
hold on
plot(GVD_end(1),GVD_end(2),'*','MarkerSize',8)
hold on
plot(robot_start(1),robot_start(2),'d','MarkerSize',8)
hold on
plot(robot_end(1),robot_end(2),'d','MarkerSize',8)


% figure(4)
% GVD_rev = ~GVD;
% map = binaryOccupancyMap(GVD_rev);
% map.LocalOriginInWorld=[0.5 0.5];
% show(map)
% hold on
% plot(robot_start(1,1),robot_start(1,2),'*');
% hold on
% plot(robot_end(1,1),robot_end(1,2),'d');
% hold on
% plot(GVD_start(1,1),GVD_start(1,2),'*');
% hold on
% plot(GVD_end(1,1),GVD_end(1,2),'d');

figure(3)

hold on
plot(GVD_start(1), GVD_start(2), 'g*', 'MarkerSize', 15)
plot(GVD_end(1), GVD_end(2), 'g*', 'MarkerSize', 15)
hold off
D1 = bwdistgeodesic(GVD, GVD_start(1), GVD_start(2), 'quasi-euclidean');
D2 = bwdistgeodesic(GVD, GVD_end(1), GVD_end(2), 'quasi-euclidean');

D = D1 + D2;
D = round(D * 8) / 8;

D(isnan(D)) = inf;
skeleton_path = imregionalmin(D);
P = imoverlay(GVD, imdilate(skeleton_path, ones(3,3)), [1 0 0]);
imshow(P, 'InitialMagnification', 200)
hold on
plot(GVD_start(1), GVD_start(2), 'g*', 'MarkerSize', 15)
plot(GVD_end(1), GVD_end(2), 'g*', 'MarkerSize', 15)
hold off
path_length = D(skeleton_path);
path_length = path_length(1)


     

