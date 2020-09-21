clear all
close all
clc
%% NOTE
%dimensione stanza come input e anche robot size
%% Generazione del workspace e degli ostacoli 

raggio_disco = 10;
stanza=ones(500,500); % Creo il workspace, inizializzando una matrice di 1 di dimensione 500x500

numero_ostacoli=input('Inserire il numero di ostacoli: ');
disp('Disegnare i poligoni sulla figura. Doppio click su ogni poligono dopo aver selezionato al ROI per andare avanti: ');

    for i=1:numero_ostacoli
         ostacoli=roipoly(stanza); % seleziono la region of interest ( ROI) per ogni ostacolo
         stanza=stanza-ostacoli;
    end 
    
save('obstacle_space.mat','stanza');
stanza = load('obstacle_space.mat');
stanza = stanza.stanza;
stanza_dim = size(stanza);
imshow(stanza,[]); % Mostro il nuovo workspace aggiornato

hold on;


%% Generazione del diagramma di Voronoi

%Applicazione della growing procedure per la costruzione del C Space

        C_space= imdilate(~stanza, strel('disk',raggio_disco));
        C_space=~C_space;
        C_space(1:raggio_disco,:)=0;
        C_space(:,1:raggio_disco)=0;
        C_space(stanza_dim(1)-raggio_disco:stanza_dim(1),:)=0;
        C_space(:,stanza_dim(1)-raggio_disco:stanza_dim(1))=0;
        C_space = bwdist(1-C_space);

% Calcolo del diagramma di Voronoi con le funzioni del2 e rescale. del2
% restituisce una matrice di uguale dimensione di C Space dove ogni
% elemento è ottenuto come differenza dell'elemento di C space meno la
% media dei 4 vicini. 
        [GVD]= rescale(del2(C_space))<0.5; % con questo comando quando 
        %i valori sono minori della metà del range sono settati a 0, quando
        %maggiori sono settati a 1. E' un valore logico, quindi 1 quando è
        %vero e 0 quando è falso
        %Filtraggio del diagramma di Voronoi appena calcolato 
        GVD = bwmorph(GVD,'spur');
        GVD = bwmorph(GVD,'thin');
        GVD = bwmorph(GVD,'clean');
%         GVD=bwareaopen(GVD,20);
   
cc = regionprops(GVD,'Area');
maxarea = max([cc.Area]);
GVD = bwareaopen(GVD,maxarea);

%% Definizione della start pose

matrice_robot=zeros(500,500);

robot_start = []; % Inizializzate 
robot_end = [];

t=1;% Istante di tempo.

bool=true;
risposta_utente=bool;

while(risposta_utente==true)
% Seleziono il nodo di partenza del robot
    disp('Selezionare la posizione di partenza: ');
    start_point = ginput(1);
    robot_start=start_point;
    
   
 % Controllo sulla posizione corretta 
    
    origin = [robot_start(1) robot_start(2)]; %centro della matrice
[xx,yy] = meshgrid((1:size(matrice_robot,2))-origin(1),(1:size(matrice_robot,1))-origin(2)); % create x and y grid
matrice_robot(sqrt(xx.^2 + yy.^2) <= raggio_disco) = 1; % punti interni alla matrice =1
bool=1;
if (start_point(1)+raggio_disco <= stanza_dim(1) && start_point(2)+raggio_disco <=stanza_dim(2) && start_point(1)-raggio_disco>=1 && start_point(2)-raggio_disco>=1)
for i=1:1:size(stanza,1)
for j=1:1:size(stanza,2)
if (matrice_robot(i,j)&(~stanza(i,j)==1))
bool=0;
else
continue
end
end
end
else
    bool=0;
end

if bool==0
% Controllo sulla posizione corretta
while(~(bool==1 && start_point(1)+raggio_disco <= stanza_dim(1) && start_point(2)+raggio_disco <=stanza_dim(2) && start_point(1)-raggio_disco>=1 && start_point(2)-raggio_disco>=1))
disp('Posizione di partenza non ammissibile. Selezionarne una nuova')
start_point= ginput(1);
clear robot_start;
robot_start=start_point;
clear matrice_robot;
matrice_robot = zeros(500);
origin = [robot_start(1) robot_start(2)]; %centro della matrice
[xx,yy] = meshgrid((1:size(matrice_robot,2))-origin(1),(1:size(matrice_robot,1))-origin(2)); % create x and y grid
matrice_robot(sqrt(xx.^2 + yy.^2) <= raggio_disco) = 1; % punti interni alla matrice =1
bool=1;
for i=1:1:size(stanza,1)
for j=1:1:size(stanza,2)
if (matrice_robot(i,j)&(~stanza(i,j)==1))
bool=0;
else
continue
end
end
end
end
end

    % Aggiorno la posa del robot con i dati di input 
    robot_start =start_point;
    robot_start=round(robot_start);
    
    % Il robot ha forma circolare, di raggio scelto dall'utente.
    rectangle('Position',[start_point(1)-raggio_disco, start_point(2)-raggio_disco, 2*raggio_disco, 2*raggio_disco],'Curvature',[1 1], 'EdgeColor', 'r', 'FaceColor', 'r');
  
    
%% Definizione della goal pose   
 % Scelgo la goal pose
    disp('Selezionare la posizione di arrivo ');
    end_point = ginput(1);
    robot_end = end_point;
   matrice_robot=zeros(500,500);
   
    % Controllo sulla posizione corretta 
    
    origin = [robot_end(1) robot_end(2)]; %centro della matrice
[xx,yy] = meshgrid((1:size(matrice_robot,2))-origin(1),(1:size(matrice_robot,1))-origin(2)); % create x and y grid
matrice_robot(sqrt(xx.^2 + yy.^2) <= raggio_disco) = 1; % punti interni alla matrice =1
bool=1;
if (end_point(1)+raggio_disco <= stanza_dim(1) && end_point(2)+raggio_disco <=stanza_dim(2) && end_point(1)-raggio_disco>=1 && end_point(2)-raggio_disco>=1)
for i=1:1:size(stanza,1)
for j=1:1:size(stanza,2)
if (matrice_robot(i,j)&(~stanza(i,j)==1))
bool=0;
else
continue
end
end
end
else
    bool=0;
end

if bool==0
% Controllo sulla posizione corretta
while(~(bool==1 && end_point(1)+raggio_disco <= stanza_dim(1) && end_point(2)+raggio_disco <=stanza_dim(2) && end_point(1)-raggio_disco>=1 && end_point(2)-raggio_disco>=1))
disp('Posizione di arrivo non ammissibile. Selezionarne una nuova')
end_point= ginput(1);
clear robot_end;
robot_end=end_point;
clear matrice_robot;
matrice_robot = zeros(500);
origin = [robot_end(1) robot_end(2)]; %centro della matrice
[xx,yy] = meshgrid((1:size(matrice_robot,2))-origin(1),(1:size(matrice_robot,1))-origin(2)); % create x and y grid
matrice_robot(sqrt(xx.^2 + yy.^2) <= raggio_disco) = 1; % punti interni alla matrice =1
bool=1;
for i=1:1:size(stanza,1)
for j=1:1:size(stanza,2)
if (matrice_robot(i,j)&(~stanza(i,j)==1))
bool=0;
else
continue
end
end
end
end
end
    
    robot_end = end_point;
    robot_end=round(robot_end);

    rectangle('Position',[end_point(1)-raggio_disco, end_point(2)-raggio_disco, 2*raggio_disco, 2*raggio_disco],'Curvature',[1 1], 'EdgeColor', 'b', 'FaceColor', 'b');

    
    % Mostro le posizioni di partenza e arrivo per l'algoritmo di planning
    plot(end_point(:,1),end_point(:,2),'g--o');
       

           
%% Operazioni sul diagramma di Voronoi

%Trovo nearest point 

[yRows,xColumns]=find(GVD==1);
distances = sqrt((xColumns - robot_start(1)).^2 + (yRows - robot_start(2)).^2);
% Trovo il più vicino
[minDistance, indexOfMin] = min(distances);
% .. e le sue coordinate
GVD_start(1) = xColumns(indexOfMin);
GVD_start(2) = yRows(indexOfMin);

distances2 = sqrt((xColumns - robot_end(1)).^2 + (yRows - robot_end(2)).^2);
[minDistance2, indexOfMin2] = min(distances2);
GVD_end(1) = xColumns(indexOfMin2);
GVD_end(2) = yRows(indexOfMin2);

 
D1 = bwdistgeodesic(GVD, GVD_start(1), GVD_start(2), 'quasi-euclidean');
D2 = bwdistgeodesic(GVD, GVD_end(1), GVD_end(2), 'quasi-euclidean');

D = D1 + D2;
D = round(D * 8) / 8;

D(isnan(D)) = inf;
skeleton_path = imregionalmin(D);

[xp,yp]=find(skeleton_path==1);
percorso=[yp,xp];

check1=[GVD_start ; robot_start];
check2=[GVD_end ; robot_end];

if (pdist(check1)>stanza_dim(1)*0.2 || pdist(check2)>stanza_dim(1)*0.2)
    msgbox('Percorso non trovato')
    close (figure(3), figure(2))   
else

P = imoverlay(GVD, imdilate(skeleton_path, ones(3,3)), [1 0 0] );
imshow(P, 'InitialMagnification', 200)
hold on
plot(GVD_start(1), GVD_start(2), 'g*', 'MarkerSize', 15)
plot(GVD_end(1), GVD_end(2), 'g*', 'MarkerSize', 15)
plot(robot_start(1),robot_start(2),'d','MarkerSize',8)
plot(robot_end(1),robot_end(2),'d','MarkerSize',8)
hold off
path_length = D(skeleton_path);
path_length = path_length(1)


figure(2)
imshow(~(~C_space|GVD))

hold on
plot(GVD_start(1),GVD_start(2),'*','MarkerSize',8)

plot(GVD_end(1),GVD_end(2),'*','MarkerSize',8)

plot(robot_start(1),robot_start(2),'d','MarkerSize',10)

plot(robot_end(1),robot_end(2),'d','MarkerSize',10)


plot(percorso(:,1),percorso(:,2),'o','color','r')
hold on
plot([GVD_start(1),robot_start(1)],[GVD_start(2),robot_start(2)],'color','r','LineWidth',6)
plot([GVD_end(1),robot_end(1)],[GVD_end(2),robot_end(2)],'color','r','LineWidth',6)


%% ANIMAZIONE 

% figure (4)
% imshow(P)
% hold on
% for i=1:5:size(percorso,1)
%     plot(percorso(i,1),percorso(i,2),'o','MarkerSize',9),hold all
%     drawnow, pause(0.5)
% end
% set(gca,'XLim',[0 stanza_dim(1)],'YLim',[0 stanza_dim(2)]); grid on
end

% dims = [1 50];
% prompt = {'Si','No'};
% dlgtitle = 'Vuoi inserire altri punti di partenza e arrivo?';
% dati = inputdlg(prompt,dlgtitle,dims);
% Si=dati{1};
% No=dati{2};
% if 
%     risposta_utente==Si
 
answer = questdlg('Vorresti modificare le posizioni di arrivo e partenza del robot?', 'Choice Menu','Si','No','No');
switch answer
case 'Si'
risposta_utente = true;
case 'No'
risposta_utente=false;
end

end
