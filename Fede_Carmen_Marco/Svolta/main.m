clear all
close all
clc
%% Generazione del workspace e degli ostacoli 

raggio_disco = 5;
clearance = 2;
CVD_robots = [];
stanza=ones(500,500); % Creo il workspace, inizializzando una matrice di 1 di dimensione 500x500
hfig=figure(1); % Plot  Workspace
imshow(stanza,[]); % Mostra the Workspace

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
    
    % Il robot ha forma circolare, di raggio scelto dall'utente.
    rectangle('Position',[start_point(1)-raggio_disco, start_point(2)-raggio_disco, 2*raggio_disco, 2*raggio_disco], 'EdgeColor', 'r', 'FaceColor', 'r');
    
    % Scelgo la goal pose
    disp('Selezionare la posizione di arrivo ');
    end_point = ginput(1);
    
    % Controllo la posizione corretta
    while(~(end_point(1)+raggio_disco <= stanza_dim(1) && end_point(2)+raggio_disco <=stanza_dim(2) && end_point(1)-raggio_disco>=1 && end_point(2)-raggio_disco>=1))
        disp('Posizione di arrivo non ammissibile. Selezionarne una nuova')
        end_point= ginput(1);
    end
    
    robots_end = end_point;%
    
    % Mostro le posizioni di partenza e arrivo per l'algoritmo di planning
    plot(end_point(:,1),end_point(:,2),'g--o');
    
%% Generazione del diagramma di Voronoi


