clear all
close all
clc
%% NOTE
%dimensione stanza come input e anche robot size
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
    rectangle('Position',[start_point(1)-raggio_disco, start_point(2)-raggio_disco, 2*raggio_disco, 2*raggio_disco],'Curvature',[1 1], 'EdgeColor', 'r', 'FaceColor', 'r');
    % Scelgo la goal pose
    disp('Selezionare la posizione di arrivo ');
    end_point = ginput(1);
    
    % Controllo la posizione corretta
    while(~(end_point(1)+raggio_disco <= stanza_dim(1) && end_point(2)+raggio_disco <=stanza_dim(2) && end_point(1)-raggio_disco>=1 && end_point(2)-raggio_disco>=1))
        disp('Posizione di arrivo non ammissibile. Selezionarne una nuova')
        end_point= ginput(1);
    end
    
    robot_end = end_point;%
    
    % Mostro le posizioni di partenza e arrivo per l'algoritmo di planning
    plot(end_point(:,1),end_point(:,2),'g--o');
    
%% Generazione del diagramma di Voronoi
        updated_map = algoritmo_minkowski(raggio_disco, clearance, stanza);
        updated_map_robot = updated_map(:,:,t,1);
        
        GVD_temp = updated_map_robot;
        updated_map_robot_temp = bwdist(1-GVD_temp);


        [GVD]= rescale(del2(updated_map_robot_temp))<0.5;
        %Remove spurious pixels
        GVD = bwmorph(GVD,'spur');
        GVD = bwmorph(GVD,'thin');
        GVD = bwmorph(GVD,'clean');
        GVD=bwareaopen(GVD,20);
        

       [junction_points, updated_map_robot] = CVD_prima_mappa(GVD, updated_map_robot); % Modifica il nome che CVD non esiste
%        % junctions points sono le coordinate rispetto alla stanza dei
%        % vertici del diagramma di Voronoi 
       [delaunay_triangulation,junction_points,witness_circle_radii,removed_centers] = crea_cerchi_testimoni(junction_points,updated_map_robot);
       
       roadmap = crea_roadmap(GVD, updated_map_robot_temp, updated_map_robot, delaunay_triangulation, junction_points);
       [y,x]=find(~(roadmap==0 | roadmap==1));
       GVD_end   = punto_GVD_vicino(robot_end,x,y);
       GVD_start = punto_GVD_vicino(robot_start,x,y);% trova il punto del GVD più vicino alla posa di start/goal
      
       
       figure(2)
       imshow(GVD)
       hold on
       for i=1:1:size(junction_points,1)
           plot((junction_points(i,1)),(junction_points(i,2)),'*');
       end
%        hold on
%        for i=1:1:size(removed_centers,1)
%            plot((removed_centers(i,1)),(removed_centers(i,2)),'o');
%        end
%        
%             GVD(removed_centers) = 1; % prova a toglierlo
            
       
 figure(3)     
   imshow(GVD)
   hold on
   plot((robot_start(1,1)),(robot_start(1,2)),'d');
   hold on
   plot((robot_end(1,1)),(robot_end(1,2)),'o');
   hold on
   plot((GVD_start(1,1)),(GVD_start(1,2)),'d');
   hold on
    plot((GVD_end(1,1)),(GVD_end(1,2)),'o');
         
         

curr_pos=robot_start;
GVD_start_flags= 0;
GVD_end_flags=0;
goal_complete_flag=0;
%flag for simulation end
end_sim = 0;


if abs((GVD_start(1,1)-curr_pos(1,1)))>0.01
                slope=(GVD_start(1,2)-curr_pos(1,2))/(GVD_start(1,1)-curr_pos(1,1));
                c=curr_pos(1,2)- slope*curr_pos(1,1);
                if(curr_pos(1,1) < GVD_start(1,1))
                    k = 1;
                else
                    k = -1;
                end
                curr_pos(1,:) =  [curr_pos(1,1)+k, slope*(curr_pos(1,1)+k)+c];
            else
                 if(curr_pos(1,1) < GVD_start(1,1))
                    k = 1;
                else
                    k = -1;
                end
                curr_pos(1,:)=[curr_pos(1,1), curr_pos(1,2)+k];
            end
                if pdist([curr_pos(1,:);GVD_start],'euclidean')<0.1
                    GVD_start_flag=1;
                end
                %traverse on the roadmap till the closes point on roadmap
                %to goal is reached
           if GVD_end_flags==0
            [curr_pos(1,:), x, y] = next_point(curr_pos(1,:), x, y, GVD_end);
            if pdist([curr_pos(1,:);GVD_end],'euclidean')<0.1
                GVD_end_flags(1,1)=1;
            end
            
            
            
            
                 %traverse line joining closest point on roadmap to goal, by traversing the line  
        elseif goal_complete_flags(1,1)==0
            if abs((curr_pos(1,1)-end_point(1,1)))>0.01
                slope=(curr_pos(1,2)-end_point(1,2))/(curr_pos(1,1)-end_point(1,1));
                c=curr_pos(1,2)- slope*curr_pos(1,1);
                if(end_point(1,1)<curr_pos(1,1))
                    k=-1;
                else
                    k=1;
                end
                curr_pos(1,:) = [curr_pos(1,1)+k, slope*(curr_pos(1,1)+k)+c];
            else
                if(end_point(1,1)<curr_pos(1,1))
                    k=-1;
                else
                    k=1;
                end
                curr_pos(1,:)=[curr_pos(1,1), curr_pos(1,2)+k];
            end
               %check if the goal has been reached
            if pdist([curr_pos(1,:);end_point],'euclidean')<1
                goal_complete_flags(1,1)=1;
            end

    
    

    
        end
        disp('Robot moving in figure!');
        figure(7)
        imshow(roadmap)
        hold on
        %plot robot positions on roadmap
        plot(curr_pos(1,1), curr_pos(1,2), 'b--O');
        pause(0.1);
        
       
count = 0;

    
        if goal_complete_flag==1
            count=count+1;
        end