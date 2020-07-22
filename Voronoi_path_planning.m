clear all
clc
tic
%% Ambiente
%Crea una mappa di dimensione e risoluzione scelta dall'utente.
dims = [1 50];
prompt = {'Lunghezza mappa x [m]','Lunghezza mappa y [m]','Risoluzione'};
dlgtitle = 'Inserire i dati della mappa';               
dati_mappa = inputdlg(prompt,dlgtitle,dims); % funzione prompt comandi
map_size_x=str2num(dati_mappa{1}); 
map_size_y=str2num(dati_mappa{2}); 
risoluzione=str2num(dati_mappa{3}); %avendo la risoluzione a 10, la precisione della mappa è al cm  
map = binaryOccupancyMap(map_size_x,map_size_y,risoluzione);
% matrice_mappa = occupancyMatrix(map);
% Metto i muri sulla mappa, ponendo a uno i bordi della matrice
%     matrice_mappa(:,1)=1;
%     matrice_mappa(:,size(matrice_mappa,2))=1;
% 
%     matrice_mappa(1,:)=1;
%     matrice_mappa(size(matrice_mappa,1),:)=1;
% 
%  map=binaryOccupancyMap(matrice_mappa);
 
%% Inserimento ostacoli
%Successivamente l'utente può scegliere se posizionare un numero N di
%ostacoli in maniera casuale nella mappa, oppure può scegliere
%personalmente la posizione di essi.
dims = [1 50];
prompt = {'Numero di ostacoli','Raggio di ogni ostacolo'};
dlgtitle = 'Inserire il numero di ostacoli presenti nella mappa e il loro raggio';               
numero_ostacoli = inputdlg(prompt,dlgtitle,dims);
N=str2num(numero_ostacoli{1});
raggio_ostacoli=str2num(numero_ostacoli{2});
soglia_ostacoli=((raggio_ostacoli*2)+raggio_ostacoli*0.2)*risoluzione;

choice = menu('Scelta sul posizionamento degli ostacoli','Random','Personalizzata');

switch choice
    case 1
        for i=1:1:N
        obstacle_positions(i,1)=randi([0 map_size_x],1,1);
        obstacle_positions(i,2)=randi([0 (map_size_y)],1,1);
        end
        
%  % La matrice delle distanze ( triangolare) contiene le distanze di ogni vertice
%  %del diagramma di Voronoi dagli altri punti. 
%         for i=1:1:size(obstacle_positions,1)
%             for j=1:1:size(obstacle_positions,1)
%                 distanze_ogni_punto(i,j)=pdist([obstacle_positions(i,:);obstacle_positions(j,:)],'euclidean');
%                 if j==i
%                     continue
%                 end
%             end
%         end
% distanze_ogni_punto=triu(distanze_ogni_punto);        
%         
% riga_da_cambiare=5;
% colonna_da_cambiare=10;
% 
% % Funzionante. Cambia la posizione dell'ostacolo qualora esso si sovrapponga ad un altro. 
%  while(riga_da_cambiare~=0 & colonna_da_cambiare~=0)
%     [riga_da_cambiare, colonna_da_cambiare] = find(distanze_ogni_punto<soglia_ostacoli & distanze_ogni_punto>0);
% 
%     for i=1:1:size(riga_da_cambiare,1)
%         obstacle_positions(riga_da_cambiare(i),1)=randi([0 map_size_x],1,1);
%         obstacle_positions(riga_da_cambiare(i),2)=randi([0 map_size_y],1,1);
%     end
% 
%     for j=1:1:size(colonna_da_cambiare,1)
%         obstacle_positions(colonna_da_cambiare(j),1)=randi([0 map_size_x],1,1);
%         obstacle_positions(colonna_da_cambiare(j),2)=randi([0 map_size_y],1,1);
%     end
%     
%      for i=1:1:size(obstacle_positions,1)
%             for j=1:1:size(obstacle_positions,1)
%                 distanze_ogni_punto(i,j)=pdist([obstacle_positions(i,:);obstacle_positions(j,:)],'euclidean');
%                 if j==i
%                     continue
%                 end
%             end
%         end
% distanze_ogni_punto=triu(distanze_ogni_punto); 
% [riga_da_cambiare, colonna_da_cambiare] = find(distanze_ogni_punto<soglia_ostacoli & distanze_ogni_punto>0);
% end

    case 2    
        h=msgbox('Scelta posizione ostacolo');
    uiwait(h,5);
         if ishandle(h) == 1
         delete(h);
         end
                  
        xlabel('Usa il click sx del mouse per posizionare ostacolo','Color','black');
        but=0;
        
        axis([0 map_size_x 0 map_size_y])
        grid on;
        hold on;
        while (but ~= 1) %Ripete finché non premi click sx
        [xval,yval,but]=ginput(N);
        end
      
       obstacle_positions=[xval yval];
    
end

%% Posizionamento ostacoli sulla mappa
axis([0 map_size_x 0 map_size_y])
setOccupancy(map,obstacle_positions,1);
inflate(map,(raggio_ostacoli));
show(map)
figure(1)
grid on
grid minor
hold on

%% Scelta delle pose iniziali e finali per il planning

 h=msgbox('Scelta della start pose');
    uiwait(h,5);
         if ishandle(h) == 1
         delete(h);
         end
                  
        xlabel('Usa il click sx del mouse per scegliere la posizione iniziale','Color','red');
        but=0;
        axis([0 map_size_x 0 map_size_y])
        grid on;
        hold on;
        while (but ~= 1) %Ripete finché non premi click sx
        [xval,yval,but]=ginput(1);
        end
        
    start_pose=[xval yval];
    
 h=msgbox('Scelta della goal pose');
    uiwait(h,5);
         if ishandle(h) == 1
         delete(h);
         end
                  
        xlabel('Usa il click sx del mouse per scegliere la posizione finale','Color','blue');
        but=0;
        axis([0 map_size_x 0 map_size_y])
        grid on;
        hold on;
        while (but ~= 1) %Ripete finché non premi click sx
        [xval,yval,but]=ginput(1);
        end
        
    goal_pose=[xval yval]; 
    
    plot(start_pose(:,1),start_pose(:,2),'*');
    hold on
    plot(goal_pose(:,1),goal_pose(:,2),'o');
    hold on

%% Diagramma di Voronoi
% mi restituisce i vertici e le celle 
[vertici_diagramma,celle]=voronoin(obstacle_positions);
[vx,vy]=voronoi(obstacle_positions(:,1),obstacle_positions(:,2));
graph=voronoi(obstacle_positions(:,1),obstacle_positions(:,2));

% mi disegna il diagramma
voronoi(obstacle_positions(:,1),obstacle_positions(:,2))

%% Cancellazione dei dati del diagramma fuori range

[vx_range,vy_range,vertici_diagramma_range]=elimina_dati_fuori_range (vx,vy,vertici_diagramma,map_size_x,map_size_y);

%% Algoritmo di ricerca del percorso
%Calcolo il costo per ogni edge del diagramma di voronoi, che equivale alla
%distanza euclidea tra i vertici del grafo. 
for i=1:1:size(vx,2)
    edge=[vx(:,i) vy(:,i)];
    edge(1,1)=vx(1,i)-vx(2,i);
    edge(1,2)=vy(1,i)-vy(2,i);
    edge(2,:)=[];
    pesi(i)=vecnorm(edge);
end

% ricavo la dimensione della cella più grande
for i=1:1:size(celle,1)
    tempor=cell2mat(celle(i));
    dim(i)=size(tempor,2);
end
M=max(dim);

%conversione del cell array in matrice
for i=1:1:size(celle,1)
    temp=cell2mat(celle(i));
    dim=size(temp,2);
    if(dim==M)
        celle_mat(i,1:dim)=temp;
    else
    %celle_mat(i,1:dim)=temp;
    celle_mat(i,1:M)=[temp zeros(1,(M-dim))];
    end
    clear temp;
end

for i=1:1:size(celle_mat,1)
    for j=1:1:(size(celle_mat,2)-1)
        if (celle_mat(i,j)~=0) && (celle_mat(i,j+1)~=0)
            ad(celle_mat(i,j),celle_mat(i,j+1))=1;
            ad(celle_mat(i,j+1),celle_mat(i,j))=1;

        else
            if (celle_mat(i,j)~=0) && (celle_mat(i,j+1)==0)
                ad(celle_mat(i,1),celle_mat(i,j))=1;
                ad(celle_mat(i,j),celle_mat(i,1))=1;
            else
                continue
            end
        end
    end
end
toc











