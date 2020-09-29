clear all
close all
clc
%% NOTE
%dimensione stanza come input e anche robot size
%% Generazione del workspace e degli ostacoli 

raggio_disco = 10; %imposto il raggio del disco 
stanza=ones(500,500); % Creo il workspace, inizializzando una matrice di 1 di dimensione 500x500

numero_ostacoli=input('Inserire il numero di ostacoli: ');%prendo da tastiera il numero di ostacoli presenti nella mappa
disp('Disegnare i poligoni sulla figura. Doppio click su ogni poligono dopo aver selezionato al ROI per andare avanti: ');

    for i=1:numero_ostacoli
         ostacoli=roipoly(stanza); % seleziono la region of interest ( ROI) per ogni ostacolo
         stanza=stanza-ostacoli; %Inserisce uno alla volta ogni ostacolo nella matrice stanza. 
    end 
    
save('obstacle_space.mat','stanza'); %salva il workspace in formato .mat
stanza = load('obstacle_space.mat');
stanza = stanza.stanza;
stanza_dim = size(stanza); % in stanza_dim c'è la dimensione della stanza
imshow(stanza,[]); % Mostro il nuovo workspace aggiornato

hold on;

%% Generazione del diagramma di Voronoi

%Applicazione della growing procedure per la costruzione del C Space

C_space= imdilate(~stanza, strel('disk',raggio_disco)); %dilata la forma degli ostacoli, di una dimensione pari a raggio disco di forma sferica. 
C_space=~C_space; % inverto gli 0 con gli 1
C_space(1:raggio_disco,:)=0;
C_space(:,1:raggio_disco)=0;
C_space(stanza_dim(1)-raggio_disco:stanza_dim(1),:)=0;
C_space(:,stanza_dim(1)-raggio_disco:stanza_dim(1))=0;
% growing delle pareti dell stanza, della dimensione di raggio disco. 
C_space_dist = bwdist(1-C_space); % matrice delle distanze di ogni pixel dal pixel non nullo più vicino. Con 1-C_space invertiamo gli 0 e gli 1 ( 0 nel C free e 1 nell'ostacolo). con bwdist calcoliamo la distanza di ogni nuovo 0 dall'1 più vicino ( bordo ostacolo). Ora negli ostacoli avremo 0, nel C free le distanze progressive.

% Calcolo del diagramma di Voronoi con le funzioni del2 e rescale. del2
% restituisce una matrice di uguale dimensione di C Space dove ogni
% elemento è ottenuto come differenza dell'elemento di C space meno la
% media dei 4 vicini. 
        [GVD]= rescale(del2(C_space_dist))<0.5; % con questo comando quando 
        %i valori sono minori della metà del range sono settati a 1, quando
        %maggiori sono settati a 0. E' un valore logico, quindi 1 quando è
        %vero e 0 quando è falso. Nella matrice che viene confrontata con
        %0.5 i pixel che hanno valore minore di 0.5 appartengono al
        %diagramma, per cui viene assegnato loro valore pari a 1. Tutti gli
        %altri hanno valore maggiore di 0.5, per cui viene loro assegnato
        %valore di 0 logico. 
        %Filtraggio del diagramma di Voronoi appena calcolato 
        
        %operazioni di filtraggio sul diagramma di Voronoi appena calcolato
     
        GVD = bwmorph(GVD,'spur'); % rimuove i pixel finali delle linee senza rimuovere completamente gli oggetti piccoli 
        GVD = bwmorph(GVD,'thin'); % fa in modo che le file di GVD siano formate da una sola linea di 1
        GVD = bwmorph(GVD,'clean');% rimuove i pixel isolati ( 1 circondati da 0 ) 
        GVD=bwareaopen(GVD,30); % rimuove gli oggetti ( i blocchi di 1) isolati con meno di un certo numero di pixel ( 500 nel caso in esame)

%% Definizione della start pose
matrice_robot=zeros(500,500);

robot_start = []; % Inizializzazione di robot start ed end . 
robot_end = [];

t=1;% Istante di tempo.

bool=true;
risposta_utente=bool; % inizializzo variabile risposta utente, di tipo booleano. 

while(risposta_utente==true) % il ciclo si ripete finché l'utente vuole selezionare una nuova posizione.È stato utilizzato questo codice per dimostrare che l'approccio è multiple-query
% Seleziono il nodo di partenza del robot
    disp('Selezionare la posizione di partenza: ');
    start_point = ginput(1); 
    robot_start=start_point; % acquisisco da mouse la posizione di partenza del robot
    
   
 % Controllo sulla posizione corretta 
    
    origin = [robot_start(1) robot_start(2)]; %centro della nuova matrice di controllo collisioni, in start position
[xx,yy] = meshgrid((1:size(matrice_robot,2))-origin(1),(1:size(matrice_robot,1))-origin(2)); % seleziona la porzione di matrice nella quale voglio andare a mettere gli 1, che sarebbe la posizione occupata del robot. 
matrice_robot(sqrt(xx.^2 + yy.^2) <= raggio_disco) = 1; % punti interni alla matrice =1, entro la misura del raggio del robot. 
bool=1; % inizializzo la variabile booleana a 1 
if (start_point(1)+raggio_disco <= stanza_dim(1) && start_point(2)+raggio_disco <=stanza_dim(2) && start_point(1)-raggio_disco>=1 && start_point(2)-raggio_disco>=1) % condizioni nelle quali la posizione scelta dall'utente non è valida
for i=1:1:size(stanza,1)
for j=1:1:size(stanza,2)
if (matrice_robot(i,j)&(~stanza(i,j)==1)) % controllo sulla collisione. Se il risultato dell'And logico è uguale a 1, vuol dire che c'è collisione quindi in corrispondenza dello stesso pixel abbiamo due 1, per cui la posizione non è ammissibile. 
bool=0; % la variabile viene settata a 0 in questo caso. 
else
continue
end
end
end
else
    bool=0;
end

if bool==0
% Controllo sulla posizione corretta. La posizione non è ammissibile quando
% la variabile bool è 1. Le posizioni non ammissibili sono quelle dove il
% robot si trova troppo vicino ai bordi della stanza, oppure viene scelta
% una posizione interna o nelle immediate vicinanze di un ostacolo. 
while(~(bool==1 && start_point(1)+raggio_disco <= stanza_dim(1) && start_point(2)+raggio_disco <=stanza_dim(2) && start_point(1)-raggio_disco>=1 && start_point(2)-raggio_disco>=1))
disp('Posizione di partenza non ammissibile. Selezionarne una nuova')
start_point= ginput(1);
clear robot_start;
robot_start=start_point;
clear matrice_robot;
matrice_robot = zeros(500);
origin = [robot_start(1) robot_start(2)]; %centro della matrice
[xx,yy] = meshgrid((1:size(matrice_robot,2))-origin(1),(1:size(matrice_robot,1))-origin(2)); 
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
    
    % Il robot ha forma circolare, di raggio scelto dall'utente. La funzione è usata per dare
    % la forma circolare al robot. 
    rectangle('Position',[start_point(1)-raggio_disco, start_point(2)-raggio_disco, 2*raggio_disco, 2*raggio_disco],'Curvature',[1 1], 'EdgeColor', 'r', 'FaceColor', 'r');
  
    
%% Definizione della goal pose   
 % Scelgo la goal pose
    disp('Selezionare la posizione di arrivo ');
    end_point = ginput(1);
    robot_end = end_point;
   matrice_robot=zeros(500,500);
   
    % Controllo sulla posizione corretta, esattamente come fatto per la
    % start pose. 
    
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

    
    % Mostro le posizioni di partenza e arrivo per l'algoritmo di planning.
    plot(end_point(:,1),end_point(:,2),'g--o');
       

           
%% Operazioni sul diagramma di Voronoi

%Trovo il punto del GVD più vicino a robot start, indispensabile per
%completare l'algoritmo di retraction .

[yRows,xColumns]=find(GVD==1); % trovo le coordinate dei punti dove GVD vale 1, ossia i punti del diagramma di Voronoi
distances = sqrt((xColumns - robot_start(1)).^2 + (yRows - robot_start(2)).^2); % calcolo le distanze,tramite il teorema di pitagora, di robot start da ogni punto del GVD 
% Trovo il più vicino
[minDistance, indexOfMin] = min(distances);
% .. e le sue coordinate
GVD_start(1) = xColumns(indexOfMin);
GVD_start(2) = yRows(indexOfMin);

% medesima operazione per GVD end 
distances2 = sqrt((xColumns - robot_end(1)).^2 + (yRows - robot_end(2)).^2);
[minDistance2, indexOfMin2] = min(distances2);
GVD_end(1) = xColumns(indexOfMin2);
GVD_end(2) = yRows(indexOfMin2);

 %bwdistgeodesic calcola le distanze sul GVD dai punti selezionati . Quindi
 %per quanto riguarda D1(D2), alla coordinata GVD start(end) viene impostato in D1(D2) valore
 %0, a tutti i pixel connessi a GVD start (end), quindi quelli che hanno valore
 %1,viene sostituito il valore della distanza di ognuno di essi dal punto
 %di partenza(arrivo). A quelli non connessi viene sostituito il valore Nan
 
D1 = bwdistgeodesic(GVD, GVD_start(1), GVD_start(2), 'quasi-euclidean');
D2 = bwdistgeodesic(GVD, GVD_end(1), GVD_end(2), 'quasi-euclidean');

D = D1 + D2; % Viene effettuata la somma membro a membro delle due matrici. 
D = round(D); % Viene arrotondato ognuno dei valori della matrice all'intero più vicino. 

D(isnan(D)) = inf; % sostituisce i Nan con gli inf
skeleton_path = imregionalmin(D); 
% Selezionata una matrice di ingresso D, la funzione imregionalmin individua
% una regione di pixel uguali tra loro
% circondata da pixel che hanno valore maggiore, ed assegna ad essa 
% valore unitario, ponendo a 0 tutti gli altri pixel. 
[xp,yp]=find(skeleton_path==1);
% in xp e yp vengono salvate le coordinate del percorso di minima distanza
% appena trovato 
percorso=[yp,xp];

%Viene effettuato un controllo sulla realizzabilità o meno del percorso
%individuato. Viene calcolata la distanza tra GVD start(end) e robot
%start(end), e qualora questa distanza risulti superiore ad una soglia
%prefissata l'algoritmo non porta a un percorso realizzabile, per via di un
%passaggio stretto ad esempio. 
check1=[GVD_start ; robot_start];
check2=[GVD_end ; robot_end];
path_length = D(skeleton_path); %  Calcola e restituisce la lunghezza del percorso appena calcolato 
path_length = path_length(1);

if (pdist(check1)>stanza_dim(1)*0.2 || pdist(check2)>stanza_dim(1)*0.2 || path_length==Inf)
    imshow(~(~C_space|GVD))
    msgbox('Percorso non trovato')
else

% rappresentazione grafica del percorso calcolato sul voronoi diagram 
P = imoverlay(GVD, imdilate(skeleton_path, ones(3,3)), [1 0 0] );
imshow(P, 'InitialMagnification', 200)
hold on
plot(GVD_start(1), GVD_start(2), 'g*', 'MarkerSize', 15)
plot(GVD_end(1), GVD_end(2), 'g*', 'MarkerSize', 15)
plot(robot_start(1),robot_start(2),'d','MarkerSize',8)
plot(robot_end(1),robot_end(2),'d','MarkerSize',8)
hold off
path_length = D(skeleton_path); % calcola la distanza su skeleton path da GVD start a GVD end 
dist_retr_start=sqrt((GVD_start(1)-robot_start(1))^2+(GVD_start(2)-robot_start(2))^2);
dist_retr_end=sqrt((GVD_end(1)-robot_end(1))^2+(GVD_end(2)-robot_end(2))^2);

%per la distanza totale del percorso al valore percorso sul GVD va aggiunto
%il valore di distanza percorsa dal robot da robot_start(end) a
%GVD_start(end), come imposto dall'algoritmo di retrazione. 
path_length = path_length(1)+dist_retr_start+dist_retr_end


imshow(~(~C_space|GVD))
% Rappresentazione dei percorsi e delle posizioni precedentemente
% calcolate. 
hold on
plot(GVD_start(1),GVD_start(2),'*','MarkerSize',8)

plot(GVD_end(1),GVD_end(2),'*','MarkerSize',8)

plot(robot_start(1),robot_start(2),'d','MarkerSize',10)

plot(robot_end(1),robot_end(2),'d','MarkerSize',10)


plot(percorso(:,1),percorso(:,2),'o','color','r')
hold on
% Retraction algorithm
plot([GVD_start(1),robot_start(1)],[GVD_start(2),robot_start(2)],'color','r','LineWidth',6)
plot([GVD_end(1),robot_end(1)],[GVD_end(2),robot_end(2)],'color','r','LineWidth',6)
end
 
%Menu di scelta di nuove posizioni. Task richiesto per la verifica del
%Multy-query. 
answer = questdlg('Vorresti modificare le posizioni di arrivo e partenza del robot?', 'Choice Menu','Si','No','No');
switch answer
case 'Si'
risposta_utente = true;
case 'No'
risposta_utente=false;
end

end
