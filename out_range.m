function [A,B,C]=elimina_dati_fuori_range (D,E,F,G,H)
    indici_out_range_x_prima=find(D(1,:)<0 | D(1,:)>G);
indici_out_range_x_seconda=find(D(2,:)<0 | D(2,:)>G);
indici_out_range_x=[indici_out_range_x_prima indici_out_range_x_seconda];
%
indici_out_range_y_prima=find(E(1,:)<0 | E(1,:)>H);
indici_out_range_y_seconda=find(E(2,:)<0 | E(2,:)>H);
indici_out_range_y=[indici_out_range_y_prima indici_out_range_y_seconda];
% 
indici_out_range=[indici_out_range_x indici_out_range_y];
% Creo un nuovo vettore di vertici con tutti gli edge nel range della mappa 
A=D;
B=E;
A(:,indici_out_range)=[];
B(:,indici_out_range)=[];

% Stesso ragionamento per i vertici 
 
indici_out_range_vertici_prima=find(F(:,1)<0 | F(:,1)>G);
indici_out_range_vertici_seconda=find(F(:,2)<0 | F(:,2)>H);
indici_out_range_vertici=[indici_out_range_vertici_prima ;indici_out_range_vertici_seconda];
indici_out_range_vertici=unique(indici_out_range_vertici);
 
% Creo un nuovo vettore di vertici con tutti gli edge nel range della mappa 
C=F;
C(indici_out_range_vertici,:)=[];
end
