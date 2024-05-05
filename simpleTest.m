clear all;
close all;
clc;


X = randn(1600, 2);
Y = randn(10, 2);
D = pdist2(X,Y,'euclidean'); % euclidean distance

subs = 1:size(X,1);

[val inx]=min(D);

scatter(X(:,1),X(:,2),5,'K'); %Munsell 5B
hold on,
scatter(Y(:,1),Y(:,2),5,'r'); %Munsell 5B
hold on,


for i=1: max(size(inx)),
    
   Bufx=[Y(i,1) X(inx(i),1)];
   Bufy=[Y(i,2) X(inx(i),2)];
   line(Bufx,Bufy,'Color','k','LineWidth',1);
   hold on,    
end

grid on;


