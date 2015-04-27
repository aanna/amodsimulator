%% load nodes
nodes = dlmread('nodes.txt');
%scatter(nodes(:,2), nodes(:,3))
%% kmeans
K = 50;
[idx,C] = kmeans(nodes(:,2:3), K);
cmap = hsv(K);
for i=1:K
    plot(nodes(idx==i,2),nodes(idx==i,3),'.','Color', cmap(i,:),'MarkerSize',12); hold on;
end
hold off;
%%
dlmwrite('stations.txt', [(1:K)' C], 'delimiter', ' ', 'precision', 10);