function G = knngraph(data,k,r,w)
% data: row vector
% w: weight for each elements
% k: k for knn
% r: search radius

N = size(data,1);
w = w/sum(w)*length(w);
G = zeros(N);
%%

% dp = (data' - reshape(data',size(data,2),1,[])).*w';
% dist = sqrt(sum(dp.^2));
% dist = reshape(dist,N,N,1);
% [D1,idx1] = sort(dist,2);
% D1 = D1(:,1:k+1);
% idx1 = idx1(:,1:k+1);
% 
% indx0 = 1:k+1;
% G = zeros(N);
% for i = 1:N
%     indxk = idx1(i,D1(i,indx0)<=r);
%     G(i,indxk) = dist(i,indxk);
% end

%%
data = data.*w;
%KDTree = KDTreeSearcher(data); 
[idx, D] = rangesearch(data, data, r);
for i = 1:N
    minind = min([k+1,length(idx{i})]);
    G(i,idx{i}(1:minind)) = D{i}(1:minind);
end

%%
Gp = G';
G(G==0 & Gp~=0) = Gp(G==0 & Gp~=0);




end