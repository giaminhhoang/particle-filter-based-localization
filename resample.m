function [x,w,ind] = resample(x,w)

N = length(w);

u = ([0:N-1]+rand(1))/N;
% u = ([0:N-1]+rand(1,N))/N;
wc = cumsum(w);
wc = wc/wc(N);
[dum,ind1] = sort([u,wc]);
ind2 = find(ind1<=N);
ind = ind2 - (0:N-1);
x = x(:,ind);
w = ones(1,N)./N;