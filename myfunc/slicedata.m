function pp = slicedata(p1,p2,iter,slicenum0)
% 0   4   8   12
% 2   6   10
% 1   3   5   7   9   11
% 0.5 1.5 2.5 3.5 4.5 5.5 6,5 7.5 ... 11.5
%
% iter: nth iteration elements
% slicenum0: initially cut to how many piece

step0 = (p2-p1)/slicenum0;
if iter == 1
    steps = step0;
    n = (p2-p1)/steps+1;
else
    steps = step0/2^(iter-2);
    n = (p2-p1-2*step0/2^(iter-1))/steps+1;
    p1 = p1+step0/2^(iter-1);
    p2 = p2-step0/2^(iter-1);
end
pp = linspacearray(p1,p2,round(n));

end