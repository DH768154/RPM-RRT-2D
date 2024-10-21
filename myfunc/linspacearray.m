function result = linspacearray(x1,x2,n)
% linspace for array
[r,c] = size(x1);
if r~=1 && c~=1
    error('x1,x2 are 1d array')
end

if c==numel(x1)
    x1 = x1'; x2 = x2';
    fliprc = true;
else
    fliprc = false;
end

steps = (x2-x1)/(n-1);
ind = 0:n-1;
result = x1+steps*ind;
if fliprc
    result = result';
end
end