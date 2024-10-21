function iscollision = iscollision_Link5R_2d(frames,O)
n = size(frames,2)-1;
nsamples = size(frames,3);

models = NaN(2,n*2,nsamples);
models(:,1:2:end,:) = frames(:,1:end-1,:);
models(:,2:2:end,:) = frames(:,2:end,:);
Ls = reshape(models,2,2,[]);
iscollision = false(length(O),nsamples);
for i = 1:length(O)
    iscollision(i,:) = any(reshape(lineseg_meet_convex(O{i},Ls),n,[],1));
end
iscollision = any(iscollision,1);

end