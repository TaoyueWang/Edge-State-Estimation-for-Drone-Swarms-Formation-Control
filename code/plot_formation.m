function plot_formation(z)
% this function plots the formation in graph representation for the
% formation control project of ET4386. The input z should be a N-by-D
% matrix that contains D-dimensional locations for N agents, e.g., 
% z = [2,0;1,1;1,-1;0,1;0,-1;-1,1;-1,-1]; Change this file to accomodate
% the need for your use.

    % topology graph
    B = [1,-1,0,0,0,0,0,0,0,-1,0,1;
        -1,0,0,0,0,0,1,-1,0,0,0,0;
        0,1,-1,0,0,0,0,0,1,0,0,0;
        0,0,0,0,0,1,-1,0,0,1,-1,0;
        0,0,1,-1,0,0,0,0,0,0,1,-1;
        0,0,0,0,1,-1,0,0,-1,0,0,0;
        0,0,0,1,-1,0,0,1,0,0,0,0];
    % dimensions
    [N,M] = size(B);
   
    % edge set
    edges = mod(reshape(find(B~=0),2,M),N);
    edges(edges==0) = N;

    %target formation
    figure; hold on
    for i=1:M
        plot(z(edges(:,i),1),z(edges(:,i),2),'k','linewidth',1.5); 
    end
    for i=1:N
        plot(z(i,1),z(i,2),'r.','markersize',50);
    end
    axis([-2 2 -2 2]);
    hold off
end

