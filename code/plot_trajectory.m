function plot_trajectory(z_record)
% Plot the trajectories of convergence and the final position

% Input: z_record: the position vector of each iteration, shape:(N,2,K)

    [~,~,K] = size(z_record);

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

    figure; hold on
    % trajectory formation
    for i=1:N
        %plot(z(i,1),z(i,2),'.','markersize',3);
        plot(reshape(z_record(i,1,:), [1,K]),reshape(z_record(i,2,:), [1,K]),'.','markersize',3);
    end

    %target formation
    z = z_record(:,:,K);
    for i=1:M
        plot(z(edges(:,i),1),z(edges(:,i),2),'k','linewidth',1.5); 
    end
    for i=1:N
        plot(z(i,1),z(i,2),'r.','markersize',50);
    end
    axis([-2 2 -2 2]);
    hold off
end