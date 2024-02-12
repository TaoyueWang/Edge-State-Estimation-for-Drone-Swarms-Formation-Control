function z_record = control_without_noise(N,K,z,L,dt)
% Implementation of the formation control system without noise modeling

% Inputs:
% N: number of nodes (agents)
% K: total number of iterations
% z: the initial position vector, shape:(N,2)
% L: the generalized Laplacian matrix of the graph
% dt: the time step of one dynamics update

% Outputs:
% z_record: the position vector of each iteration, shape:(N,2,K)

    k = 1;
    z_record = zeros(N,2,K);
    z_record(:,:,1) = z;
    while k < K
        for i=4:N
            u = 0;
            for j=1:N
                if i == j
                    u = u;
                else
                    u = u+L(i,j)*(z_record(i,:,k)-z_record(j,:,k)); % control law
                end
            end
            z_record(i,:,k+1) = z_record(i,:,k)+dt*u; % dynamics update
        end
        z_record(1:3,:,k+1) = z_record(1:3,:,k); % keep the first three nodes unchanged
        k = k+1;
    end
end