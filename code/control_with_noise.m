function z_record = control_with_noise(N,K,z,L,dt,mu,R)
% Implementation of the formation control system with noise

% Inputs:
% N: number of nodes (agents)
% K: total number of iterations
% z: the initial position vector, shape:(N,2)
% L: the generalized Laplacian matrix of the graph
% dt: the time step of one dynamics update
% mu: the mean of the generated Gaussian noise vector, should be [0 0]
% R: the covariance matrix of the generated Gaussian noise vector

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
                    v = mvnrnd(mu, R, 1);  % draw observation noise for each edge
                    y = z_record(i,:,k)-z_record(j,:,k)+v;  % get noisy measurements
                    u = u+L(i,j) * y;  % control law
                end
            end
            z_record(i,:,k+1) = z_record(i,:,k)+dt*u;  % dynamics update
        end
        z_record(1:3,:,k+1) = z_record(1:3,:,k);  % keep the first three nodes unchanged
        k = k+1;
    end
end
