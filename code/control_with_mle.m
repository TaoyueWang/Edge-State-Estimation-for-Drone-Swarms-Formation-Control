function z_record = control_with_mle(N,K,z,L,dt,mu,R,T)
% Implementation of the Maximum Likelihood Estimator for edge state
% estimation

% Inputs:
% N: number of nodes (agents)
% K: total number of iterations
% z: the initial position vector, shape:(N,2)
% L: the generalized Laplacian matrix of the graph
% dt: the time step of one dynamics update
% mu: the mean of the generated Gaussian noise vector, should be [0 0]
% R: the covariance matrix of the generated Gaussian noise vector
% T: the number of measurements per control update

% Outputs:
% z_record: the position vector of each iteration, shape:(N,2,K)

    k = 1;
    z_record = zeros(N,2,K);
    z_record(:,:,1) = z;
    mu = repmat(mu,1,T);  % mean of the stacked noise vector
    R_hat = kron(eye(T),R);  % covariance matrix of the stacked noise vector
    H = kron(ones(T,1),eye(2)); % matrix H of the linear gaussian model
    
    while k < K
        for i=4:N
            u = 0;
            for j=1:N
                if i == j
                    u = u;
                else
                    v = mvnrnd(mu, R_hat, 1);  % draw observation noise of T realizations for each edge
                    y = H*(z_record(i,:,k)-z_record(j,:,k))' + v';  % get T noisy measurements
                    z_hat = H'*y/T;  % MLE estimate of the relative position for each edge
                    u = u+L(i,j)*z_hat';  % control law
                end
            end
            z_record(i,:,k+1) = z_record(i,:,k)+dt*u;  % dynamics update
        end
        z_record(1:3,:,k+1) = z_record(1:3,:,k);  % keep the first three nodes unchanged
        k = k+1;
    end
end