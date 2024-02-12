function z_record = control_with_kalman(N,K,z,L,dt,mu,R,T)
% Implementation of the Edge-based Kalman Filter for edge state estimation

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
    H = kron(ones(T,1),eye(2));  % matrix H of the linear gaussian model
    z_hat_record = zeros(N,2,N);  % record the state estimate for all edges, initialize to zero
    u_record = zeros(N,2);  % record the control input for all nodes
    sigma_record = repmat(2*eye(2),N,1,N); % record the estimated covariance matrix for all edges
                                           % initialize to 2*I
    
    while k < K
        for i=4:N
            u = 0;
            for j=1:N
                if i == j
                    u = u;
                else
                    % prediction step
                    z_prediction = z_hat_record(i,:,j)+dt*(u_record(i,:)-u_record(j,:)); % predict the edge state
                    sigma_prediction = sigma_record(2*i-1:2*i,:,j); % predict the covariance matrix
                    % update step
                    v = mvnrnd(mu, R_hat, 1);
                    y = H*(z_record(i,:,k)-z_record(j,:,k))' + v';
                    K_gain = sigma_prediction*H'*inv(H*sigma_prediction*H'+R_hat);  % calculate Kalman gain
                    z_new = z_prediction'+K_gain*(y-H*z_prediction'); % update the edge state
                    sigma_new = (eye(2)-K_gain*H)*sigma_prediction;  % update the covariance matrix

                    u = u+L(i,j)*z_new';  % control law
                    z_hat_record(i,:,j) = z_new';
                    sigma_record(2*i-1:2*i,:,j) = sigma_new;
                end
            end
            z_record(i,:,k+1) = z_record(i,:,k)+dt*u;  % dynamics update
            u_record(i,:) = u;
        end
        z_record(1:3,:,k+1) = z_record(1:3,:,k);   % keep the first three nodes unchanged
        k = k+1;
    end
end