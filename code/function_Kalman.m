function z_record = function_Kalman(N,K,z,L,dt,mu,R,T)
%FUNCTION_MLE Summary of this function goes here
%   Detailed explanation goes here
k = 1;
z_record = zeros(N,2,K);
z_record(:,:,1) = z;

H = kron(ones(T,1),eye(2));
mu = repmat(mu,1,T);
sigma = kron(eye(T),R);

z_hat_all = zeros(N,N,2);   % estimated state for all edges
sigma_hat_all = zeros(N,N,2,2);  % estimated covariance matrix for all edges
for i=1:N
    for j=1:N
        sigma_hat_all(i,j,:,:) = 2*eye(2);
    end
end
u_all = zeros(N,2);   % control input for each node

while k < K
    for i = 4:N
        u = 0;
        for j = 1:N
            if L(i,j)~=0 & i~=j
                % prediction step
                z_hat_k_1_k_1 = reshape(z_hat_all(i,j,:),[2,1]);
                z_hat_k_k_1 = z_hat_k_1_k_1 + dt*u_all(i,:)' - dt*u_all(j,:)';
                % there is no need to change the covariance matrix
                sigma_hat_k_1_k_1 = reshape(sigma_hat_all(i,j,:,:),[2,2]);
                sigma_hat_k_k_1 = sigma_hat_k_1_k_1;
                
                % update step
                v = mvnrnd(mu,sigma,1)';
                y = H * (z_record(i,:,k)-z_record(j,:,k))' + v;
                Kalman_gain = sigma_hat_k_k_1 * H' * inv(H*sigma_hat_k_k_1*H'+sigma);
                z_hat_k_k = z_hat_k_k_1 + Kalman_gain * (y-H*z_hat_k_k_1);
                sigma_hat_k_k = (eye(2)-Kalman_gain*H) * sigma_hat_k_k_1;
                z_hat_all(i,j,:) = reshape(z_hat_k_k,[1,1,2]);
                sigma_hat_all(i,j,:,:) = reshape(sigma_hat_k_k,[1,1,2,2]);
                
                % control input
                u = u + L(i,j) * z_hat_k_k';
            end
        end
        u_all(i,:) = u;
        z_record(i,:,k+1) = z_record(i,:,k)+dt * u_all(i,:);
    end
    z_record(1:3,:,k+1) = z_record(1:3,:,k);
    k = k+1;
end

end

