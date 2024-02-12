function z_record = function_MMSE(N,K,z,L,dt,mu,R,T,var_prior)
%FUNCTION_MLE Summary of this function goes here
%   Detailed explanation goes here
k = 1;
z_record = zeros(N,2,K);
z_record(:,:,1) = z;

H = kron(ones(T,1),eye(2));
mu = repmat(mu,1,T);
sigma = kron(eye(T),R);

z_hat_all = zeros(N,N,2);
prior_sigma = var_prior * eye(2);

while k < K
    for i = 4:N
        u = 0;
        for j = 1:N
            if L(i,j)~=0 & i~=j
                
                rng(k);
                v = mvnrnd(mu,sigma,1)';
                y = H * (z_record(i,:,k)-z_record(j,:,k))' + v;
                z_hat_pre = reshape(z_hat_all(i,j,:),[2,1]);
                z_hat = z_hat_pre + prior_sigma * H' * inv(H*prior_sigma*H'+sigma)*(y-H*z_hat_pre); % MMSE estimate of the state
                z_hat_all(i,j,:) = reshape(z_hat,[1,1,2]);
                u = u + L(i,j) * z_hat';
            end
        end
        z_record(i,:,k+1) = z_record(i,:,k)+dt*u;
    end
    z_record(1:3,:,k+1) = z_record(1:3,:,k);
    k = k+1;
end

end
