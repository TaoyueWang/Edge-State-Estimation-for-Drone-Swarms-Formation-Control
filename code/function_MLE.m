function z_record = function_MLE(N,K,z,L,dt,mu,R,T)
%FUNCTION_MLE Summary of this function goes here
%   Detailed explanation goes here
k = 1;
z_record = zeros(N,2,K);
z_record(:,:,1) = z;

H = kron(ones(T,1),eye(2));
mu = repmat(mu,1,T);
sigma = kron(eye(T),R);

while k < K
    for i = 4:N
        u = 0;
        for j = 1:N
            if L(i,j)~=0 & i~=j
                v = mvnrnd(mu,sigma,1)';
                y = H * (z_record(i,:,k)-z_record(j,:,k))' + v;
                z_hat = (1/T) .* H' * y;    % MLE estimate of the state
                u = u + L(i,j) * z_hat';
            end
        end
        z_record(i,:,k+1) = z_record(i,:,k)+dt*u;
    end
    z_record(1:3,:,k+1) = z_record(1:3,:,k);
    k = k+1;
end

end

