function e = calculate_error(z_record, z_star)
% Calculate the error of each iteration

% Inputs: 
% z_record: the position vector of each iteration, shape:(N,2,K)
% z_star: the target position vector, shape:(N,2)

% Output:
% e: the error of each iteration

    [N,~,K] = size(z_record);
    e = zeros(1,K);

    for k=1:K
        error = 0;
        for i=4:N
            error = error+norm(z_record(i,:,k)-z_star(i,:));
        end
        e(1,k) = error;
    end 
end