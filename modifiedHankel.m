function [H_1,H_2] = modifiedHankel(U_1, Y_1, L)
    % u: input data (nInputs x N)
    % y: output data (nOutputs x N)
    % L: parameter for the Hankel matrix
    
    % Get the length of the data
    N = length(U_1);

    % Initialize w(t) = [u(t); y(t)]
    W = [U_1;Y_1(:,1:end-1)]; % nInputs+nOutputs x N matrix (since u is nInputs x N and y is nOutputs x N)
    
    
    % Construct H_1
    H_1 = [];
    for i = 0:(L-1)
        H_1 = [H_1; W(:, (i+1):(N-L+i+1))];
    end
    
    % Construct H_2
    H_2 = Y_1(:, L+1:end);

    % Concatenate H_1 and H_2 to form H
    H = [H_1; H_2];

end