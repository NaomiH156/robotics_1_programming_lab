function output = check_SE3(T)
    %checks if a given 4x4 transformation matrix belongs to SE3
    true = 1;
    false = 0;
    
    %a transformation matrix is in SE3 if the following conditions are met:
        %all entries are real (not imaginary or complex)
        %the rotation part, R, has R^t R = 1
        %det(R) = 1
    R = zeros(3,3);
    R = T(1:3, 1:3);
    
    check_R = check_transpose_mult(R);
    
    
    if ((isreal(T) == 1) && (check_transpose_mult(R) == 1) && (det(R) == 1))
        output = true;
    else
        output = false;
    end   
end

function output = check_transpose_mult(R)
    true = 1;
    false = 0;
    
    I = eye(3);
    RTR = R.'*R;
    
    dif = sum(sum(RTR - I));
    
    if (dif <= 0.0001 ) %i.e. if RTR = I
                     %had to make this a less than statement because dif
                     %was evaluating to be around 3.7*10^-16 which is not
                     %useful lol
        output = true;
    else
        output = false;
    end
end