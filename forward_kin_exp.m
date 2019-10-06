function gst_final = forward_kin_exp(thetas)
    %Accepts the argument vector "thetas," which is a 5-by-1 vector of the 
    %joint positions.
    %Returns gst_final, a 4-by-4 matrix of the resulting transformation.

    %This function uses the product-of-exponential method.

    %Segment lengths, in inches:
    L0 = 13;
    L1 = 14.7;
    L2 = 12;
    L3 = 12;
    L4 = 9;

    %calculating zero configuration (all angles zero - account for angle offset
    %later)
    q_st = [L2+L3+L4, L0, L1].'; %note that q is "vertical"
    gst_0 = eye(4);
    gst_0(1:3, 4) = q_st
    
    %calculating positions of each q at zero configuration
    q1 = [0; L0; L1];
    q1 = q1; q2 = q1;
    q3 = q2 + [L2; 0; 0];
    q4 = q3 + [L3; 0; 0];
    q5 = q4 + [L4; 0; 0];
    
    %accounting for offset in joint angles
    offset = [-0.003 -0.002 0 0 0 -1.571].';
    theta_actual = thetas - offset;
    
    %let I be the 3-by-3 identity matrix
    I = eye(3);
    
    %Calculating the twist exponentials
    e1 = eye(4);
    R1 = Rz(theta_actual(1));
    e1(1:3, 1:3) = R1;
    e1(1:3, 4) = (I-R1)*q1;
    
    e2 = eye(4);
    R2 = Ry(theta_actual(2));
    e2(1:3, 1:3) = R2;
    e2(1:3, 4) = (I-R2)*q2;
    
    e3 = eye(4);
    R3 = Ry(theta_actual(3)*-1); %because this rotation is antiparallel to Y axis
    e3(1:3, 1:3) = R3;
    e3(1:3, 4) = (I-R3)*q3;
    
    e4 = eye(4);
    R4 = Ry(theta_actual(4)*-1); %because this rotation is antiparallel to Y axis
    e4(1:3, 1:3) = R4;
    e4(1:3, 4) = (I-R4)*q4;
    
    e5 = eye(4);
    R5 = Rx(theta_actual(5));
    e5(1:3, 1:3) = R5;
    e5(1:3, 4) = (I-R5)*q5;
    
    gst_final = e1*e2*e3*e4*e5*gst_0;
end

function R_x = Rx(theta)
    %takes a given angle, theta, and returns a 3-by-3 rotation matrix.
    R_x = [1 0 0; 0 cos(theta) -1*sin(theta); 0 sin(theta) cos(theta)];
end

function R_y = Ry(theta)
    %takes a given angle, theta, and returns a 3-by-3 rotation matrix.
    R_y = [cos(theta) 0 -1*sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
end

function R_z = Rz(theta)
    %takes a given angle, theta, and returns a 3-by-3 rotation matrix.
    R_z = [cos(theta) -1*sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
end