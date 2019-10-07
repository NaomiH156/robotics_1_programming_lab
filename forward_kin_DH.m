function gst_final = forward_kin_DH(thetas)
    %Accepts the argument vector "thetas", which is a 5-by-1 vector of all
    %the joint positions.
    
    %Returns gst_final, a 4-by-4 matrix of the resulting transformation.
    
    %This function uses the Denavit-Hartenberg parameterization method.
    
    %Segment lengths, in inches:
    L0 = 13;
    L1 = 14.7;
    L2 = 12;
    L3 = 12;
    L4 = 9;
    
    %Transformation matrix from origin S to point 0:
        %This is a pure translation so this is simple:
    T_s0 = eye(4);
    T_s0 (1:3,4) = [0 -1*L0 L1].';
    
    %accounting for angle offsets:
    offset = [-0.003 -0.002 0 0 0 -1.571].';
    theta = thetas - offset;
    
    %vectors of other DH parameters
    d = [0 0 0 0 L4];
    a = [0 0 L2 -1*L3 0];
    alpha = [-1*pi/2 -1*pi/2 0 0 -1*pi/2];
    
    T_01 = get_TF_matrix(d(1), theta(1), a(1), alpha(1));
    T_12 = get_TF_matrix(d(2), theta(2), a(2), alpha(2));
    T_23 = get_TF_matrix(d(3), theta(3), a(3), alpha(3));
    T_34 = get_TF_matrix(d(4), theta(4)-pi/2, a(4), alpha(4));
    T_45 = get_TF_matrix(d(5), theta(5), a(5), alpha(5));
    
    gst_final = T_s0*T_01*T_12*T_23*T_34*T_45;
    
end

function T = get_TF_matrix(d, theta, a, alpha);
    %calculates the transformation matrix using the four DH params
    T = eye(4);
    T(1, 1:4) = [cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta)];
    T(2, 1:4) = [sin(theta), cos(alpha)*cos(theta), -1*sin(alpha)*cos(theta), a*sin(theta)];
    T(3, 1:4) = [0 sin(alpha) cos(alpha) d];
end
