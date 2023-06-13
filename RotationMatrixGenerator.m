function R = RotationMatrixGenerator(theta,phi,omega,order)

    Rx = [1      0           0;
          0 cos(theta) -sin(theta);
          0 sin(theta)  cos(theta)];

    Ry = [ cos(phi)  0   sin(phi);
              0      1      0
          -sin(phi)  0   cos(phi)];

    Rz = [cos(omega) -sin(omega) 0;
          sin(omega)  cos(omega) 0;
               0           0     1];

    R = eye(3);
    for i=1:length(order)
        if order(i) == 'X'
            R = Rx*R;
        elseif order(i) == 'Y'
            R = Ry*R;
        else
            R = Rz*R;
        end
    end

end