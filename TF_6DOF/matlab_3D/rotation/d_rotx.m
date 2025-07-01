function Rx = d_rotx(a)
    Rx = [0, 0, 0;
          0, -sin(a), -cos(a);
          0, cos(a), -sin(a)];
end