function [t01,t02,t03,t04,t05,t06,t07,ja] = jacobian(dh)
     dh_parameters = dh;
   
     t01  = transformation(dh_parameters(1,1), dh_parameters(1,2), dh_parameters(1,3), dh_parameters(1,4));
     t02 = t01*transformation(dh_parameters(2,1), dh_parameters(2,2), dh_parameters(2,3), dh_parameters(2,4));
     t03 = t02*transformation(dh_parameters(3,1), dh_parameters(3,2), dh_parameters(3,3), dh_parameters(3,4));
     t04 = t03*transformation(dh_parameters(4,1), dh_parameters(4,2), dh_parameters(4,3), dh_parameters(4,4));
     t05 = t04*transformation(dh_parameters(5,1), dh_parameters(5,2), dh_parameters(5,3), dh_parameters(5,4));
     t06 = t05*transformation(dh_parameters(6,1), dh_parameters(6,2), dh_parameters(6,3), dh_parameters(6,4));
     t07 = t06*transformation(dh_parameters(7,1), dh_parameters(7,2), dh_parameters(7,3), dh_parameters(7,4));
     
    rotation_matrix = t07(1:3,1:3);
    w_1 = atan2(rotation_matrix(2,3),rotation_matrix(1,3));
    w_2 = atan2((sqrt(rotation_matrix(1,3)^2 + rotation_matrix(3,3)^2)),rotation_matrix(3,3));
    w_3 = atan2(rotation_matrix(3,2), - rotation_matrix(3,1));
    
    T = [0, -sin(w_1), cos(w_1)*sin(w_2);
           0, cos(w_1), sin(w_1)*sin(w_2);
           1,0,cos(w_2)];
   
    j = [cross([0;0;1],(t07(1:3,4)- [0;0;0])),     cross(t01(1:3,3),(t07(1:3,4)-t01(1:3,4))),   cross(t02(1:3,3),(t07(1:3,4)- t02(1:3,4))), cross(t03(1:3,3),(t07(1:3,4)- t03(1:3,4))), cross(t04(1:3,3),(t07(1:3,4)- t04(1:3,4))), cross(t05(1:3,3),(t07(1:3,4)- t05(1:3,4))), cross(t06(1:3,3),(t07(1:3,4)- t06(1:3,4)));
           [0;0;1]                           ,        t01(1:3,3)                          ,             t02(1:3,3)                     ,        t03(1:3,3),                                  t04(1:3,3),                            t05(1:3,3),                                   t06(1:3,3)     ];

    TA = [1 0 0 0 0 0;
          0 1 0 0 0 0;
          0 0 1 0 0 0;
          0 0 0 T(1,1) T(1,2) T(1,3);
          0 0 0 T(2,1) T(2,2) T(2,3);
          0 0 0 T(3,1) T(3,2) T(3,3)];
    
    ja = inv(TA)*(j);

end   