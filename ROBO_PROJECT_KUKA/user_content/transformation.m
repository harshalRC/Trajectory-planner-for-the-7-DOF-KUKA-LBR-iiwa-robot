function trans = transformation(d, v, a, alp)
      trans = [cos(v) -sin(v)*cos(alp) sin(v)*sin(alp) a*cos(v);
               sin(v) cos(v)*cos(alp) -cos(v)*sin(alp) a*sin(v);
               0 sin(alp) cos(alp) d;
               0 0 0 1];
end
