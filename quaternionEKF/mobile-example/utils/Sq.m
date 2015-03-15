function S=Sq(q)
% The matrix S(q) defined in (13.11c)
   q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
   S=[-q1 -q2 -q3;
       q0 -q3  q2;
       q3  q0 -q1;
      -q2  q1  q0];
end
