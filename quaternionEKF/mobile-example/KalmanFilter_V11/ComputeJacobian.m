function [J] = ComputeJacobian(a,b,c,d,Ax,Ay,Az,Mx,My,Mz)
%Compute the quaternion Jacobian

J11=(2*a*Ax+2*b*Ay+2*c*Az);
J12=(-2*b*Ax+2*a*Ay+2*d*Az);
J13=(-2*c*Ax-2*d*Ay+2*a*Az);
J14=(2*d*Ax-2*c*Ay+2*b*Az);

J21=(2*b*Ax-2*a*Ay-2*d*Az);
J22=(2*a*Ax+2*b*Ay+2*c*Az);
J23=(2*d*Ax-2*c*Ay+2*b*Az);
J24=(2*c*Ax+2*d*Ay-2*a*Az);

J31=(2*c*Ax+2*d*Ay-2*a*Az);
J32=(-2*d*Ax+2*c*Ay-2*b*Az);
J33=(2*a*Ax+2*b*Ay+2*c*Az);
J34=(-2*b*Ax+2*a*Ay+2*d*Az);

J41=(2*a*Mx+2*b*My+2*c*Mz);
J42=(-2*b*Mx+2*a*My+2*Mz*d);
J43=(-2*c*Mx-2*d*My+2*a*Mz);
J44=(2*d*Mx-2*c*My+2*b*Mz);

J51=(2*b*Mx-2*a*My-2*d*Mz);
J52=(2*a*Mx+2*b*My+2*c*Mz);
J53=(2*d*Mx-2*c*My+2*b*Mz);
J54=(2*c*Mx+2*d*My-2*a*Mz);

J61=(2*c*Mx+2*d*My-2*a*Mz);
J62=(-2*d*Mx+2*c*My-2*b*Mz);
J63=(2*a*Mx+2*b*My+2*c*Mz);
J64=(-2*b*Mx+2*a*My+2*d*Mz);

J=-[J11 J12 J13 J14;J21 J22 J23 J24;J31 J32 J33 J34;J41 J42 J43 J44;J51 J52 J53 J54;J61 J62 J63 J64];

end

