% Test quaternion functions and operators
%% Scalar tests
p       = 3;
c       = complex( 1, 2 );
q0      = quaternion.rand;
qs      = quaternion.rand;
q       = quaternion
q       = quaternion( 0, 0, 0, 0 )
q       = quaternion( [1;2;3;4] )
q       = quaternion( [1 2 3 4] )
q1      = quaternion( 1, 2, 3, 4 )
q       = quaternion( [1;2;3] )
q       = quaternion( [1 2 3] )
q2      = quaternion( 1, 2, 3 )
q3      = quaternion( c )
n       = abs(q)
q       = conj(q)
d       = double(q)
l       = eq(q1,q2)
l       = equiv(q1,q2)
qe      = exp(q)
qi      = inverse(q)
q3      = ldivide(q1,q2)
q3      = ldivide(q1,c)
q3      = q1 .\ q2
ql      = log(q)
q3      = minus(q1,q2)
q3      = q1 - q2
q3      = q1 - c
qp      = mpower(qs,p)
qp      = qs^p
q3      = mtimes(qs,qs)
q3      = qs * qs
qp      = q^p
qp      = q^c
q3      = mtimes(q1,q2)
q3      = q1 * q2
q3      = q1 * c
l       = ne(q1,q1)
l       = ne(q1,q2)
n       = norm(q)
[q, n]  = normalize(q)
q3      = plus(q1,q2)
q3      = q1 + q2
q3      = q1 + c
qp      = power(q,p)
qp      = q.^p
qp      = q.^c
q3      = rdivide(q1,q2)
q3      = q1 ./ q2
q3      = q1 ./ c
q3      = times(q1,q2)
q3      = q1 .* q2
q3      = q1 .* c
qm      = uminus(q)
qm      = -q
qp      = uplus(q)
qp      = +q
%% Array tests
p       = 3;
t       = [0, 1/3, 2/3, 1];
c       = complex( rand(3,4), rand(3,4) );
q0      = quaternion.zeros(2)
q0      = quaternion.rand( [2, 2, 1, 2] );
q0(1)   = quaternion( zeros(4,1) );
q0(2)   = quaternion( ones(4,1) );
q1      = quaternion.ones
q3      = quaternion.rand( 3, 1 );
qe      = quaternion.eye(2)
qs      = quaternion.rand( 2, 2 );
qc      = quaternion(c)
qn      = quaternion.nan( 1, 2 );
n       = abs(q0)
q       = bsxfun(@plus,qc,q3)
q       = conj(q0)
q       = cumprod(q0)
q       = cumprod(q0,2)
q       = cumsum(q0)
q       = cumsum(q0,2)
q       = diff(q0)
q       = diff(q0,[],2)
q       = diff(q3,2)
d       = double(q0)
l       = eq(q0(1),q0)
l       = eq(q0,q0)
l       = equiv(-q0(1),q0)
l       = equiv(-q0,q0)
qe      = exp(q0)
qi      = inverse(q0)
l       = isequal(qc,qc,qc)
l       = isequal(qc,c)
l       = isequalwithequalnans(qn,qn)
q3      = ldivide(q1,q0)
q3      = q1 .\ q0
q3      = ldivide(q0,q0)
ql      = log(q0)
q3      = q0 .\ q0
q3      = minus(q1,q0)
q3      = q1 - q0
q3      = mpower(qs,p)
q3      = qs^p
q3      = mtimes(qs,qs)
q3      = qs * qs
q3      = minus(q0,q0)
q3      = q0 - q0
l       = ne(q0(1),q0)
l       = ne(q0,q0)
n       = norm(q0)
[q, n]  = normalize(q0)
q3      = plus(q1,q0)
q3      = q1 + q0
q3      = plus(q0,q0)
q3      = q0 + q0
qp      = power(q0,p)
qp      = q0.^p
q       = prod(q0)
q       = prod(q0,2)
q3      = rdivide(q1,q0)
q3      = q1 ./ q0
q3      = rdivide(q0,q0)
q3      = q0 ./ q0
q       = slerp(q0,q1)
q       = slerp(q0(3),q1,t)
q       = sqrt(q0)
q       = sum(q0)
q       = sum(q0,2)
q3      = times(q1,q0)
q3      = q1 .* q0
q3      = times(q0,q0)
q3      = q0 .* q0
qm      = uminus(q0)
qm      = -q0
%% Array constructor tests
angle0  = (0 : 30 : 330) * pi / 180;
axis0   = [repmat( [1;0;0], 1, 4 ), repmat( [0;1;0], 1, 4 ), repmat( [0;0;1], 1, 4 )];
cos0    = cos( angle0 );
sin0    = sin( angle0 );
R0a     = [cos0(3) -sin0(3) 0; sin0(3) cos0(3) 0; 0 0 1];
R0b     = [cos0(4) 0 sin0(4); 0 1 0; -sin0(4) 0 cos0(4)];
R0c     = [1 0 0; 0 cos0(6) -sin0(6); 0 sin0(6) cos0(6)];
R0      = cat( 3, R0a*R0b*R0c, R0b*R0c*R0a, R0c*R0a*R0b );
%
qa      = quaternion.angleaxis( angle0, axis0 )
qa      = quaternion.angleaxis( angle0(4), axis0 )
qa      = quaternion.angleaxis( angle0, axis0(:,1) )
qR0     = quaternion.rotationmatrix( R0 )
qR0a    = quaternion.rotationmatrix( R0a )
qR0b    = quaternion.rotationmatrix( R0b )
qR0c    = quaternion.rotationmatrix( R0c )
qR03    = quaternion.rotationmatrix( cat( 3, R0a, R0b, R0c ))
RotationMatrix( qR0a * qR0b * qR0c )
RotationMatrix( qR0b * qR0c * qR0a )
RotationMatrix( qR0c * qR0a * qR0b )
qEA     = quaternion.eulerangles( '123', [1, 2, 3] )
qEA     = quaternion.eulerangles( '123', [1; 2; 3] )
qEA     = quaternion.eulerangles( 'xyz', 1, 2, 3 )
qEA     = quaternion.eulerangles( ['121';'123'], [1,2,1;1,2,3] )
qEA     = quaternion.eulerangles( {'121','123'}, [1 1;2 2;1 3] )
qEA     = quaternion.eulerangles( ['iji';'ijk'], angle0(2:4) )
qEA     = quaternion.eulerangles( ['121';'123'], [1;1], [2;2], [1;3] )
qEA     = quaternion.eulerangles( {'121','123'}, [1 1], [2 2], [1 3] )
qEA     = quaternion.eulerangles( ['iji';'ijk'], angle0(2), angle0(3), angle0(4) )
EulerAngles( qEA, '121' ) * 180 / pi
EulerAngles( qEA, {'xyx','xyz'} ) * 180 / pi
%% Conversion tests
q21     = quaternion.rand(2,1);
q12     = quaternion.rand(1,2);
q22     = quaternion.rand(2,2);
q222    = quaternion.rand(2,2,2);
[an21, ax21]    = q21.AngleAxis;
[an12, ax12]    = q12.AngleAxis;
[an22, ax22]    = q22.AngleAxis;
[an222, ax222]  = q222.AngleAxis;
Q21     = quaternion.angleaxis( an21, ax21 );
Q12     = quaternion.angleaxis( an12, ax12 );
Q22     = quaternion.angleaxis( an22, ax22 );
Q222    = quaternion.angleaxis( an222, ax222 );
q21 - Q21
q12 - Q12
q22 - Q22
q222 - Q222
ea21    = q21.EulerAngles( '123' );
ea12    = q12.EulerAngles( '123' );
ea22    = q22.EulerAngles( '123' );
ea222   = q222.EulerAngles( '123' );
Q21     = quaternion.eulerangles( '123', ea21 );
Q12     = quaternion.eulerangles( '123', ea12 );
Q22     = quaternion.eulerangles( '123', ea22 );
Q222    = quaternion.eulerangles( '123', ea222 );
% q21 - Q21
q12 - Q12
q22 - Q22
q222 - Q222
R21     = q21.RotationMatrix;
R12     = q12.RotationMatrix;
R22     = q22.RotationMatrix;
R222    = q222.RotationMatrix;
Q21     = quaternion.rotationmatrix( R21 );
Q12     = quaternion.rotationmatrix( R12 );
Q22     = quaternion.rotationmatrix( R22 );
Q222    = quaternion.rotationmatrix( R222 );
q21 - Q21
q12 - Q12
q22 - Q22
q222 - Q222