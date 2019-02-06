0;
% computes the pose 2d pose vector v from an homogeneous transform A
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function v=t2v(A)
	v(1:2, 1)=A(1:2,3);
	v(3,1)=atan2(A(2,1),A(1,1));
end

% computes the homogeneous transform matrix A of the pose vector v
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function A=v2t(v)
  	c=cos(v(3));
  	s=sin(v(3));
	A=[c, -s, v(1) ;
	s,  c, v(2) ;
	0   0  1  ];
end


function v=flattenIsometryByColumns(T)
v=zeros(6,1);
v(1:4)=reshape(T(1:2,1:2),4,1);
v(5:6)=T(1:2,3);
endfunction



#derivative of rotation matrix w.r.t rotation around z, in 0
global  R0=[0 -1;
			 1 0];
