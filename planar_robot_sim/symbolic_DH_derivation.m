close all
syms theta alpha a d
% this first formula is the one used by LASA's C++ code
DH_link(theta, alpha, a, d) = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);...
    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);...
    0, sin(alpha), cos(alpha), d;...
    0, 0, 0, 1];
% 
% DH_link2(theta, alpha, a, d) = [cos(theta), -sin(theta), 0, a;...
%     sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;...
%     sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d;...
%     0, 0, 0, 1];

num_links = 3;
Q = sym('q', [1 num_links]);
Alpha = sym('alpha', [1 num_links]);
A = sym('a', [1 num_links]);
D = sym('d', [1 7]);%num_links]);
all_args = [Q(1:num_links) Alpha(1:num_links) A(1:num_links) D(1:num_links)];
H = sym('FK', [4 4]);
for i = 1:size(H,1)
    for j = 1:size(H, 2)
        if i == j
            H(i,j) = sym(1);
        else
            H(i,j) = sym(0);
        end
    end
end
for i = 1:num_links
    H = H*DH_link(Q(i), Alpha(i), A(i), D(i));
end
H_pos= [H(1, 4); H(2,4); H(3, 4)];
J = jacobian(H_pos, transpose(Q(1:num_links)));
H_times_J = transpose(J)*H_pos;
H_planar = subs(H_pos, [Alpha(1:num_links) D(1:num_links)],zeros(1,num_links*2));
J_planar = subs(J, [Alpha(1:num_links) D(1:num_links)],zeros(1,num_links*2));
H_times_J_planar = transpose(subs(H_times_J, [Alpha(1:num_links) D(1:num_links)],zeros(1,num_links*2)));
KUKA_DH = sym('KDH', [7 4]);
KUKA_DH(1,1) = 0; KUKA_DH(1,2) = D(1); KUKA_DH(1,3) = pi/2;
KUKA_DH(2,1) = 0; KUKA_DH(2,2) = 0; KUKA_DH(2,3) = -pi/2;
KUKA_DH(3,1) = 0; KUKA_DH(3,2) = D(3); KUKA_DH(3,3) = -pi/2;
KUKA_DH(4,1) = 0; KUKA_DH(4,2) = 0; KUKA_DH(4,3) = pi/2;
KUKA_DH(5,1) = 0; KUKA_DH(5,2) = D(5); KUKA_DH(5,3) = pi/2;
KUKA_DH(6,1) = 0; KUKA_DH(6,2) = 0; KUKA_DH(6,3) = -pi/2;
KUKA_DH(7,1) = 0; KUKA_DH(7,2) = D(7); KUKA_DH(7,3) = 0;
assume([KUKA_DH Q], 'real');
H_pos_KUKA = simplify(subs(H, [A(1:num_links) D(1:num_links) Alpha(1:num_links)], ...
    transpose([KUKA_DH(1:num_links,1) ;KUKA_DH(1:num_links,2); KUKA_DH(1:num_links,3)])));
H_times_J_KUKA = simplify(subs(H_times_J, [A(1:num_links) D(1:num_links) Alpha(1:num_links)], ...
    transpose([KUKA_DH(1:num_links,1) ;KUKA_DH(1:num_links,2); KUKA_DH(1:num_links,3)])))
real(rewrite(H_pos_KUKA, 'exp'));

Xe = sym('xe_', [3 1]);
Xe(3) = sym(0);
H_times_J_frameshifted = transpose(J)*(H_pos -Xe)
H_times_J_planar_frameshifted = simplify(transpose(J_planar)*(H_planar -Xe))
H_times_J_KUKA_frameshifted = simplify(subs(H_times_J_frameshifted, [A(1:num_links) D(1:num_links) Alpha(1:num_links)], ...
    transpose([KUKA_DH(1:num_links,1) ;KUKA_DH(1:num_links,2); KUKA_DH(1:num_links,3)])))



% 	mSKinematicChain->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD( -85.), DEG2RAD( 85.), DEG2RAD(98.0)*0.90);
% 	mSKinematicChain->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -90.), DEG2RAD( 90.), DEG2RAD(98.0)*0.90);
% 	mSKinematicChain->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-100.), DEG2RAD(100.), DEG2RAD(100.0)*0.90);
% 	mSKinematicChain->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-110.), DEG2RAD(110.), DEG2RAD(130.0)*0.90);
% 	mSKinematicChain->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-140.), DEG2RAD(140.), DEG2RAD(140.0)*0.90);
% 	mSKinematicChain->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -90.), DEG2RAD( 90.), DEG2RAD(180.0)*0.90); // reduced joint angle to save the fingers
% 	mSKinematicChain->setDH(6, 0.0, 0.1260,    0.0, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)*0.90);
% 


