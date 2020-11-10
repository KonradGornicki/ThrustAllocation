
Ly= 0.2075; Lx=0.235;

A = [1 1 0 0;0 0 1 1;Ly -Ly Lx -Lx]

A_inv = pinv(A)

% To get moment arms Ly and Lx
% % _a=1.0556,_b=1.1955;
% a= 1.0556;
% b= 1.1955;
% 
% A_prime = [0.5 0 a;0.5 0 -a; 0 0.5 b; 0 0.5 -b]
% 
% A_prime_inv = pinv(A_prime)