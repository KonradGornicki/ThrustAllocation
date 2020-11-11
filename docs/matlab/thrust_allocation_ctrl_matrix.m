
%Moment arms
Ly= 0.2075; Lx=0.235;
% Control allocation matrix, B:
B = [1 1 0 0;0 0 1 1;Ly -Ly Lx -Lx]

%The Moore-Penrose pseudoinverse, B+
B_plus = pinv(B)
