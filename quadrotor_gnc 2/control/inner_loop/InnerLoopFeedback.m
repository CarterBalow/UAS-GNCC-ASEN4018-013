function [Fc, Gc] = InnerLoopFeedback(var)


T1 = .5; %constraint
T2 = .05;
Ix = 5.8e-5;
Iy = 7.2e-5;

lambda1 = -1/T1;
lambda2 = -1/T2;

kp_roll = Ix*(-lambda1 * lambda2);
kd_roll = Ix*(lambda1 * lambda2);

kp_pitch = Iy*(-lambda1 * lambda2);
kd_pitch = Iy*(lambda1 * lambda2);
k_yaw = -.068*9.81*1e-4;





Gc = [kp_roll*var(4) - kd_roll*var(10); kp_pitch*var(5) - kd_pitch*var(11); k_yaw * var(12)];
Fc = [0; 0; .068*9.81];



end