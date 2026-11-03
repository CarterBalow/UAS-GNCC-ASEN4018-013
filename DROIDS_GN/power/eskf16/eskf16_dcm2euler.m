function e = eskf16_dcm2euler(Q)
% Q_{N/B} -> 3-2-1 Euler [phi; theta; psi]
e = [atan2(Q(3,2), Q(3,3)); -asin(max(min(Q(3,1), 1), -1)); atan2(Q(2,1), Q(1,1))];
end
