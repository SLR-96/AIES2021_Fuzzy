%%%%%%%%%% Takes in the cartesian coordinates and converts them to the
%%%%%%%%%% three link angles
function theta=cart_pol(x, y, z)

L1=1.5; L2=0.8; L3=0.7;
theta=zeros(1,3);

theta(1) = atan2(y, x);

hyp = sqrt(x^2+y^2+(z-L1)^2);

phi = acos((L2^2+hyp^2-L3^2)/(2*L2*hyp));

theta(2) = asin((z-L1)/hyp)+phi;

theta(3) = acos((hyp^2-L2^2-L3^2)/(2*L2*L3));

end