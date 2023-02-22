%%%%%%%%%% Generates the output of the fuzzy system, using errors as the
%%%%%%%%%% input, and taking other input parameters like the output range,
%%%%%%%%%% vector of activated MFs, output of rules, and output MFs
function tau = output_generator(out_range, mfs, a, k_in, e, out1, out2, out3)

out_length = length(out_range);
% Preallocating
output1 = zeros(1, out_length);
output2 = output1;
output3 = output1;
j=1; k=1; l=1;

for i=1:9
    in1 = e.e1(mfs.e1_active(i))*k_in(1); % Checking the MF activated in each rule
    in2 = e.e2(mfs.e2_active(i))*k_in(2);
    if in1*in2~=0 % Not calculating the inactive MFs
        output1(j, :) = min(out1(a(i),:), (in1*in2));
        j=j+1;
    end
    in3 = e.e3(mfs.e3_active(i+9))*k_in(3); % Checking the MF activated in each rule
    in4 = e.e4(mfs.e4_active(i+9))*k_in(4);
    if in3*in4~=0 % Not calculating the inactive MFs
        output2(k, :) = min(out2(a(i+9),:), (in3*in4));
        k=k+1;
    end
    in5 = e.e5(mfs.e5_active(i+18))*k_in(5); % Checking the MF activated in each rule
    in6 = e.e6(mfs.e6_active(i+18))*k_in(6);
    if in5*in6~=0 % Not calculating the inactive MFs
        output3(l, :) = min(out3(a(i+18),:), (in5*in6));
        l=l+1;
    end
end


% Preallocating
aggregated1 = zeros(1, out_length);
aggregated2 = aggregated1;
aggregated3 = aggregated1;

% Aggregating the outputs
for i=1:out_length
    aggregated1(i) = max(output1(:,i));
    aggregated2(i) = max(output2(:,i));
    aggregated3(i) = max(output3(:,i));
end

tau = zeros(1,3);
% Defuzzification by centroid method
dA1 = 0.01*(aggregated1);
xdA1 = (out_range).*dA1;
tau(1) = sum(xdA1)/sum(dA1);

dA2 = 0.01*(aggregated2);
xdA2 = (out_range).*dA2;
tau(2) = sum(xdA2)/sum(dA2);

dA3 = 0.01*(aggregated3);
xdA3 = (out_range).*dA3;
tau(3) = sum(xdA3)/sum(dA3);
end
