function points = quinticPathGenerator(data)
%quinticPathGenerator generates path points along a smooth Quintic 
%trajectory between via points within a given joint space.
%
%points = quinticPathGenerator(data) where data is the input array composed
%of joint segment information in the form: [T,Pi,Pf,Vi,Vf,Ai,Af; %segment 1
%                                           T,Pi,Pf,Vi,Vf,Ai,Af; %segment 2
%                                                    :
%                                           T,Pi,Pf,Vi,Vf,Ai,Af] %segment n
%
%and points is the output column vector of positions that satisfy the 
%segment constraints for the joint of interest

    %extracting number of segments for joint of interest
    [segments, ~] = size(data);

    %creating containers to hold quintic constant values for each segment
    a0 = zeros(1, segments);
    a1 = zeros(1, segments);
    a2 = zeros(1, segments);
    a3 = zeros(1, segments);
    a4 = zeros(1, segments);
    a5 = zeros(1, segments);
    
    %computing quintic constant values for each segment
    for i = 1:segments
        a0(i)=data(i,2);                                                    %a0 = Pi                                  
        a1(i)=data(i,4);                                                    %a1 = Vi   
        a2(i)=data(i,6)/2;                                                  %a2 = Ai/2   
        a3(i)= ((20*data(i,3))-(20*data(i,2))- (((8*data(i,5))...           %a3 = (20Pf-20Pi-(8Vf+12Vi)T-(3Ai-Af)T^2) / 2T^3
            +(12*data(i,4)))*data(i,1))-(((3*data(i,6))-(data(i,7)))...
            *data(i,1)*data(i,1)))/(2*data(i,1)*data(i,1)*data(i,1));
        a4(i)= ((30*data(i,2))-(30*data(i,3))+ (((14*data(i,5))...          %a4 = (30Pi-30Pf+(14Vf+16Vi)T+(3Ai-2Af)T^2) / 2T^4
            +(16*data(i,4)))*data(i,1))+(((3*data(i,6))-(2*data(i,7)))...
            *data(i,1)*data(i,1)))/(2*data(i,1)*data(i,1)*data(i,1)...
            *data(i,1));
        a5(i)= ((12*data(i,3))-(12*data(i,2))- (((6*data(i,5))...           %a5 = (12Pf-12Pi-(6Vf+6Vi)T-(Ai-Af)T^2) / 2T^5
            +(6*data(i,4)))*data(i,1))-(((data(i,6))-(data(i,7)))...
            *data(i,1)*data(i,1)))/(2*data(i,1)*data(i,1)*data(i,1)...
            *data(i,1)*data(i,1));
    end

    %creating container to hold points
    points = zeros(61, 1); %61 = 10*6 + 1
    
    %defining index tracking position in path
    currentpoint = 1;
    
    %evaluating joint segment polynomials at 10 intermitant time points
    for i=1:segments                                                        %for each segment
        segmentTime = data(i, 1);                                           %extracting time duration of current segment
        for t = 0:(segmentTime/10):segmentTime-(segmentTime/10)             %for 10 intermediate points     
            points(currentpoint) = a0(i)+ (a1(i)*t)+(a2(i)*t.^2)+(a3(i)...  %evaluate proper segment polynomial at intermediate point
                                    *t.^3)+(a4(i)*t.^4)+(a5(i)*t.^5);
            currentpoint = currentpoint + 1;                                %update index for point container
        end
    end
    
    %add final point (+1)
    points(currentpoint) = a0(6)+ (a1(6)*data(segments, 1))...
        +(a2(6)*data(segments, 1).^2)+(a3(6)*data(segments, 1).^3)...
        +(a4(6)*data(segments, 1).^4)+(a5(6)*data(segments, 1).^5);
end