function [assignments, unassignedTracks, unassignedDetections] = ...
    myOverlapAssignment(overlap_ratio,costOfoverlap_ratio,nTracks,nDetections)
% costOfoverlap_ratio = 0.5;
% overlap_ratio =[0    0.7148         0         0
%     0.6         0    0.5538         0
%     0.7801         0         0         0
%     0         0         0         0
%     0         0         0    0];
% nTracks =5;
% nDetections =4;



assignments = [];
unassignedTracks = [];
unassignedDetections = [];
if nTracks~=0
    if nDetections~=0
        for j=1:nDetections
            Ind = find(overlap_ratio(:,j) > costOfoverlap_ratio);
            if length(Ind)==1 %State: Tracked
                assignments = [assignments;Ind,j];
            else if length(Ind)>1 %State: Occluded
                    unassignedTracks = [unassignedTracks;Ind];
                else
                    unassignedDetections = [unassignedDetections;j];
                end
            end
        end
        
        for i=1:nTracks
           Ind = find(overlap_ratio(i,:) > costOfoverlap_ratio);
           if length(Ind)== 0
               unassignedTracks = [unassignedTracks;i];
           end
        end
        
    else
    end
    
else
    for j=1:nDetections
        unassignedDetections = [unassignedDetections;j];
    end
end

end


% assignments =
% 
%            3           1
%            1           2
%            2           3
%            5           4
%            
%            
% 
% unassignedTracks = 
%            4
% 
% unassignedDetections =[]
