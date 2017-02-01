  % Predicted locations of objects in the current frame.
  % Predictions can be obtained, for example, by using 
  % vision.KalmanFilter.
   predictions = [1,1; 2,2];
  
  % Locations of actual objects detected in the current frame.
  % Note that there are currently 2 tracks, and 3 new detections.
  % At least one of the detections would be unmatched, 
  % meaning that it may be a brand new track.
  detections = [1.1, 1.1; 2.1, 2.1; 1.5, 3];
  
  % pre-allocate a cost matrix
  numPredictions = size(predictions, 1);
  numDetections = size(detections, 1);
  cost = zeros(numPredictions, numDetections);
  
  % for each prediction, compute the cost of matching each detection.
  for i = 1:numPredictions
      % the cost is defined as the Euclidean distance between the 
      % prediction and the detection
      diff = detections - repmat(predictions(i, :), [numDetections, 1]);
      cost(i, :) = sqrt(sum(diff .^ 2, 2));
  end
  
  % assign detections to predictions
  % detection 1 should match to track 1, detection 2 to track 2, and
  % detection 3 should be unmatched.
  [assignment, unassignedTracks, unassignedDetections] = ...
      assignDetectionsToTracks(cost, 0.2);
 
  figure;
  plot(predictions(:, 1), predictions(:, 2), '*', ...
     detections(:, 1), detections(:, 2), 'ro');
  hold on;
  legend('predictions', 'detections');
  for i = 1:size(assignment, 1)
    text(predictions(assignment(i, 1), 1)+0.1, ...
        predictions(assignment(i, 1), 2)-0.1, num2str(i));
    text(detections(assignment(i, 2), 1)+0.1, ...
        detections(assignment(i, 2), 2)-0.1, num2str(i));
  end
  for i = 1:length(unassignedDetections)
    text(detections(unassignedDetections(i), 1)+0.1, ...
        detections(unassignedDetections(i), 2)+0.1, 'unassigned');
  end
  xlim([0, 4]);
  ylim([0, 4]);