%%%%%%%% Instrctions of Our Codes %%%%%%%%
%% EECS442 %%
%% Fan Bu, Yingjie Cai, Yi Yang %%
%% University of Michigan %%

Please install Matlab Computer Vision Tool Box first.

To reproduce the result from our EECS442 project,
run "MOTwithKCF.m" for tracking with Faster RCNN and KCF;
run "fusedMOT.m" for tracking with Faster RCNN and Kalman Filter.

Before running, please change the video name you want you read and save through the following codes in those .m files: 
        obj.reader = vision.VideoFileReader('basketball.mp4');
        obj.writer = vision.VideoFileWriter('basketball_rcnn_KCF.avi');
Then, copy the three ".mat" files from the folder with the same name as the video.
For example, If you choose "obj.reader = vision.VideoFileReader('basketball.mp4');"
Then, copy "bboxes_rcnn.mat centroids_rcnn.mat filename_jpg.mat" from subfolder "basketball" to where the main function is.
Now you are ready to go!