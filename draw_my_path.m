
% draw_my_path
%     sub_img = rossubscriber('/pose_estimate/image');
%     sub_str = rossubscriber('/pose_estimate/str');
%     pub = rospublisher('/pose_track/image','sensor_msgs/Image');
%     track_img = rosmessage('sensor_msgs/Image');
%     track_img.Encoding = 'rgb8';
%     writeImage(track_img,frame)
%     send(pub,track_img)
%     sub_odm = rossubscriber('/segway/odometry/local_filtered');
%     odm = receive(sub_odm)
clear all
close all
    sub_traj = rossubscriber('/segway/pose');
    traj = receive(sub_traj);
    sub_map = rossubscriber('/map');
    map = receive(sub_map);
    myMap = readBinaryOccupancyGrid(map);
    hold on
    show(myMap);
    for i=1:length(traj.Cells)
    trajectory_x(i)=(traj.Cells(i, 1).X);
    trajectory_y(i)=(traj.Cells(i, 1).Y);
    end
% %     plot(traj.Poses(5, 1).Pose.Position.X,traj.Poses(5, 1).Pose.Position.Y) 
plot(trajectory_x,trajectory_y) 
% %     end