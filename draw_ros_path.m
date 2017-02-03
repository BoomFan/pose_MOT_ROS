% draw_ros_path
%     sub_img = rossubscriber('/pose_estimate/image');
%     sub_str = rossubscriber('/pose_estimate/str');
%     pub = rospublisher('/pose_track/image','sensor_msgs/Image');
%     track_img = rosmessage('sensor_msgs/Image');
%     track_img.Encoding = 'rgb8';
%     writeImage(track_img,frame)
%     send(pub,track_img)
%     sub_odm = rossubscriber('/segway/odometry/local_filtered');
%     odm = receive(sub_odm)
    sub_traj = rossubscriber('/trajectory');
    traj = receive(sub_traj)
    for i=1:length(traj.Poses)
    trajectory_x(i)=(traj.Poses(i, 1).Pose.Position.X);
    trajectory_y(i)=(traj.Poses(i, 1).Pose.Position.Y);
    end
%     plot(traj.Poses(5, 1).Pose.Position.X,traj.Poses(5, 1).Pose.Position.Y) 
plot(trajectory_x,trajectory_y) 
%     end