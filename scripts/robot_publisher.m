function robot_publisher()
  % Script to move Robot
  % Initialize ROS node
%   rosinit('localhost');

  % Create a publisher which can "talk" to Robot and tell it to move
  pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');

  % Create a Twist message and add linear x and angular z values
  move_cmd = rosmessage(pub);

  % Set publish rate at 10 Hz
  rate = rosrate(10);

  %%%%%%%%%% Move Straight %%%%%%%%%%
  disp("Moving Straight");
  move_cmd.Linear.X = 0.3;           % move in X axis at 0.3 m/s
  move_cmd.Angular.Z = 0.0;

  % For the next 3 seconds publish cmd_vel move commands
  now = rostime('now');
  while rostime('now') - now < rosduration(3)
      send(pub, move_cmd);          % publish to Robot
      waitfor(rate);
  end

  %%%%%%%%%% Rotating Counterclockwise %%%%%%%%%%
  disp("Rotating");
  move_cmd.Linear.X = 0.0;
  move_cmd.Angular.Z = 0.2;         % rotate at 0.2 rad/sec

  % For the next 3 seconds publish cmd_vel move commands
  now = rostime('now');
  while rostime('now') - now < rosduration(3)
      send(pub, move_cmd);          % publish to Robot
      waitfor(rate);
  end

  %%%%%%%%%% Stop %%%%%%%%%%
  disp("Stopping");
  move_cmd.Linear.X = 0.0;
  move_cmd.Angular.Z = 0.0;         % Giving both zero will stop the robot

  % For the next 1 seconds publish cmd_vel move commands
  now = rostime('now');
  while rostime('now') - now < rosduration(1)
      send(pub, move_cmd);          % publish to Robot
      waitfor(rate);
  end

  disp("Exit");

  % Shutdown ROS node
  rosshutdown;
end