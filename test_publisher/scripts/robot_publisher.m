function robot_publisher()
    % initialize the ros node
    rosinit('localhost');

    % create publisher which can talk to Robot and tell it to move
    pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');

    % create a twist message and add linear x and angular z values
    move_cmd = rosmessage(pub);

    % set publish rate at 10 Hz
    rate = rosrate(10);

    % -- move straight --
    disp('Moving straight');
    move_cmd.Linear.X = 0.3;
    move_cm.Angular.Z = 0.0;

    % for the next 3 seconds publish cmd_vel move commands
    now = rostime('now');
    while rostime('now') - now < rosduration(3)
        send(pub, move_cmd);
        waitfor(rate);
    end

    % -- rotating counterclockwise --
    disp('Rotating');
    move_cmd.Linear.X = 0.0;
    move_cmd.Angular.Z = 0.2;

    % for the next 3 seconds publish cmd_vel move commands
    now = rostime('now');
    while rostime('now') - now < rosduration(3)
        send(pub, move_cmd);
        waitfor(rate);
    end

    % -- stop --
    disp('Stopping');
    move_cmd.Linear.X = 0.0;
    move_cmd.Angular.Z = 0.0;

    now = rostime('now');
    while rostime('now') - now < rosduration(1)
        send(pub, move_cmd);
        waitfor(rate); 
    end

    disp('Exit');

    % shutdown ROS node
    rosshutdown;
end

