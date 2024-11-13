clear ros2node
node = ros2node("/matlab_node");
posPub = ros2publisher(node, '/drone/position', 'geometry_msgs/Point');
pause(0.5);
t = 0:0.1:10;
mass = 1.5; force = [0; 0; 14.7]; accel = force' / mass;
position = [10, 5, 100]; velocity = [1, 1, 0]; u = [0; 0; 0];
ekf = extendedKalmanFilter(@stateTransitionFcn, @measurementFcn, [position, velocity]);
dronePosition = zeros(length(t), 3); estimatedStateHistory = zeros(length(t), 6);
for i = 1:length(t)
    predict(ekf, u);
    measuredPosition = getROSPosition();
    estimatedState = correct(ekf, measuredPosition);
    dronePosition(i, :) = position;
    estimatedStateHistory(i, :) = estimatedState;
    position = position + velocity * 0.1;
    velocity = velocity + accel * 0.1;
    posMsg = ros2message('geometry_msgs/Point');
    [posMsg.x, posMsg.y, posMsg.z] = deal(position(1), position(2), position(3));
    send(posPub, posMsg);
end
% Plotting
figure; 
plot3(dronePosition(:,1), dronePosition(:,2), dronePosition(:,3), '-o');
xlabel('X'), ylabel('Y'), zlabel('Z'), grid on, title('Drone 3D Trajectory');
figure;
subplot(3,1,1), plot(t, dronePosition(:,1), t, estimatedStateHistory(:,1)), xlabel('Time'), ylabel('X');
subplot(3,1,2), plot(t, dronePosition(:,2), t, estimatedStateHistory(:,2)), xlabel('Time'), ylabel('Y');
subplot(3,1,3), plot(t, dronePosition(:,3), t, estimatedStateHistory(:,3)), xlabel('Time'), ylabel('Z');
figure;
subplot(3,1,1), plot(t, estimatedStateHistory(:, 4)), xlabel('Time'), ylabel('X Velocity');
subplot(3,1,2), plot(t, estimatedStateHistory(:, 5)), xlabel('Time'), ylabel('Y Velocity');
subplot(3,1,3), plot(t, estimatedStateHistory(:, 6)), xlabel('Time'), ylabel('Z Velocity');
function z = getROSPosition()
    persistent posSub nodeInitialized latestPosMsg
    if isempty(nodeInitialized)
        node = ros2node("/measurement_node");
        posSub = ros2subscriber(node, '/drone/position', 'geometry_msgs/Point');
        nodeInitialized = true;
    end
    try
        posMsg = receive(posSub, 1);
        latestPosMsg = posMsg;
    catch
    end
    if ~isempty(latestPosMsg)
        z = [latestPosMsg.x, latestPosMsg.y, latestPosMsg.z];
    else
        z = [0, 0, 0]; % Fallback if no message is received
    end
end
function xNext = stateTransitionFcn(x, u)
    dt = 0.1; 
    A = [eye(3), dt*eye(3); zeros(3), eye(3)];
    B = [zeros(3); eye(3)];
    xNext = A * x + B * u;
end
function z = measurementFcn(x)
    z = x(1:3);
end