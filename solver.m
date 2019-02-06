% this function requires a landmark_list (each list store bearings measurament for the specific landamark)
% and the robot poses. It reconstructs the landmark pose based on the intersection of each bearing measurament
% of the landmark
function Xl = solver(landmark_list, XR_guess)

  global id_poses;
  global poses_id;

  A = zeros(2,2);
  b = zeros(2,1);

  for b=1:length(landmark_list)
    curr_pose = landmark_list(b, 1);

    Xr = t2v(XR_guess(:,:, poses_id(curr_pose)));

    x = Xr(1);
    y = Xr(2);
    p = [x; y];
    theta = Xr(3);

    phi = landmark_list(b, 2);

    n = [cos(phi+theta); sin(phi+theta)];

    A += (n * n' - eye(2));
    b += (n * n' - eye(2))*p;

  endfor

  Xl = A\b;

endfunction;
