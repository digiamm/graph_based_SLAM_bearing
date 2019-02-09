% function XL_guess = reconstruct_landmark_position(observations, XR_guess)
%   global id_poses;
%   global poses_id;
%   % epselon = 0.2; %20 deg
%   % epselon = 1.70; %90 deg
%   epselon = pi/2;
%   XL_guess = zeros(3, 0);
%   n = 1;
%   id_to_state_map = ones(300, 1)*-1;
%
%   for zi = 1:length(observations)
%
%     pose_idi = observations(zi).pose_id;
%     land_idi = observations(zi).land_id;
%     bearingi = observations(zi).bearing;
%
%     if(land_idi != 0)
%       if(id_to_state_map(land_idi) == -1)
%
%         for zj = 1:length(observations)
%           pose_idj = observations(zj).pose_id;
%           land_idj = observations(zj).land_id;
%           bearingj = observations(zj).bearing;
%
%           if(land_idi==land_idj)
%
%             Xri = t2v(XR_guess(:,:, poses_id(pose_idi)));
%             Xrj = t2v(XR_guess(:,:, poses_id(pose_idj)));
%
%             relative_pose = inv(XR_guess(:,:, poses_id(pose_idi))) * XR_guess(:,:, poses_id(pose_idj));
%             if (norm(t2v(relative_pose)(1:2)) > 0.5)
%
%             % if (Xrj(1) > (Xri(1) + 2) || Xrj(2) > (Xri(2) + 2))
%
%               Xl = parallax(Xri, Xrj, bearingi, bearingj);
%
%               id_to_state_map(land_idi) = n++;
%
%               % if the landmark is an outlier, don't push it to the list
%               if (abs(Xl(1)) > 20 || abs(Xl(2)) > 20)
%                 break;
%               endif
%               XL_guess(:, end+1) = [land_idi; Xl(1); Xl(2)];
%               % endif
%
%               printf("Adding landmark %i -- x: %f y: %f\n", land_idi, Xl(1), Xl(2));
%               % pause(0.6);
%               break;
%             endif
%           endif
%
%         endfor
%       endif
%
%     endif
%   endfor
%
%
%   printf("Number of landmarks reconstructed %i\n", n);
% endfunction;


% INTERSECTION OF LINES METHOD, UNCOMMENT THIS TO RECONSTRUCT LAND POSES IN THIS WAY
function XL_guess = reconstruct_landmark_position(observations, XR_guess)
  global id_poses;
  global poses_id;
  XL_guess = zeros(3, 0);
  n = 1;
  id_to_state_map = ones(300, 1)*-1;

  for zi = 1:length(observations)

    landmark_list = zeros(0, 2);

    pose_idi = observations(zi).pose_id;
    land_idi = observations(zi).land_id;
    bearingi = observations(zi).bearing;

    if(land_idi != 0)
      if(id_to_state_map(land_idi) == -1)

        for zj = 1:length(observations)
          pose_idj = observations(zj).pose_id;
          land_idj = observations(zj).land_id;
          bearingj = observations(zj).bearing;

          if(land_idi==land_idj)
            landmark_list(end+1, :) = [pose_idj, bearingj];
          endif
        endfor

        if(size(landmark_list, 1) > 7)
          Xl = solver(landmark_list, XR_guess);

          id_to_state_map(land_idi) = n++;
          % if (abs(Xl(1)) > 20 || abs(Xl(2)) > 20)
          %   break;
          % endif
          XL_guess(:, end+1) = [land_idi; Xl(1); Xl(2)];

          printf("Adding landmark %i -- x: %f y: %f\n", land_idi, Xl(1), Xl(2));
        endif
      endif
    endif

  endfor

  printf("Number of landmarks reconstructed %i\n", n);
endfunction;
