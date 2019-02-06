% #   This source code is part of the localization and SLAM package
% #   deveoped for the lectures of probabilistic robotics at 
% #   Sapienza, University of Rome.
% #  
% #     Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti
% #  
% #   It is licences under the Common Creative License,
% #   Attribution-NonCommercial-ShareAlike 3.0
% #  
% #   You are free:
% #     - to Share - to copy, distribute and transmit the work
% #     - to Remix - to adapt the work
% #  
% #   Under the following conditions:
% #  
% #     - Attribution. You must attribute the work in the manner specified
% #       by the author or licensor (but not in any way that suggests that
% #       they endorse you or your use of the work).
% #    
% #     - Noncommercial. You may not use this work for commercial purposes.
% #    
% #     - Share Alike. If you alter, transform, or build upon this work,
% #       you may distribute the resulting work only under the same or
% #       similar license to this one.
% #  
% #   Any of the above conditions can be waived if you get permission
% #   from the copyright holder.  Nothing in this license impairs or
% #   restricts the author's moral rights.
% #  
% #   This software is distributed in the hope that it will be useful,
% #   but WITHOUT ANY WARRANTY; without even the implied 
% #   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% #   PURPOSE.
% #
%   
% # load a file.g2o file and returns the four structs of landmark, poses, transitions, observations

function [landmarks, poses, transitions, current_obs] = loadG2o(filepath)

	%%-----G2O specification---
	VERTEX_XY = 'VERTEX_XY';						%landmarks
	VERTEX_SE2 = 'VERTEX_SE2';						%poses
	EDGE_SE2 = 'EDGE_SE2';							%transitions
	EDGE_BEARING_SE2_XY = 'EDGE_BEARING_SE2_XY';	%observations (bearing only)

	%%-------------------------

	%open the file
	fid = fopen(filepath, 'r');
	

	%debug stuff (no such importance)
	i_landmarks = 0;
	i_poses = 0;
	i_transitions = 0;
	i_obs = 0;
  
	%    
	curr_id = -1;

	while true
		%get current line
		c_line = fgetl(fid);

		%stop if EOF
		if c_line == -1
			break;
		end

		%Split the line using space as separator
		elements = strsplit(c_line,' ');

		switch(elements{1})
			case VERTEX_XY
        			landmarks(end+1) = extractLandmark(elements);
				i_landmarks = i_landmarks + 1; %do not use pre/post increment. Keep the Matlab compatibility
			case VERTEX_SE2
        			poses(end+1) = extractPose(elements);
				i_poses = i_poses + 1;
			case EDGE_SE2
			      transitions(end+1) = extractTransition(elements);
				i_transitions = i_transitions + 1;
			case EDGE_BEARING_SE2_XY
				current_obs(end+1) = extractPoint(elements);					
				i_obs = i_obs + 1;

			otherwise
				disp('Error in reading first element');
		end
	end
  
  printf('[G2oWrapper] loading file...\n#landmarks: %d \n#poses: %d \n',i_landmarks, i_poses);
  printf('#transitions: %d \n#observation(bearing-only): %d \n',i_transitions, i_obs);	
  fflush(stdout);

end

function out = extractLandmark(elements)
  id = str2double(elements{2});
  x_pose = str2double(elements{3});
  y_pose = str2double(elements{4});
  out = landmark(id,x_pose,y_pose);
end

function out = extractPose(elements)
  id = str2double(elements{2});
  x_pose = str2double(elements{3});
  y_pose = str2double(elements{4});
  theta_pose = str2double(elements{5});
  out = pose(id,x_pose, y_pose, theta_pose);
end

function out = extractTransition(elements)
  from_id = str2double(elements{2});
  to_id = str2double(elements{3});
  x_t = str2double(elements{4});
  y_t = str2double(elements{5});
  theta_t = str2double(elements{6});
  out = transition(from_id, to_id, x_t, y_t, theta_t);
end

function out = extractPoint(elements)
  from_id = str2double(elements{2});
  land_id = str2double(elements{3});
  bearing = str2double(elements{4});
  out = observation(from_id, land_id, bearing);
end
