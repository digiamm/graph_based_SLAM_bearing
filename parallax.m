function Xl = parallax(Xri, Xrj, bearingi, bearingj)

  x_i     = Xri(1);
  y_i     = Xri(2);
  theta_i = Xri(3);

  x_j     = Xrj(1);
  y_j     = Xrj(2);
  theta_j = Xrj(3);

  % get absolute angle of rotation
  abs_bearing_i = theta_i+bearingi;
  abs_bearing_j = theta_j+bearingj;


  % get trigonometric function
  s_i = sin(abs_bearing_i);
  s_j = sin(abs_bearing_j);

  c_i = cos(abs_bearing_i);
  c_j = cos(abs_bearing_j);

  % reconstruct landmark position
  x_hat = ((x_i*s_i*c_j)-(x_j*s_j*c_i)+(y_j-y_i)*c_i*c_j)/((s_i*c_j)-(s_j*c_i));
  y_hat = ((y_j*s_i*c_j)-(y_i*s_j*c_i)+(x_i-x_j)*s_i*s_j)/((s_i*c_j)-(s_j*c_i));

  Xl = [x_hat; y_hat];

endfunction;

function n_theta=normalizeAngle(theta)
 n_theta = atan2(sin(theta), cos(theta));
endfunction;
