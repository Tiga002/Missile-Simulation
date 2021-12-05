function log_trim_conditions(log_file_name, x_trim, u_trim, y_trim, mach_trim, h_trim)

UNIT_RAD2DEG = 180 / pi ;                              	                    % Radian to Degree

fileID = fopen(log_file_name, 'w');
fprintf(fileID, '===========================================');
fprintf(fileID, '           Trim Flight Conditions          ');
fprintf(fileID, '===========================================');
fprintf(fileID, '\n');
fprintf(fileID, ' Mach Number = %3.2f \n', mach_trim);
fprintf(fileID, ' Altitude = %3.2f    \n', h_trim);
fprintf(fileID, '\n');

fprintf(fileID, '===========================================');
fprintf(fileID, '           Trim States and Inputs         ');
fprintf(fileID, '===========================================');
fprintf(fileID, '\n');
fprintf(fileID, ' U      = %3.4f m/s   \n ', x_trim(1));
fprintf(fileID, ' V      = %3.4f m/s   \n ', x_trim(2));
fprintf(fileID, ' W      = %3.4f m/s   \n ', x_trim(3));
fprintf(fileID, ' P      = %3.4f deg/s \n ', x_trim(4) * UNIT_RAD2DEG);
fprintf(fileID, ' R      = %3.4f deg/s \n ', x_trim(5) * UNIT_RAD2DEG);
fprintf(fileID, ' Q      = %3.4f deg/s \n ', x_trim(6) * UNIT_RAD2DEG);
fprintf(fileID, ' PHI    = %3.4f deg/s \n ', x_trim(7) * UNIT_RAD2DEG);
fprintf(fileID, ' THETA    = %3.4f deg/s \n ', x_trim(8) * UNIT_RAD2DEG);
fprintf(fileID, ' PSI    = %3.4f deg/s \n ', x_trim(9) * UNIT_RAD2DEG);
fprintf(fileID, ' ALPHA    = %3.4f deg \n ', y_trim(10) * UNIT_RAD2DEG);
fprintf(fileID, ' BETA    = %3.4f deg \n ', y_trim(11) * UNIT_RAD2DEG);
fprintf(fileID, ' V_air_speed    = %3.4f m/s \n ', y_trim(12));

fprintf(fileID, '\n');

fprintf(fileID,'Thrust  = %3.4f N \n ', u_trim(1));
fprintf(fileID,'Elevator = %3.4f N \n ', u_trim(2) * UNIT_RAD2DEG);
fprintf(fileID,'Aileron  = %3.4f N \n ', u_trim(3) * UNIT_RAD2DEG);
fprintf(fileID,'Rudder  = %3.4f N \n ', u_trim(4) * UNIT_RAD2DEG);
end