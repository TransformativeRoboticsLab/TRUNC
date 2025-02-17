classdef trunc_model < handle
    properties
        theta_1 sym
        theta_2 sym
        theta_3 sym
        theta_4 sym
        theta_5 sym
        theta_6 sym
        L_0
        L sym
        dl
        d_tool_n

        % Symbolic
        T_shoulder sym
        T_elbow sym
        T_wrist sym
        T_tool sym
        
        % Numeric
        T_shoulder_n
        T_elbow_n
        T_wrist_n
        T_tool_n

        % Triads
        tri_origin
        tri_shoulder
        tri_elbow
        tri_wrist

        % Cable state
        comp_segments
        segments
        delta_segments
        max_delta 
        alpha

    end
    
    methods
        function obj = trunc_model()
            % Initialize symbolic variables
            syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 L real
            obj.theta_1 = theta_1;
            obj.theta_2 = theta_2;
            obj.theta_3 = theta_3;
            obj.theta_4 = theta_4;
            obj.theta_5 = theta_5;
            obj.theta_6 = theta_6;
            obj.L_0 = 710;
            obj.L = L;
            obj.d_tool_n = 83;
            obj.comp_segments = load('./state/model_comp.mat').segments;
            obj.max_delta = -20;
            obj.alpha = -0.4;

            % Symbolic forward kinematics
            T_shoulder_local = obj.segment_transform(obj.theta_1, obj.theta_2, -3*obj.L/7);
            obj.T_shoulder = T_shoulder_local;
            
            T_elbow_local = obj.segment_transform(obj.theta_3, obj.theta_4, -2*obj.L/7);
            obj.T_elbow = obj.T_shoulder * T_elbow_local;
            
            T_wrist_local = obj.segment_transform(obj.theta_5, obj.theta_6, -2*obj.L/7);
            obj.T_wrist = obj.T_elbow * T_wrist_local;
            
            T_tool_local = obj.segment_transform(0, 0, -obj.d_tool_n);
            obj.T_tool = obj.T_wrist * T_tool_local;
        end
        
        % Spherical joint with pristimatic joint
        function T = segment_transform(~,theta_1,theta_2,d)
            Rx = [1,0,0,0;
                  0,cos(theta_1),-sin(theta_1),0;
                  0,sin(theta_1),cos(theta_1),0;
                  0,0,0,1];
            Ry = [cos(theta_2),0,sin(theta_2),0;
                  0,1,0,0;
                  -sin(theta_2),0,cos(theta_2),0;
                  0,0,0,1];
            Rz = [cos(theta_2),-sin(theta_2),0,0;
                  sin(theta_2),cos(theta_2),0,0;
                  0,0,1,0;
                  0,0,0,1];
            Rz_m = [cos(-theta_2),-sin(-theta_2),0,0;
                  sin(-theta_2),cos(-theta_2),0,0;
                  0,0,1,0;
                  0,0,0,1];
            Tz = [1,0,0,0;
                  0,1,0,0;
                  0,0,1,d;
                  0,0,0,1];
            T = Rz_m*Tz*Rx*Rz;
       end
        
        function update_config(obj, theta_vals, dl)
            % Unpack theta values
            theta_1_n = deg2rad(theta_vals(1));
            theta_2_n = deg2rad(theta_vals(2));
            theta_3_n = deg2rad(theta_vals(3));
            theta_4_n = deg2rad(theta_vals(4));
            theta_5_n = deg2rad(theta_vals(5));
            theta_6_n = deg2rad(theta_vals(6));
            L_n = obj.L_0 + dl;
            obj.dl = dl;

            obj.T_shoulder_n = double(subs(obj.T_shoulder,[obj.theta_1, obj.theta_2, obj.theta_3, obj.theta_4, obj.theta_5, obj.theta_6, obj.L],...
                                [theta_1_n, theta_2_n, theta_3_n, theta_4_n, theta_5_n, theta_6_n, L_n]));
            obj.T_elbow_n = double(subs(obj.T_elbow,[obj.theta_1, obj.theta_2, obj.theta_3, obj.theta_4, obj.theta_5, obj.theta_6, obj.L],...
                                [theta_1_n, theta_2_n, theta_3_n, theta_4_n, theta_5_n, theta_6_n, L_n]));
            obj.T_wrist_n = double(subs(obj.T_wrist,[obj.theta_1, obj.theta_2, obj.theta_3, obj.theta_4, obj.theta_5, obj.theta_6, obj.L],...
                                [theta_1_n, theta_2_n, theta_3_n, theta_4_n, theta_5_n, theta_6_n, L_n]));
            obj.T_tool_n = double(subs(obj.T_tool,[obj.theta_1, obj.theta_2, obj.theta_3, obj.theta_4, obj.theta_5, obj.theta_6, obj.L],...
                                [theta_1_n, theta_2_n, theta_3_n, theta_4_n, theta_5_n, theta_6_n, L_n]));
        end
        
        function [segments,delta_segments] = find_segments(obj)
            % Plot triangles
            obj.tri_origin = [0,65.*sin(pi/3),-65.*sin(pi/3),0,;
                        65,-65.*cos(pi/3),-65.*cos(pi/3),65;
                        0,0,0,0;
                        1,1,1,1];
            obj.tri_shoulder = obj.T_shoulder_n*obj.tri_origin;
            obj.tri_elbow = obj.T_elbow_n*obj.tri_origin;
            obj.tri_wrist = obj.T_wrist_n*obj.tri_origin;

            % Find segment lengths
            s_shoulder = sqrt(sum((obj.tri_shoulder(1:3,1:3)-obj.tri_origin(1:3,1:3)).^2,1));
            s_elbow = sqrt(sum((obj.tri_elbow(1:3,1:3)-obj.tri_shoulder(1:3,1:3)).^2,1));
            s_wrist = sqrt(sum((obj.tri_wrist(1:3,1:3)-obj.tri_elbow(1:3,1:3)).^2,1));
            
            s1_s = s_shoulder(1); s2_s = s_shoulder(2); s3_s = s_shoulder(3);
            s1_e = s_elbow(1); s2_e = s_elbow(2); s3_e = s_elbow(3);
            s1_w = s_wrist(1); s2_w = s_wrist(2); s3_w = s_wrist(3);
            segments = [s1_w,s1_e,s1_s,s2_w,s2_e,s2_s,s3_w,s3_e,s3_s];
            delta_segments = segments - obj.comp_segments;
            obj.segments = segments;
            obj.delta_segments = delta_segments;
        end

        function [delta_segments] = find_adjusted_segments(obj)
            obj.find_segments();
            shoulder = obj.delta_segments(3:3:end);
            % Limit relative rotations and ensure
            shoulder = min(shoulder) + obj.alpha.*(min(shoulder)-shoulder);

            elbow = obj.delta_segments(2:3:end);
            % Limit relative rotations and ensure 
            elbow = min(elbow) + max(obj.alpha.*(min(elbow)-elbow),obj.max_delta);

            wrist = obj.delta_segments(1:3:end);
            wrist = min(wrist) + max(obj.alpha.*(min(wrist)-wrist),obj.max_delta);

            delta_segments = [wrist(1),elbow(1),shoulder(1),...
                              wrist(2),elbow(2),shoulder(2),...
                              wrist(3),elbow(3),shoulder(3)];
            obj.delta_segments = delta_segments;

        end

        function [delta_lengths] = find_lengths(obj)
            
            obj.find_adjusted_segments();
            
            % Extract each joint
            dl_wrist = obj.delta_segments(1:3:end);
            dl_elbow = obj.delta_segments(2:3:end);
            dl_shoulder = obj.delta_segments(3:3:end);
            
            dl_offset = (max(abs(dl_wrist)) + max(abs(dl_elbow)) + max(abs(dl_shoulder)))/8
            dl = obj.dl + dl_offset;
            
            % Summing segments into lengths
            dl_elbow = dl_elbow + dl_shoulder;
            dl_wrist = dl_wrist + 0.75.*dl_elbow;
            
            % Add in compression values
            dl_wrist = dl_wrist + dl;
            dl_elbow = dl_elbow + (5/7).*dl;
            dl_shoulder = dl_shoulder + (3/7).*dl;
            
            delta_lengths = [dl_wrist(1),dl_elbow(1),dl_shoulder(1),...
                            dl_wrist(2),dl_elbow(2),dl_shoulder(2),...
                            dl_wrist(3),dl_elbow(3),dl_shoulder(3)];

        
        end

        % Helper function for drawing arm
        function draw_arm(obj)
            % Plot of the arm
            figure(1); clf; hold on
            set(gcf, 'Color', 'white');
        
            segments = obj.find_segments();

            plot3(obj.tri_origin(1,:),obj.tri_origin(2,:),obj.tri_origin(3,:),'Color',[0.5,0.5,0.5,0.5],'LineWidth',3)
            plot3(obj.tri_shoulder(1,:),obj.tri_shoulder(2,:),obj.tri_shoulder(3,:),'Color',[0.5,0.5,0.5,0.5],'LineWidth',3)
            plot3(obj.tri_elbow(1,:),obj.tri_elbow(2,:),obj.tri_elbow(3,:),'Color',[0.5,0.5,0.5,0.5],'LineWidth',3)
            plot3(obj.tri_wrist(1,:),obj.tri_wrist(2,:),obj.tri_wrist(3,:),'Color',[0.5,0.5,0.5,0.5],'LineWidth',3)
        
            % Plot of cables
            plot3([obj.tri_origin(1,1),obj.tri_shoulder(1,1),obj.tri_elbow(1,1),obj.tri_wrist(1,1)],...
                  [obj.tri_origin(2,1),obj.tri_shoulder(2,1),obj.tri_elbow(2,1),obj.tri_wrist(2,1)],...
                  [obj.tri_origin(3,1),obj.tri_shoulder(3,1),obj.tri_elbow(3,1),obj.tri_wrist(3,1)],'Color','k','LineWidth',2)
            plot3([obj.tri_origin(1,2),obj.tri_shoulder(1,2),obj.tri_elbow(1,2),obj.tri_wrist(1,2)],...
                  [obj.tri_origin(2,2),obj.tri_shoulder(2,2),obj.tri_elbow(2,2),obj.tri_wrist(2,2)],...
                  [obj.tri_origin(3,2),obj.tri_shoulder(3,2),obj.tri_elbow(3,2),obj.tri_wrist(3,2)],'Color','k','LineWidth',2)
            plot3([obj.tri_origin(1,3),obj.tri_shoulder(1,3),obj.tri_elbow(1,3),obj.tri_wrist(1,3)],...
                  [obj.tri_origin(2,3),obj.tri_shoulder(2,3),obj.tri_elbow(2,3),obj.tri_wrist(2,3)],...
                  [obj.tri_origin(3,3),obj.tri_shoulder(3,3),obj.tri_elbow(3,3),obj.tri_wrist(3,3)],'Color','k','LineWidth',2)
            
            % Plot joints
            r = 30;
            [X, Y, Z] = sphere(100);
            X = r * X; Y = r * Y; Z = r * Z;
            
            surf(X+obj.T_shoulder_n(1,4), Y+obj.T_shoulder_n(2,4), Z+obj.T_shoulder_n(3,4),'FaceColor','r','EdgeColor','none');
            surf(X+obj.T_elbow_n(1,4), Y+obj.T_elbow_n(2,4), Z+obj.T_elbow_n(3,4),'FaceColor','r','EdgeColor','none');
            surf(X+obj.T_wrist_n(1,4), Y+obj.T_wrist_n(2,4), Z+obj.T_wrist_n(3,4),'FaceColor','r','EdgeColor','none');
            
            % Plot of links
            plot3([0,obj.T_shoulder_n(1,4)],[0,obj.T_shoulder_n(2,4)],[0,obj.T_shoulder_n(3,4)],'LineWidth',4,'Color','k')
            plot3([obj.T_shoulder_n(1,4),obj.T_elbow_n(1,4)],[obj.T_shoulder_n(2,4),obj.T_elbow_n(2,4)],[obj.T_shoulder_n(3,4),obj.T_elbow_n(3,4)],'LineWidth',4,'Color','k')
            plot3([obj.T_elbow_n(1,4),obj.T_wrist_n(1,4)],[obj.T_elbow_n(2,4),obj.T_wrist_n(2,4)],[obj.T_elbow_n(3,4),obj.T_wrist_n(3,4)],'LineWidth',4,'Color','k')
            plot3([obj.T_wrist_n(1,4),obj.T_tool_n(1,4)],[obj.T_wrist_n(2,4),obj.T_tool_n(2,4)],[obj.T_wrist_n(3,4),obj.T_tool_n(3,4)],'LineWidth',4,'Color','k')
            
            % Visualize the origin triad
            hold on; % Keep the current plot
            quiver3(0, 0, 0, 100, 0, 0, 'r',LineWidth=3); % X-axis in red
            quiver3(0, 0, 0, 0, 100, 0, 'b',LineWidth=3); % Y-axis in green
            quiver3(0, 0, 0, 0, 0, 100, 'g',LineWidth=3); % Z-axis in blue
            hold off;
            
            % Adjust the camera view
            view(30, 30); % Set the azimuth and elevation
            
            % Other plot settings
            axis equal; % Equal scaling for all axes
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            
            % Add lighting to enhance 3D effect
            camlight left;
            lighting phong;
            grid on;
            
            xlim([-500, 500])
            ylim([-500, 500])
            zlim([-800 100])
            
            set(gcf, 'Position', [100, 100, 800, 800]);
            ax = gca;
            ax.FontSize = 14;
            ax.LineWidth = 2;
        end

    end
end
