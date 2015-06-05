classdef visualizer2 < handle
    % visualizer
    
    properties(Access='private')
        m_fig;
        m_axis;
        m_coordAxis;
        m_hBox = [];
        m_hgt;
        phoneDepthZ  = 8.6/100;
        phoneLengthY = 138/100;
        phoneWidthX  = 69/100;
        m_xBox = [  1,  1, -1,  1,  1,  1;
            1,  1, -1,  1, -1, -1;
            -1, -1, -1,  1, -1, -1;
            -1, -1, -1,  1,  1,  1] * 69/100;
        m_yBox = [  1,  1,  1,  1, -1,  1;
            -1, -1,  1,  1, -1,  1;
            -1, -1, -1, -1, -1,  1;
            1,  1, -1, -1, -1,  1] * 138/100;
        m_zBox = [ -1,  1,  1,  1,  1,  1;
            -1,  1, -1, -1,  1,  1;
            -1,  1, -1, -1, -1, -1;
            -1,  1,  1,  1, -1, -1] * 8.6/100;
        
        m_axisPos = {[-(69/100+.1),  1;
            0                    0;
            0                    0],...
            [0,                   0;
            -(138/100+.1)             1;
            0                    0],...
            [0,                   0;
            0,                   0;
            -(8.6/100+.1)   1]};
    end
    
    methods
        % Constructor
        function self = visualizer2(name)
            if nargin<2
                self.m_fig = figure;
                set(self.m_fig, 'Renderer', 'OpenGl');
                self.m_axis = axes('CameraPosition', [0, -5, 0], ...
                    'CameraTarget',[0,0,0],...
                    'CameraUpVector',[0,0,1],...
                    'Parent',self.m_fig);
%                 set(self.m_fig, 'HandleVisibility', 'off');
                set(self.m_fig, 'name', name);
                self.m_hgt = hgtransform('Parent',  self.m_axis);
            end
            hold(self.m_axis,'on');
            self.m_coordAxis(1) = line(self.m_axisPos{1}(1, :),...
                self.m_axisPos{1}(2, :),...
                self.m_axisPos{1}(3, :),...
                'LineWidth', 2, 'Color', 'b');
            self.m_coordAxis(2) = line(self.m_axisPos{2}(1, :),...
                self.m_axisPos{2}(2, :),...
                self.m_axisPos{2}(3, :),...
                'LineWidth', 2, 'Color', 'g');
            self.m_coordAxis(3) = line(self.m_axisPos{3}(1, :),...
                self.m_axisPos{3}(2, :),...
                self.m_axisPos{3}(3, :),...
                'LineWidth', 2, 'Color', 'r');
            axis(self.m_axis, kron(1.5*[1 1 1], [-1 1]));
            view(self.m_axis,[-37.5, 30]);
            grid on;
            axis(self.m_axis, 'off');
            self.m_hBox = fill3(self.m_xBox, self.m_yBox, self.m_zBox, 'k',...
                'Parent', self.m_hgt);
            transparency = 0.9;
            set(self.m_hBox(1), 'FaceColor', [.9, .9, .9], 'FaceAlpha',transparency);
            set(self.m_hBox(2), 'FaceColor', [.9, .9, .9], 'FaceAlpha',transparency);
            set(self.m_hBox(3), 'FaceColor', [.9, .9, .9], 'FaceAlpha',transparency);
            set(self.m_hBox(4), 'FaceColor', [.9, .9, .9], 'FaceAlpha',transparency);
            set(self.m_hBox(5), 'FaceColor', [.9, .9, .9], 'FaceAlpha',transparency);
            set(self.m_hBox(6), 'FaceColor', [.9, .9, .9], 'FaceAlpha',transparency);
            
            set(self.m_coordAxis, 'Parent', self.m_hgt);
            self.setOrientation([1;0;0;0]);
        end
        
        % Destructor
        function delete(self)
            if ishandle(self.m_fig)
                delete(self.m_fig);
            end
        end
        
        % Set quaternion orientation
        function setOrientation(self,q)
            R = self.Qq(q);
            set(self.m_hgt, 'Matrix', [R, zeros(3,1); 
                                      [0  0  0  1]]);
%             R2 = [R(:,1), R(:,2), [0 0 1]'];
%             set(self.m_hgt, 'Matrix', [R2, zeros(3,1);
%                                       [0   0   0   1]]);
            drawnow;
        end
        
    end
    
    
    methods(Access = 'private')
        % Transform quaternion to rotation matrix
        function Q = Qq(self,q)
            % The matrix Q(q) defined in (13.16)
            q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
            Q = [2*(q0^2+q1^2) - 1  2*(q1*q2-q0*q3)    2*(q1*q3+q0*q2);
                2*(q1*q2+q0*q3)    2*(q0^2+q2^2) - 1  2*(q2*q3-q0*q1);
                2*(q1*q3-q0*q2)    2*(q2*q3+q0*q1)    2*(q0^2+q3^2) - 1];
        end
    end
    
end