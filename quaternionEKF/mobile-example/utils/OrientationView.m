classdef OrientationView < handle
% ORIENTATIONVIEW  Class to visualize an orientation

  properties(Access='private')
    orientationQuat_ = [1; 0; 0; 0];
    fig_;
    axis_;
    coordAxis_;
    hP_ = [-1, -1, -1];
    hText_ = [];

    xBox_ = [  1,  1, -1,  1,  1,  1;
               1,  1, -1,  1, -1, -1;
              -1, -1, -1,  1, -1, -1;
              -1, -1, -1,  1,  1,  1] * 2.7/5.275*.6;
    yBox_ = [  1,  1,  1,  1, -1,  1;
              -1, -1,  1,  1, -1,  1;
              -1, -1, -1, -1, -1,  1;
               1,  1, -1, -1, -1,  1] * .6;
    zBox_ = [ -1,  1,  1,  1,  1,  1;
              -1,  1, -1, -1,  1,  1;
              -1,  1, -1, -1, -1, -1;
              -1,  1,  1,  1, -1, -1] * .35/5.275*.6;

    axisPos_ = {[-(2.7/5.275*.6+.1),  1;
                 0                    0;
                 0                    0],...
                [0,                   0;
                 -(.6+.1)             1;
                 0                    0],...
                [0,                   0;
                 0,                   0;
                 -(.35/5.275*.6+.1)   1]};
    hBox_ = [];
  end

  methods
    function self = OrientationView(figname, useAxis)
    % ORIENTATIONVIEW Default constructor
      if nargin>1
        self.fig_ = -1;
        set(get(useAxis, 'Parent'), 'Renderer', 'OpenGL');
        cla(useAxis);
        self.axis_ = useAxis;
        set(useAxis, 'CameraPosition', [0, -5, 0],...
                     'CameraTarget', [0, 0, 0],...
                     'CameraUpVector', [0, 0, 1]);
      else
        self.fig_ = figure;
        set(self.fig_, 'Renderer', 'OpenGL');
        self.axis_ = axes('CameraPosition', [0, -5, 0],...
                          'CameraTarget', [0, 0, 0],...
                          'CameraUpVector', [0, 0, 1],...
                          'Parent', self.fig_);
        set(self.fig_, 'HandleVisibility', 'off');
        if nargin < 1
          set(self.fig_, 'name', 'Orientation View');
        else
          set(self.fig_, 'name', figname);
        end
      end
      hold(self.axis_, 'on');
      self.coordAxis_(1) = line(self.axisPos_{1}(1, :),...
                                self.axisPos_{1}(2, :),...
                                self.axisPos_{1}(3, :),...
                                'Parent', self.axis_,...
                                'LineWidth', 3, 'Color', 'b');
      self.coordAxis_(2) = line(self.axisPos_{2}(1, :),...
                                self.axisPos_{2}(2, :),...
                                self.axisPos_{2}(3, :),...
                                'Parent', self.axis_,...
                                'LineWidth', 3, 'Color', 'g');
      self.coordAxis_(3) = line(self.axisPos_{3}(1, :),...
                                self.axisPos_{3}(2, :),...
                                self.axisPos_{3}(3, :),...
                                'Parent', self.axis_,...
                                'LineWidth', 3, 'Color', 'r');
      axis(self.axis_, kron(1.5*[1 1 1], [-1 1]));
      view(self.axis_, [0, 0]);
      axis(self.axis_, 'off');
      self.hBox_ = fill3(self.xBox_, self.yBox_, self.zBox_, 'k',...
                         'Parent', self.axis_);
      set(self.hBox_(2), 'FaceColor', [.9, .9, .9]);
      self.setOrientation([1; 0; 0; 0], .01*eye(4));

      self.hText_(1) = text(1, 0, 1, 'Standstill', 'Parent', self.axis_,...
                            'Color', [0, 0.5, 0], 'FontSize', 16,...
                            'Visible', 'off');
      self.hText_(2) = text(1, 0, 0.7, 'Mag dist', 'Parent', self.axis_,...
                            'Color', [0.8, 0, 0], 'FontSize', 16,...
                            'Visible', 'off');
      self.hText_(3) = text(1, 0, 0.7, 'Mag OK', 'Parent', self.axis_,...
                            'Color', [0, .4, 0], 'FontSize', 16,...
                            'Visible', 'off');
      self.hText_(4) = text(1, 0, 0.4, 'Acc dist', 'Parent', self.axis_,...
                            'Color', [0.8, 0, 0], 'FontSize', 16,...
                            'Visible', 'off');
      self.hText_(5) = text(1, 0, 0.4, 'Acc OK', 'Parent', self.axis_,...
                            'Color', [0, .4, 0], 'FontSize', 16,...
                            'Visible', 'off');
    end

    function delete(self)
    % DELETE Object destructor
      if ishandle(self.fig_)
        delete(self.fig_);
      end
    end

    function addOffset(self, x, y)
    % ADDOFFSET Add offset to displayed window location
      set(self.fig_, 'Position', get(self.fig_, 'Position') + [x, y, 0, 0]);
    end

    function setOrientation(self, q, P)
    % SETORIENTATION Set the orientation to be displayed
    % setOrientation(orientationView, quaternion)

      A = self.getAxis(q);
      for i=1:3
        foo = A * self.axisPos_{i};
        set(self.coordAxis_(i), 'XData', foo(1, :),...
                                'YData', foo(2, :),...
                                'ZData', foo(3, :));
        if ishandle(self.hP_(i))
          delete(self.hP_(i));
        end
      end
      x = self.xBox_*A(1, 1) + self.yBox_*A(1, 2) + self.zBox_*A(1, 3);
      y = self.xBox_*A(2, 1) + self.yBox_*A(2, 2) + self.zBox_*A(2, 3);
      z = self.xBox_*A(3, 1) + self.yBox_*A(3, 2) + self.zBox_*A(3, 3);
      for i=1:6
        set(self.hBox_(i),...
            'XData', x(:, i), 'YData', y(:, i), 'ZData', z(:, i));
      end
      if nargin>2
        A = A * [self.axisPos_{1}(:, 2), self.axisPos_{2}(:, 2), self.axisPos_{3}(:, 2)];
        [Px, Py, Pz] = self.getAxisCov(q, P);

        [U, S] = svd(Px);
        U = U*sqrt(S)*U';
        R = (eye(3) - 0.9*A(:, 1)*A(:, 1)'/(A(:, 1)'*A(:, 1)));
        [X, Y, Z] = ellipsoid(0, 0, 0, 1, 1, 1);
        foo = bsxfun(@plus, R*U*[X(:), Y(:), Z(:)]', A(:, 1));
        X(:) = foo(1, :);
        Y(:) = foo(2, :);
        Z(:) = foo(3, :);
        self.hP_(1) = surf(self.axis_, X, Y, Z,...
            'FaceColor', 'b', 'FaceAlpha', .4, 'EdgeColor', 'none');

        [U, S] = svd(Py);
        U = (U*sqrt(S)*U')';
        R = (eye(3) - 0.9*A(:, 2)*A(:, 2)'/(A(:, 2)'*A(:, 2)));
        [X, Y, Z] = ellipsoid(0, 0, 0, 1, 1, 1);
        foo = bsxfun(@plus, R*U*[X(:), Y(:), Z(:)]', A(:, 2));
        X(:) = foo(1, :);
        Y(:) = foo(2, :);
        Z(:) = foo(3, :);
        self.hP_(2) = surf(self.axis_, X, Y, Z,...
            'FaceColor', 'g', 'FaceAlpha', .4, 'EdgeColor', 'none');

        [U, S] = svd(Pz);
        U = U*sqrt(S)*U';
        R = (eye(3) - 0.9*A(:, 3)*A(:, 3)'/(A(:, 3)'*A(:, 3)));
        [X, Y, Z] = ellipsoid(0, 0, 0, 1, 1, 1);
        foo = bsxfun(@plus, R*U*[X(:), Y(:), Z(:)]', A(:, 3));
        X(:) = foo(1, :);
        Y(:) = foo(2, :);
        Z(:) = foo(3, :);
        self.hP_(3) = surf(self.axis_, X, Y, Z,...
            'FaceColor', 'r', 'FaceAlpha', .4, 'EdgeColor', 'none');
      end
      drawnow;
    end

    function title(self, varargin)
    % TITLE Set title of window
      title(self.axis_, varargin{:});
    end

    function setStandStill(self, flag)
    % SETSTANDSTILL Set the stand still indicator
      if flag
        set(self.hText_(1), 'Visible', 'on');
      else
        set(self.hText_(1), 'Visible', 'off');
      end
    end

    function setAccDist(self, flag)
    % SETACCDIST Set acceleration disturbance indicator (external acceleration present)
      if flag
        set(self.hText_(4), 'Visible', 'on');
        set(self.hText_(5), 'Visible', 'off');
      else
        set(self.hText_(4), 'Visible', 'off');
        set(self.hText_(5), 'Visible', 'on');
      end
    end

    function setMagDist(self, flag)
    % SETMAGDIST Set magnetometer disturbance indicator
      if flag
        set(self.hText_(2), 'Visible', 'on');
        set(self.hText_(3), 'Visible', 'off');
      else
        set(self.hText_(2), 'Visible', 'off');
        set(self.hText_(3), 'Visible', 'on');
      end
    end
  end

  methods(Access = 'private')
    function A = getAxis(self, q)
    % GETAXIS Compute rotated coordinate axes
      q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
      A = [2*(q0^2+q1^2) - 1  2*(q1*q2-q0*q3)    2*(q1*q3+q0*q2);
           2*(q1*q2+q0*q3)    2*(q0^2+q2^2) - 1  2*(q2*q3-q0*q1);
           2*(q1*q3-q0*q2)    2*(q2*q3+q0*q1)    2*(q0^2+q3^2) - 1];
    end

    function [Px, Py, Pz] = getAxisCov(self, q, P)
    % GETAXISCOV Compute covariances for rotated coordinate axes
      q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
      Qx = 2 * [ q0  q1 -q2 -q3;
                 q3  q2  q1  q0;
                -q2  q3 -q0  q1];
      Qy = 2 * [-q3  q2  q1 -q0;
                 q0 -q1  q2 -q3;
                 q1  q0  q3  q2];
      Qz = 2 * [ q2  q3  q0  q1;
                -q1 -q0  q3  q2;
                 q0 -q1 -q2  q3];

      Px = Qx*P*Qx';
      Py = Qy*P*Qy';
      Pz = Qz*P*Qz';
    end
  end
end
