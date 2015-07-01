function startup()
% STARTUP  Load necessary java packages

  % Load appropriate java jar-files
  fname = fullfile(fileparts(mfilename('fullpath')), 'sensordata.jar');
  fprintf('Loading: %s\n', fname);
  javaaddpath(fname);
end
