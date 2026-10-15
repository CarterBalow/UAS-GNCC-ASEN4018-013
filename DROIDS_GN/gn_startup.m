function GN = gn_startup()
%GN_STARTUP  Set up the D.R.O.I.D.S. G&N project for a MATLAB session.
%   GN = gn_startup() adds the project folders to the MATLAB path and returns
%   the parameter struct from params/gn_params.m. Also places GN in the base
%   workspace so Simulink models can reference it.
%
%   Not code-generated (tooling only).

root = fileparts(mfilename('fullpath'));

% Folders that hold code/models. results/ and docs/ are intentionally excluded.
codeDirs = {'params', 'src', 'models', 'placeholders', 'tests'};
for k = 1:numel(codeDirs)
    addpath(genpath(fullfile(root, codeDirs{k})));
end

GN = gn_params();
assignin('base', 'GN', GN);

fprintf('[G&N] Project root: %s\n', root);
fprintf('[G&N] Parameters loaded into base workspace as struct GN.\n');
end
