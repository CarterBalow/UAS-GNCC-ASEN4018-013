function tests = test_M1_0_structure
%TEST_M1_0_STRUCTURE  Verification for step M1.0 (project skeleton + params).
%   Pass criteria:
%     P1  Required folders exist.
%     P2  gn_params() runs and returns the sourced mission limits with the
%         exact values from RFP / DR-1 (2 m, 0.5 m/s, 5x5 m, 0.30 m, 900 s).
%     P3  Every TBD sequencer field is NaN (so accidental use fails loudly).
%     P4  Every value line in gn_params.m carries a SRC / TBD / PLACEHOLDER tag.
tests = functiontests(localfunctions);
end

function setupOnce(tc)
tc.TestData.root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(tc.TestData.root, 'params'));
end

function testP1_folders(tc)
req = {'docs/modules','params','src/common','src/M1_sequencer','models', ...
       'placeholders/dynamics','placeholders/control','tests','results'};
for k = 1:numel(req)
    verifyTrue(tc, isfolder(fullfile(tc.TestData.root, req{k})), ['missing ' req{k}]);
end
end

function testP2_sourced_limits(tc)
GN = gn_params();
verifyEqual(tc, GN.mission.alt_max_agl_m,   2.0);
verifyEqual(tc, GN.mission.speed_max_mps,   0.5);
verifyEqual(tc, GN.mission.aoi_size_ne_m,   [5.0; 5.0]);
verifyEqual(tc, GN.mission.marker_acc_r_m,  0.30);
verifyEqual(tc, GN.mission.endurance_min_s, 900);
end

function testP3_tbd_are_nan(tc)
GN = gn_params();
f = fieldnames(GN.seq);
for k = 1:numel(f)
    verifyTrue(tc, isnan(GN.seq.(f{k})), ['GN.seq.' f{k} ' should be NaN (TBD)']);
end
end

function testP4_every_value_tagged(tc)
txt = splitlines(fileread(fullfile(tc.TestData.root, 'params', 'gn_params.m')));
isAssign = startsWith(strtrim(txt), 'GN.');
lines = txt(isAssign);
tagged = contains(lines, '[SRC:') | contains(lines, '[TBD:') | contains(lines, '[PLACEHOLDER:');
verifyTrue(tc, all(tagged), sprintf('untagged line(s):\n%s', strjoin(lines(~tagged), newline)));
end
