function GN = gn_params() %#codegen
%GN_PARAMS  Single source of truth for every G&N number.
%   Every value carries ONE tag in its comment:
%     [SRC: ...]            sourced (RFP / DR-1 slide / datasheet / teammate)
%     [TBD: owner]          not yet known; value is NaN so misuse fails loudly
%     [PLACEHOLDER: why]    illustrative only; must be listed in open items
%   Units are SI and are in each field name suffix (_m, _mps, _s, _rad).
%
%   Step M1.0: only mission-level limits that the sequencer will check are
%   populated. Other modules add their own sub-structs as they start.

%% Mission limits (test/demo)
GN.mission.alt_max_agl_m      = 2.0;    % [SRC: RFP UAS-4; DR-1 UAS-14]
GN.mission.speed_max_mps      = 0.5;    % [SRC: RFP UAS-4; DR-1 UAS-15]
GN.mission.aoi_size_ne_m      = [5.0; 5.0]; % [SRC: RFP Definitions 1, "~5 m x 5 m"] (N x E extent; orientation TBD)
GN.mission.marker_acc_r_m     = 0.30;   % [SRC: RFP UAS-3; DR-1 UAS-10] radius
GN.mission.endurance_min_s    = 15*60;  % [SRC: RFP UAS-2; DR-1 UAS-7] lower bound on flight time

%% Mission sequencer (M1) — timing / thresholds (filled in later M1 steps)
GN.seq.rate_hz                = NaN;    % [TBD: G&N + Avionics] sequencer rate on Pi 5
GN.seq.search_timeout_s       = NaN;    % [TBD: G&N] give-up time before RTL; needs M3 coverage time + endurance budget
GN.seq.relay_ack_timeout_s    = NaN;    % [TBD: G&N + Rover comms] DR-1 ConOps step 6 requires ack
GN.seq.wp_reached_radius_m    = NaN;    % [TBD: G&N + D&C] tied to UAS-GNCC-10 (path accuracy, TBD)
end
