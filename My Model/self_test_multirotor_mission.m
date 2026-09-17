function report = self_test_multirotor_mission()
%SELF_TEST_MULTIROTOR_MISSION Repeatable end-to-end regression check.

overrides.randomSeed = 314159;
overrides.visualization.showFigures = false;
overrides.visualization.animate = false;
results = run_multirotor_mission(overrides);
assert(results.checks.allPassed,'Seeded end-to-end mission checks failed.');

randomCfg = mission_config(struct('randomSeed',[]));
firstAOI = generate_random_aoi(randomCfg);
pause(0.02);
secondAOI = generate_random_aoi(randomCfg);
randomizationDistanceM = norm(firstAOI.centerEN-secondAOI.centerEN);
assert(randomizationDistanceM > 1e-9, ...
    'Two ordinary AOI generations unexpectedly produced the same center.');

report = results.checks;
report.randomAOICentersDiffer = true;
report.randomAOICenterSeparationM = randomizationDistanceM;
fprintf('Random AOI check: PASS (centers differ by %.3f m).\n', ...
    randomizationDistanceM);
end
