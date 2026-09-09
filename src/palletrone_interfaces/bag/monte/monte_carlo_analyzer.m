%% Population-level analysis for the v5 CoM-bias Monte Carlo experiment
scriptDir = fileparts(mfilename('fullpath'));
resultsPath = fullfile(scriptDir, 'results', 'monte_carlo_results.csv');
T = readtable(resultsPath, 'VariableNamingRule', 'preserve');
if isempty(T)
    error('Monte Carlo results contain no trials: %s', resultsPath);
end

if iscell(T.success) || isstring(T.success)
    success = strcmpi(string(T.success), "true") | string(T.success) == "1";
else
    success = logical(T.success);
end
scales = unique(T.com_scale, 'sorted');
fprintf('Total number of trials: %d\n', height(T));
fprintf('\nPer-CoM statistics (performance metrics use successful trials):\n');
fprintf(['scale  total success P_success  position RMS [mean median std min max Q90 Q95]' ...
         '  attitude RMS [mean median Q95]  final CoM [mean median Q95]' ...
         '  convergence [mean median Q95]  saturation_rate\n']);

q95Position = nan(size(scales));
q95Attitude = nan(size(scales));
successProbability = nan(size(scales));
for k = 1:numel(scales)
    atScale = T.com_scale == scales(k);
    good = atScale & success;
    nTotal = nnz(atScale);
    nSuccess = nnz(good);
    successProbability(k) = nSuccess / nTotal;
    p = describeMetric(T.position_rms(good));
    a = describeMetric(T.attitude_rms(good));
    c = describeMetric(T.final_com_error(good));
    convergence = describeMetric(T.moce_convergence_time(good));
    q95Position(k) = p.q95;
    q95Attitude(k) = a.q95;
    saturationRate = mean(T.saturation_time(atScale) > 0, 'omitnan');
    fprintf(['%.2f %6d %7d %9.3f  [%g %g %g %g %g %g %g]' ...
             '  [%g %g %g]  [%g %g %g]  [%g %g %g]  %.3f\n'], ...
        scales(k), nTotal, nSuccess, successProbability(k), ...
        p.mean, p.median, p.std, p.min, p.max, p.q90, p.q95, ...
        a.mean, a.median, a.q95, c.mean, c.median, c.q95, ...
        convergence.mean, convergence.median, convergence.q95, saturationRate);
end

fprintf('\nFailure reason counts:\n');
failureReasons = string(T.failure_reason(~success));
if isempty(failureReasons)
    fprintf('none: 0\n');
else
    [reasonNames, ~, reasonIndex] = unique(failureReasons);
    reasonCounts = accumarray(reasonIndex, 1);
    for k = 1:numel(reasonNames)
        fprintf('%s: %d\n', reasonNames(k), reasonCounts(k));
    end
end

figure('Name', 'Success probability');
plot(scales, successProbability, '-o', 'LineWidth', 1.5); grid on;
xlabel('CoM scale'); ylabel('Success probability'); ylim([0 1]);

figure('Name', 'Position RMS distribution');
boxchart(categorical(T.com_scale(success)), T.position_rms(success)); grid on;
xlabel('CoM scale'); ylabel('Position RMS [m]');

figure('Name', 'Attitude RMS distribution');
boxchart(categorical(T.com_scale(success)), T.attitude_rms(success)); grid on;
xlabel('CoM scale'); ylabel('Attitude RMS [rad]');

figure('Name', 'Final CoM estimation error');
boxchart(categorical(T.com_scale(success)), T.final_com_error(success)); grid on;
xlabel('CoM scale'); ylabel('Final CoM error [m]');

figure('Name', 'Q95 position RMS');
plot(scales, q95Position, '-o', 'LineWidth', 1.5); grid on;
xlabel('CoM scale'); ylabel('Q95 position RMS [m]');

figure('Name', 'Q95 attitude RMS');
plot(scales, q95Attitude, '-o', 'LineWidth', 1.5); grid on;
xlabel('CoM scale'); ylabel('Q95 attitude RMS [rad]');

figure('Name', 'Failure reasons');
if isempty(failureReasons)
    bar(categorical("none"), 0);
else
    bar(categorical(reasonNames), reasonCounts);
end
grid on; xlabel('Failure reason'); ylabel('Trial count');

function stats = describeMetric(values)
values = values(isfinite(values));
if isempty(values)
    stats = struct('mean', NaN, 'median', NaN, 'std', NaN, 'min', NaN, ...
                   'max', NaN, 'q90', NaN, 'q95', NaN);
    return;
end
stats = struct('mean', mean(values), 'median', median(values), ...
               'std', std(values), 'min', min(values), 'max', max(values), ...
               'q90', prctile(values, 90), 'q95', prctile(values, 95));
end
