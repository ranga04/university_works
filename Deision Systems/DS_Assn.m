% clear all;
% clc;
% 
% %% Lab A Code
% %% Task A.0: Initial Setup
% % Initial controller gains
% initial_gains = [1, 0.1];  % Initial guess for KP and KI
% 
% % Evaluate initial system performance
% initial_performance = evaluateControlSystem(initial_gains);
% disp('Initial performance:');
% disp(initial_performance);
% 
% % Check system stability
% if abs(initial_performance(1)) < 1
%     disp('The system is stable.');
% else
%     disp('The system is not stable.');
% end
% 
% %% Task A.1: Generate and Compare Three Sampling Plans
% num_samples = 100;  % Budget of 100 candidate designs
% K_P_range = [0.1, 10];
% K_I_range = [0.01, 1];
% 
% % 1. Full Factorial Sampling Plan with Perturbations
% points_per_dimension = [10, 10];  % Points along each dimension
% factorial_sampling = fullfactorial(points_per_dimension, 1);
% factorial_sampling = 10 * factorial_sampling + eps;
% % Add Perturbations
% perturbation_level = 0.3;  % Noise level
% factorial_perturbed = factorial_sampling + perturbation_level * randn(size(factorial_sampling));  % Gaussian noise
% 
% % 2. Latin Hypercube Sampling Plan
% lhs_sampling = rlh(100, 2, 5);
% lhs_sampling = 10 * lhs_sampling + eps;
% 
% % 3. Sobol Sequence Sampling Plan
% sobol_seq = sobolset(2);
% sobol_sampling = net(sobol_seq, 100);
% sobol_sampling = 10 * sobol_sampling + eps;
% 
% % Visualize the Sampling Plans
% figure;
% 
% % First subplot: Full Factorial Sampling
% subplot(1, 3, 1);
% scatter(factorial_perturbed(:,1), factorial_perturbed(:,2), 'filled','r')
% xlabel('K_P')
% ylabel('K_I')
% title('Full Factorial Sampling (2D)')
% grid on
% set(gca, 'FontSize', 12)
% 
% % Second subplot: Latin Hypercube Sampling
% subplot(1, 3, 2);
% scatter(lhs_sampling(:,1), lhs_sampling(:,2), 'filled','r')
% xlabel('K_P')
% ylabel('K_I')
% title('Latin Hypercube Sampling (2D)')
% grid on
% set(gca, 'FontSize', 12)
% 
% % Third subplot: Sobol Sequence Sampling
% subplot(1, 3, 3);
% scatter(sobol_sampling(:,1), sobol_sampling(:,2), 'filled','r')
% xlabel('K_P')
% ylabel('K_I')
% title('Sobol Sequence Sampling (2D)')
% grid on
% set(gca, 'FontSize', 12)
% 
% % Save the combined figure
% saveas(gcf, 'CombinedSampling.png')
% 
% 
% 
% % Assess space-filling properties using Φq metric
% phi_fullfactorial = mmphi(factorial_perturbed, 5, 2);
% phi_lhs = mmphi(lhs_sampling, 5, 2);
% phi_sobol = mmphi(sobol_sampling, 5, 2);
% 
% % Display Φq metrics
% disp('Φq metric for Full Factorial Plan:');
% disp(phi_fullfactorial);
% disp('Φq metric for Latin Hypercube Plan:');
% disp(phi_lhs);
% disp('Φq metric for Sobol Sequence Plan:');
% disp(phi_sobol);
% 
% % Identify the best plan
% [~, best_plan_idx] = min([phi_fullfactorial, phi_lhs, phi_sobol]);
% if best_plan_idx == 1
%     best_sampling_plan = factorial_perturbed;
%     best_plan_name = 'Full Factorial Plan';
% elseif best_plan_idx == 2
%     best_sampling_plan = lhs_sampling;
%     best_plan_name = 'Latin Hypercube Plan';
% else
%     best_sampling_plan = sobol_sampling;
%     best_plan_name = 'Sobol Sequence Plan';
% end
% 
% disp('Best Sampling Plan:');
% disp(best_plan_name);
% 
% %% Task A.2: Discover Insights from Sampling Plan
% num_samples = size(best_sampling_plan, 1);
% performance_metrics = zeros(num_samples, 10);  
% 
% % Evaluate the designs using the best sampling plan
% for i = 1:num_samples
%     gains = best_sampling_plan(i, :);
%     performance_metrics(i, :) = evaluateControlSystem(gains);
% end
% 
% % Visualize relationships between design variables and performance criteria
% 
% % Matrix scatter plot
% figure;
% plotmatrix(best_sampling_plan, performance_metrics);
% title('Matrix Scatter Plot of Design Variables and Performance Criteria');
% set(gca, 'FontSize', 12);
% grid on;
% saveas(gcf, 'MatrixScatterPlot.png');
% 
% % Parallel coordinates plot
% figure;
% parallelcoords(performance_metrics,'LineWidth', 2.5, 'Color', 'r');
% title('Performance Criteria Parallel Coordinates Plot');
% set(gca, 'FontSize', 12);
% grid on;
% saveas(gcf, 'ParallelCoordinatesPlot.png');
% 
% % Save the best sampling plan to a .mat file
% save('best_plan.mat', 'best_sampling_plan');
% 
% %% Lab B Code
% %% Load and Normalize Best Plan
% load('best_plan.mat', 'best_sampling_plan'); 
% normalized_sampling_plan = normalize(best_sampling_plan);
% 
% %% Evaluate Initial Control System
% Z_initial = processControlSystem(normalized_sampling_plan);
% 
% %% Optimization Process Using NSGA-II
% Z_current = processControlSystem(normalized_sampling_plan);
% 
% % Initialize hypervolume indicator
% hypervolume = [];
% reference_point = [max(Z_current)];
% 
% % NSGA-II Optimization for 250 iterations
% num_iterations = 250;
% current_sampling_plan = normalized_sampling_plan;
% priority_levels = [3 2 2 1 0 1 0 0 1 2];
% target_goals = [1 -6 20 2 10 10 8 20 1 0.67];
% mutation_boundary = [0.1 0.1; 1 1];
% 
% for iteration = 1:num_iterations
%     % Perform one iteration of NSGA-II optimization
%     [current_sampling_plan, Z_current] = runNsgaIteration(current_sampling_plan, Z_current, priority_levels, target_goals, mutation_boundary);
% 
%     % Log intermediate population and performance values for debugging
%     fprintf('Iteration %d: Hypervolume = %f\n', iteration, sum(Z_current(:)));
%     logPerformanceMetrics(current_sampling_plan, Z_current, iteration);
% 
%     % Update hypervolume for convergence plot
%     hypervolume = [hypervolume; sum(Z_current(:))]; 
% end
% 
% %% Plot Optimized Results
% plotOptimizationResults(current_sampling_plan, Z_current, num_iterations, hypervolume);
% 
% %% Supporting Functions
% 
% function Z = processControlSystem(P)
%     Z = evaluateControlSystem(P);
%     Z = adjustGainMargin(Z);
%     Z = adjustPhaseMargin(Z);
%     Z = replaceInfValues(Z);
% end
% 
% function Z = adjustGainMargin(Z)
%     % Modify Gain Margin
%     Z(:, 2) = -20 * log10(Z(:, 2));
% end
% 
% function Z = adjustPhaseMargin(Z)
%     % Modify Phase Margin
%     Z(:, 3) = abs(Z(:, 3) - 50);
% end
% 
% function Z = replaceInfValues(Z)
%     % Replace inf values with large number
%     Z(isinf(Z)) = 1e6;
% end
% 
% function [new_sampling_plan, Z_new] = runNsgaIteration(sampling_plan, Z_current, priority, goals, boundary)
%     % Save current sampling plan
%     parent_plan = sampling_plan;
% 
%     % Rank preferability
%     [rank, ~] = rankPreferability(Z_current, goals, priority);
% 
%     % Crowd sorting
%     crowd_distances = calculateCrowding(Z_current, rank);
% 
%     % Invert rankings
%     rank = max(rank) - rank;
% 
%     % Binary Tournament Selection
%     selected_indices = binaryTournamentSelection([rank, crowd_distances], 100);
% 
%     % Crossover and Mutation
%     crossover_offspring = simulatedBinaryCrossover(parent_plan(selected_indices,:), boundary);
%     mutated_offspring = polynomialMutation(crossover_offspring, boundary);
% 
%     % Combine parent and offspring
%     combined_population = [parent_plan; mutated_offspring];
% 
%     % Evaluate combined population
%     Z_combined = processControlSystem(combined_population);
% 
%     % Rank preferability for combined population
%     [rank_combined, ~] = rankPreferability(Z_combined, goals, priority);
% 
%     % Crowd sorting for combined population
%     crowd_distances_combined = calculateCrowding(Z_combined, rank_combined);
% 
%     % NSGA-II selection for next iteration
%     selected_indices_nsga = nsgaSelection(combined_population, rank_combined, crowd_distances_combined, 100);
% 
%     % Find new NSGA-II reduced population
%     new_sampling_plan = combined_population(selected_indices_nsga, :);
% 
%     % Evaluate new reduced population
%     Z_new = processControlSystem(new_sampling_plan);
% end
% 
% function logPerformanceMetrics(sampling_plan, Z_current, iteration)
%     for idx = 1:size(sampling_plan, 1)
%         fprintf('Performance of individual %d at iteration %d: %f %f %f %f %f %f %f %f %f %f\n', ...
%             idx, iteration, Z_current(idx, :));
%     end
% end
% 
% function plotOptimizationResults(sampling_plan, Z_current, num_iterations, hypervolume)
%     figure;
%     scatter(sampling_plan(:, 1), sampling_plan(:, 2), 'filled')
%     xlabel('K_P')
%     ylabel('K_I')
%     title('Optimized Sampling Plan')
% 
%     % Plot correlation with optimized Z value
%     figure;
%     labels = {'Largest CL Pole', 'Gain Margin', 'Phase Margin', 'RiseTime', ...
%         'Peak Time', 'MaxOvershoot', 'MaxUndershoot', 'SettlingTime', 'SSError',...
%         'ControlEffort'};
%     [h, ax] = plotmatrix(Z_current);
% 
%     % Set axis labels
%     num_vars = size(Z_current, 2);
%     for i = 1:num_vars
%         ax(num_vars, i).XLabel.String = labels{i};
%         ax(i, 1).YLabel.String = labels{i};
%     end
% 
%     % Parallel Plot after optimization
%     figure;
%     parallelplot(array2table(Z_current, 'VariableNames', labels));
%     title('Parallel Plot of Performance Criteria');
% 
%     % Plot hypervolume convergence 
%     figure;
%     plot(1:num_iterations, hypervolume, 'LineWidth', 2.5, 'Color', 'r');
%     xlabel('Iteration');
%     ylabel('Hypervolume');
%     title('Convergence Plot');
%     grid on;
% end
% 
% function [rank, ClassV] = rankPreferability(Z_value, goals, priority)
%     % Rank Preferability
%     deviations = abs(Z_value - goals);
%     weighted_deviations = deviations .* priority;
%     rank = sum(weighted_deviations, 2);
%     [~, ClassV] = sort(rank);
% end
% 
% function crowd_distances = calculateCrowding(Z_value, rank)
%     % Calculate Crowding Distance
%     num_points = size(Z_value, 1);
%     num_objectives = size(Z_value, 2);
%     crowd_distances = zeros(num_points, 1);
% 
%     for i = 1:num_objectives
%         [~, sorted_idx] = sort(Z_value(:, i));
%         objective_values = Z_value(sorted_idx, i);
%         max_value = max(objective_values);
%         min_value = min(objective_values);
% 
%         crowd_distances(sorted_idx(1)) = Inf;
%         crowd_distances(sorted_idx(end)) = Inf;
% 
%         for j = 2:(num_points - 1)
%             crowd_distances(sorted_idx(j)) = crowd_distances(sorted_idx(j)) + ...
%                 (objective_values(j+1) - objective_values(j-1)) / (max_value - min_value);
%         end
%     end
% end
% 
% function selected_indices = binaryTournamentSelection(fitness, n)
%     % Binary Tournament Selection
%     population_size = size(fitness, 1);
%     selected_indices = zeros(n, 1);
% 
%     for i = 1:n
%         candidates = randperm(population_size, 2);
%         if fitness(candidates(1), 1) < fitness(candidates(2), 1)
%             selected_indices(i) = candidates(1);
%         else
%             selected_indices(i) = candidates(2);
%         end
%     end
% end
% 
% function offspring = simulatedBinaryCrossover(parent, boundary)
%     % Simulated Binary Crossover
%     num_parents = size(parent, 1);
%     num_vars = size(parent, 2);
%     offspring = zeros(num_parents, num_vars);
%     for i = 1:2:num_parents
%         if i+1 > num_parents
%             break;
%         end
%         p1 = parent(i, :);
%         p2 = parent(i+1, :);
%         for j = 1:num_vars
%             if rand < 0.5
%                 beta = rand;
%                 offspring(i, j) = 0.5 * ((1 + beta) * p1(j) + (1 - beta) * p2(j));
%                 offspring(i+1, j) = 0.5 * ((1 - beta) * p1(j) + (1 + beta) * p2(j));
%             else
%                 offspring(i, j) = p1(j);
%                 offspring(i+1, j) = p2(j);
%             end
%         end
%     end
%     offspring = max(min(offspring, boundary(2, :)), boundary(1, :));
% end
% 
% function mutated_offspring = polynomialMutation(offspring, boundary)
%     % Polynomial Mutation
%     [num_offspring, num_vars] = size(offspring);
%     mutation_rate = 1/num_vars;
%     eta_m = 20;
%     mutated_offspring = offspring;
% 
%     for i = 1:num_offspring
%         for j = 1:num_vars
%             if rand < mutation_rate
%                 delta = rand;
%                 if delta < 0.5
%                     delta_q = (2 * delta)^(1/(eta_m + 1)) - 1;
%                 else
%                     delta_q = 1 - (2 * (1 - delta))^(1/(eta_m + 1));
%                 end
%                 mutated_offspring(i, j) = offspring(i, j) + delta_q * (boundary(2, j) - boundary(1, j));
%                 mutated_offspring(i, j) = min(max(mutated_offspring(i, j), boundary(1, j)), boundary(2, j));
%             end
%         end
%     end
% end
% 
% function selected_indices = nsgaSelection(population, rank_combined, crowd_distances_combined, n)
%     % NSGA-II Selection
%     [~, sorted_idx] = sortrows([rank_combined, -crowd_distances_combined], [1, 2]);
%     selected_indices = sorted_idx(1:n);
% end
% 
% function normalized_data = normalize(data)
%     % Normalize data to range [0, 1]
%     min_val = min(data(:));
%     max_val = max(data(:));
%     normalized_data = (data - min_val) / (max_val - min_val);
% end




clear all;
clc;

%% Lab A Code
%% Task A.0: Initial Setup
% Initial controller gains
initial_gains = [1, 0.1];  % Initial guess for KP and KI

% Evaluate initial system performance
initial_performance = evaluateControlSystem(initial_gains);
disp('Initial performance:');
disp(initial_performance);

% Check system stability
if abs(initial_performance(1)) < 1
    disp('The system is stable.');
else
    disp('The system is not stable.');
end

%% Task A.1: Generate and Compare Three Sampling Plans
num_samples = 100;  % Budget of 100 candidate designs
K_P_range = [0.1, 10];
K_I_range = [0.01, 1];

% 1. Full Factorial Sampling Plan with Perturbations
points_per_dimension = [10, 10];  % Points along each dimension
factorial_sampling = fullfactorial(points_per_dimension, 1);
factorial_sampling = 10 * factorial_sampling + eps;
% Add Perturbations
perturbation_level = 0.3;  % Noise level
factorial_perturbed = factorial_sampling + perturbation_level * randn(size(factorial_sampling));  % Gaussian noise

% 2. Latin Hypercube Sampling Plan
lhs_sampling = rlh(100, 2, 5);
lhs_sampling = 10 * lhs_sampling + eps;

% 3. Sobol Sequence Sampling Plan
sobol_seq = sobolset(2);
sobol_sampling = net(sobol_seq, 100);
sobol_sampling = 10 * sobol_sampling + eps;

% Visualize the Sampling Plans
figure;

% First subplot: Full Factorial Sampling
subplot(1, 3, 1);
scatter(factorial_perturbed(:,1), factorial_perturbed(:,2), 'filled', 'r')
xlabel('K_P')
ylabel('K_I')
title('Full Factorial Sampling (2D)')
grid on
set(gca, 'FontSize', 12)

% Second subplot: Latin Hypercube Sampling
subplot(1, 3, 2);
scatter(lhs_sampling(:,1), lhs_sampling(:,2), 'filled', 'g')
xlabel('K_P')
ylabel('K_I')
title('Latin Hypercube Sampling (2D)')
grid on
set(gca, 'FontSize', 12)

% Third subplot: Sobol Sequence Sampling
subplot(1, 3, 3);
scatter(sobol_sampling(:,1), sobol_sampling(:,2), 'filled', 'b')
xlabel('K_P')
ylabel('K_I')
title('Sobol Sequence Sampling (2D)')
grid on
set(gca, 'FontSize', 12)

% Save the combined figure
saveas(gcf, 'CombinedSampling.png')

% Assess space-filling properties using Φq metric
phi_fullfactorial = mmphi(factorial_perturbed, 5, 2);
phi_lhs = mmphi(lhs_sampling, 5, 2);
phi_sobol = mmphi(sobol_sampling, 5, 2);

% Display Φq metrics
disp('Φq metric for Full Factorial Plan:');
disp(phi_fullfactorial);
disp('Φq metric for Latin Hypercube Plan:');
disp(phi_lhs);
disp('Φq metric for Sobol Sequence Plan:');
disp(phi_sobol);

% Identify the best plan
[~, best_plan_idx] = min([phi_fullfactorial, phi_lhs, phi_sobol]);
if best_plan_idx == 1
    best_sampling_plan = factorial_perturbed;
    best_plan_name = 'Full Factorial Plan';
elseif best_plan_idx == 2
    best_sampling_plan = lhs_sampling;
    best_plan_name = 'Latin Hypercube Plan';
else
    best_sampling_plan = sobol_sampling;
    best_plan_name = 'Sobol Sequence Plan';
end

disp('Best Sampling Plan:');
disp(best_plan_name);

%% Task A.2: Discover Insights from Sampling Plan
num_samples = size(best_sampling_plan, 1);
performance_metrics = zeros(num_samples, 10);  

% Evaluate the designs using the best sampling plan
for i = 1:num_samples
    gains = best_sampling_plan(i, :);
    performance_metrics(i, :) = evaluateControlSystem(gains);
end

% Visualize relationships between design variables and performance criteria

% Matrix scatter plot
figure;
plotmatrix(best_sampling_plan, performance_metrics);
title('Matrix Scatter Plot of Design Variables and Performance Criteria');
set(gca, 'FontSize', 12);
grid on;
saveas(gcf, 'MatrixScatterPlot.png');

% Parallel coordinates plot
figure;
parallelcoords(performance_metrics, 'LineWidth', 2.5, 'Color', 'r');
title('Performance Criteria Parallel Coordinates Plot');
set(gca, 'FontSize', 12);
grid on;
saveas(gcf, 'ParallelCoordinatesPlot.png');

% Save the best sampling plan to a .mat file
save('best_plan.mat', 'best_sampling_plan');

%% Lab B Code
%% Load and Normalize Best Plan
load('best_plan.mat', 'best_sampling_plan'); 

% Normalize input parameters to [0, 1]
normalized_sampling_plan = normalize(best_sampling_plan);

%% Evaluate Initial Control System
Z_initial = processControlSystem(normalized_sampling_plan);

%% Optimization Process Using NSGA-II
Z_current = processControlSystem(normalized_sampling_plan);

% Initialize hypervolume indicator
hypervolume = [];
reference_point = [max(Z_current)];

% NSGA-II Optimization for 250 iterations
num_iterations = 250;
current_sampling_plan = normalized_sampling_plan;
priority_levels = [3 2 2 1 0 1 0 0 1 2];
target_goals = [1 -6 20 2 10 10 8 20 1 0.63];
mutation_boundary = [0.1 0.1; 1 1];

for iteration = 1:num_iterations
    % Perform one iteration of NSGA-II optimization
    [current_sampling_plan, Z_current] = runNsgaIteration(current_sampling_plan, Z_current, priority_levels, target_goals, mutation_boundary);

    % Log intermediate population and performance values for debugging
    fprintf('Iteration %d: Hypervolume = %f\n', iteration, sum(Z_current(:)));
    logPerformanceMetrics(current_sampling_plan, Z_current, iteration);

    % Update hypervolume for convergence plot
    hypervolume = [hypervolume; sum(Z_current(:))]; 
end

% Combined plot for initial and optimized sampling plans
figure;
scatter(normalized_sampling_plan(:, 1), normalized_sampling_plan(:, 2), 'filled', 'r')
hold on;
scatter(current_sampling_plan(:, 1), current_sampling_plan(:, 2), 'filled', 'b')
xlabel('K_P')
ylabel('K_I')
title('Sampling Plans Before and After Optimization')
legend('Initial Sampling Plan', 'Optimized Sampling Plan')
grid on
set(gca, 'FontSize', 12)
saveas(gcf, 'InitialVsOptimizedSamplingPlan.png')

%% Plot Optimized Results
plotOptimizationResults(current_sampling_plan, Z_current, num_iterations, hypervolume);

%% Supporting Functions

function Z = processControlSystem(P)
    Z = evaluateControlSystem(P);
    Z = adjustGainMargin(Z);
    Z = adjustPhaseMargin(Z);
    Z = replaceInfValues(Z);
end

function Z = adjustGainMargin(Z)
    % Modify Gain Margin
    Z(:, 2) = -20 * log10(Z(:, 2));
end

function Z = adjustPhaseMargin(Z)
    % Modify Phase Margin
    Z(:, 3) = abs(Z(:, 3) - 50);
end

function Z = replaceInfValues(Z)
    % Replace inf values with large number
    Z(isinf(Z)) = 1e6;
end

function [new_sampling_plan, Z_new] = runNsgaIteration(sampling_plan, Z_current, priority, goals, boundary)
    % Save current sampling plan
    parent_plan = sampling_plan;

    % Rank preferability
    [rank, ~] = rankPreferability(Z_current, goals, priority);
    
    % Crowd sorting
    crowd_distances = calculateCrowding(Z_current, rank);
    
    % Invert rankings
    rank = max(rank) - rank;
    
    % Binary Tournament Selection
    selected_indices = binaryTournamentSelection([rank, crowd_distances], 100);
    
    % Crossover and Mutation
    crossover_offspring = simulatedBinaryCrossover(parent_plan(selected_indices,:), boundary);
    mutated_offspring = polynomialMutation(crossover_offspring, boundary);
    
    % Combine parent and offspring
    combined_population = [parent_plan; mutated_offspring];
    
    % Evaluate combined population
    Z_combined = processControlSystem(combined_population);
    
    % Rank preferability for combined population
    [rank_combined, ~] = rankPreferability(Z_combined, goals, priority);
    
    % Crowd sorting for combined population
    crowd_distances_combined = calculateCrowding(Z_combined, rank_combined);
    
    % NSGA-II selection for next iteration
    selected_indices_nsga = nsgaSelection(combined_population, rank_combined, crowd_distances_combined, 100);
    
    % Find new NSGA-II reduced population
    new_sampling_plan = combined_population(selected_indices_nsga, :);
    
    % Evaluate new reduced population
    Z_new = processControlSystem(new_sampling_plan);
end

function logPerformanceMetrics(sampling_plan, Z_current, iteration)
    for idx = 1:size(sampling_plan, 1)
        fprintf('Performance of individual %d at iteration %d: %f %f %f %f %f %f %f %f %f %f\n', ...
            idx, iteration, Z_current(idx, :));
    end
end

function plotOptimizationResults(sampling_plan, Z_current, num_iterations, hypervolume)
    figure;
    scatter(sampling_plan(:, 1), sampling_plan(:, 2), 'filled', 'm')
    xlabel('K_P')
    ylabel('K_I')
    title('Optimized Sampling Plan')

    % Plot correlation with optimized Z value
    figure;
    labels = {'Largest CL Pole', 'Gain Margin', 'Phase Margin', 'RiseTime', ...
        'Peak Time', 'MaxOvershoot', 'MaxUndershoot', 'SettlingTime', 'SSError',...
        'ControlEffort'};
    [h, ax] = plotmatrix(Z_current);

    % Set axis labels
    num_vars = size(Z_current, 2);
    for i = 1:num_vars
        ax(num_vars, i).XLabel.String = labels{i};
        ax(i, 1).YLabel.String = labels{i};
    end

    % Parallel Plot after optimization
    figure;
    parallelplot(array2table(Z_current, 'VariableNames', labels));
    title('Parallel Plot of Performance Criteria');

    % Plot hypervolume convergence 
    figure;
    plot(1:num_iterations, hypervolume, 'LineWidth', 2.5, 'Color', 'r');
    xlabel('Iteration');
    ylabel('Hypervolume');
    title('Convergence Plot');
    grid on;
end

function [rank, ClassV] = rankPreferability(Z_value, goals, priority)
    % Rank Preferability
    deviations = abs(Z_value - goals);
    weighted_deviations = deviations .* priority;
    rank = sum(weighted_deviations, 2);
    [~, ClassV] = sort(rank);
end

function crowd_distances = calculateCrowding(Z_value, rank)
    % Calculate Crowding Distance
    num_points = size(Z_value, 1);
    num_objectives = size(Z_value, 2);
    crowd_distances = zeros(num_points, 1);
    
    for i = 1:num_objectives
        [~, sorted_idx] = sort(Z_value(:, i));
        objective_values = Z_value(sorted_idx, i);
        max_value = max(objective_values);
        min_value = min(objective_values);
        
        crowd_distances(sorted_idx(1)) = Inf;
        crowd_distances(sorted_idx(end)) = Inf;
        
        for j = 2:(num_points - 1)
            crowd_distances(sorted_idx(j)) = crowd_distances(sorted_idx(j)) + ...
                (objective_values(j+1) - objective_values(j-1)) / (max_value - min_value);
        end
    end
end

function selected_indices = binaryTournamentSelection(fitness, n)
    % Binary Tournament Selection
    population_size = size(fitness, 1);
    selected_indices = zeros(n, 1);
    
    for i = 1:n
        candidates = randperm(population_size, 2);
        if fitness(candidates(1), 1) < fitness(candidates(2), 1)
            selected_indices(i) = candidates(1);
        else
            selected_indices(i) = candidates(2);
        end
    end
end

function offspring = simulatedBinaryCrossover(parent, boundary)
    % Simulated Binary Crossover
    num_parents = size(parent, 1);
    num_vars = size(parent, 2);
    offspring = zeros(num_parents, num_vars);
    for i = 1:2:num_parents
        if i+1 > num_parents
            break;
        end
        p1 = parent(i, :);
        p2 = parent(i+1, :);
        for j = 1:num_vars
            if rand < 0.5
                beta = rand;
                offspring(i, j) = 0.5 * ((1 + beta) * p1(j) + (1 - beta) * p2(j));
                offspring(i+1, j) = 0.5 * ((1 - beta) * p1(j) + (1 + beta) * p2(j));
            else
                offspring(i, j) = p1(j);
                offspring(i+1, j) = p2(j);
            end
        end
    end
    offspring = max(min(offspring, boundary(2, :)), boundary(1, :));
end

function mutated_offspring = polynomialMutation(offspring, boundary)
    % Polynomial Mutation
    [num_offspring, num_vars] = size(offspring);
    mutation_rate = 1/num_vars;
    eta_m = 20;
    mutated_offspring = offspring;
    
    for i = 1:num_offspring
        for j = 1:num_vars
            if rand < mutation_rate
                delta = rand;
                if delta < 0.5
                    delta_q = (2 * delta)^(1/(eta_m + 1)) - 1;
                else
                    delta_q = 1 - (2 * (1 - delta))^(1/(eta_m + 1));
                end
                mutated_offspring(i, j) = offspring(i, j) + delta_q * (boundary(2, j) - boundary(1, j));
                mutated_offspring(i, j) = min(max(mutated_offspring(i, j), boundary(1, j)), boundary(2, j));
            end
        end
    end
end

function selected_indices = nsgaSelection(population, rank_combined, crowd_distances_combined, n)
    % NSGA-II Selection
    [~, sorted_idx] = sortrows([rank_combined, -crowd_distances_combined], [1, 2]);
    selected_indices = sorted_idx(1:n);
end

function normalized_data = normalize(data)
    % Normalize data to range [0, 1]
    min_val = min(data(:));
    max_val = max(data(:));
    normalized_data = (data - min_val) / (max_val - min_val);
end
