close all; clc; clear all;

rng(5);
dr_max = 60; % elbow and shoulder
dr_max_wrist = 80; % wrist rotation limit
dl_max = 60; % limit for extension
n = 1000;

%% Define sweep over the configuration space

dl = -dl_max.*rand(n,1);

% Wrist
dl_wrist = -dr_max_wrist.*rand(n, 3);
for i = 1:n
    zeroIndex = randi(3);
    dl_wrist(i, zeroIndex) = 0; % Randomly zero out to prevent a length change
end

% Elbow
dl_elbow = -dr_max.*rand(n, 3);
for i = 1:n
    zeroIndex = randi(3);
    dl_elbow(i, zeroIndex) = 0; % Randomly zero out to prevent a length change
end

% Shoulder ane elbow are coupled
dl_shoulder = dl_elbow;

dl_offset = (max(abs(dl_wrist)) + max(abs(dl_elbow)) + max(abs(dl_shoulder)))/8;
dl = dl + dl_offset;

% Anti-slackening compensation
dl_elbow = dl_elbow + dl_shoulder;
dl_wrist = dl_wrist + 0.75.*dl_elbow;

% Add in compression values
dl_wrist = dl_wrist + dl;
dl_elbow = dl_elbow + (5/7).*dl;
dl_shoulder = dl_shoulder + (3/7).*dl;

lengths_sweep = [dl_wrist(:,1),dl_elbow(:,1),dl_shoulder(:,1),...
                dl_wrist(:,2),dl_elbow(:,2),dl_shoulder(:,2),...
                dl_wrist(:,3),dl_elbow(:,3),dl_shoulder(:,3)];

%% Greedy solution to TSP
map = brewermap(9,'Set1');

delta_fast = zeros(size(lengths_sweep));
delta_fast(1,:) = lengths_sweep(1,:);

% Keep track of which answers to exclude
exclude = true(size(delta_fast,1),1);
exclude(1) = false;

for i = 1:size(delta_fast,1)-1
    % Keep track of original indices
    original_idx = find(exclude);
    
    delta_trim = lengths_sweep(exclude,:);
    idx = knnsearch(delta_trim,delta_fast(i,:));
    
    % Update indices
    original_idx_selected = original_idx(idx);

    delta_fast(i+1,:) = lengths_sweep(original_idx_selected,:);
    exclude(original_idx_selected) = false;
end

fast_diff = sqrt(sum(diff(delta_fast).^2,2));
regular_diff = sqrt(sum(diff(lengths_sweep).^2,2));

mean(fast_diff)
mean(regular_diff)

fig = figure(1); clf; hold on;
set(gcf,'color','w');
set(gcf,'position',[0,0,600,600])
set(gca,'fontsize',14);
ax = gca;
grid on; grid minor;
set(get(fig,'CurrentAxes'),'GridAlpha',1,'MinorGridAlpha',0.7);
plot(fast_diff,'lineWidth',3,'Color',map(1,:));
plot(regular_diff,'lineWidth',3,'Color',map(2,:));
legend(["Greedy KNN","random"])
xlabel("Data point index",fontSize=14)
ylabel("Total tool displacment",fontSize=14)

% Save output
if (length(delta_fast) < 5000)
    save("./trajectory/delta_fast_repeat.mat","delta_fast")
    writematrix(delta_fast,"./trajectory/delta_fast_repeat.csv")
else
    save("./trajectory/delta_fast.mat","delta_fast")
    writematrix(delta_fast,"./trajectory/delta_fast.csv")
end