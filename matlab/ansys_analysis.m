%% Global setup

close all; clear all; clc
warning('off','all')

% fig sizes and scale factor
fig_w = 300; fig_h = 300; fig_s = 4;

% fonts
ax_font_size = 5*fig_s;
legend_font_size = 5*fig_s;
set(0,'DefaultTextFontname', 'CMU Sans Serif' )
set(0,'DefaultAxesFontName', 'CMU Sans Serif' )

% colors
map = brewermap(9,'Set1');

% save path
export_fig = true;

%% Load in and plot truss data

cell_size = 28:14:112;

files = strcat(repmat("D",1,7), string(cell_size), repmat("_sim_results.csv",1,7));
truss = load_data('./ansys/truss/',files);

twist_to_bend = zeros(4,length(cell_size));

for i = 1:length(truss)
    sample = truss{i};
    for j = 1:4
        twist_to_bend(j,i) = (sample.TwistMoment_Nmm_(j)./sample.Twist(j))./(sample.BendMoment_Nmm_(j)./sample.Bend(j)); 
    end
end

fig = figure(1); clf; hold on; box on; grid on;
% Ansys data
plot(cell_size./56,twist_to_bend(2,:),'-.','marker',"o","MarkerFaceColor",map(2,:),"MarkerEdgeColor",map(2,:),"Color",map(2,:),'lineWidth',2);
plot(cell_size./56,twist_to_bend(3,:),'-.','marker',"^","MarkerFaceColor",map(3,:),"MarkerEdgeColor",map(3,:),"Color",map(3,:),'lineWidth',2);
plot(cell_size./56,twist_to_bend(4,:),'-.','marker',"square","MarkerFaceColor",map(1,:),"MarkerEdgeColor",map(1,:),"Color",map(1,:),'lineWidth',2);

ylim([0,80])

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% Experimental data
scatter(56./56,52.20,'filled',"pentagram",'MarkerEdgeColor','k','SizeData',100,'LineWidth',2,"MarkerFaceColor","k");


% export fig
if export_fig
    exportgraphics(gcf,'../figures/ansys/ansys_truss.png','Resolution',300*fig_s)
end

%% Load in and plot equitorial data

cell_size = 28:14:112;

files = strcat(repmat("D",1,7), string(cell_size), repmat("_sim_results.csv",1,7));
equitorial = load_data('./ansys/equitorial/',files);

twist_to_bend = zeros(4,length(cell_size));

for i = 1:length(equitorial)
    sample = equitorial{i};
    for j = 1:4
        twist_to_bend(j,i) = (sample.TwistMoment_Nmm_(j)./sample.Twist(j))./(sample.BendMoment_Nmm_(j)./sample.Bend(j)); 
    end
end

fig = figure(3); clf; hold on; grid on; box on;
% Ansys data
plot(cell_size./56,twist_to_bend(2,:),'-.','marker',"o","MarkerFaceColor",map(2,:),"MarkerEdgeColor",map(2,:),"Color",map(2,:),'lineWidth',2);
plot(cell_size./56,twist_to_bend(3,:),'-.','marker',"^","MarkerFaceColor",map(3,:),"MarkerEdgeColor",map(3,:),"Color",map(3,:),'lineWidth',2);
plot(cell_size./56,twist_to_bend(4,:),'-.','marker',"square","MarkerFaceColor",map(1,:),"MarkerEdgeColor",map(1,:),"Color",map(1,:),'lineWidth',2);

% Experimental data
scatter(98./56,11.34,'filled',"pentagram",'MarkerEdgeColor','k','SizeData',100,'LineWidth',2,"MarkerFaceColor","k");

ylim([0,80])

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 1.25;
height = 1;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);

% export fig
if export_fig
    exportgraphics(gcf,'../figures/ansys/ansys_equitorial.png','Resolution',300*fig_s)
end


%% Helper functions


function [allData] = load_data(folder,files)

    % Initialize to store data
    allData = {};

    % Collect all data
    for i = 1:length(files)

        csvFile = readtable(folder + files(i));

     

        allData{i} = csvFile;
    end

end




