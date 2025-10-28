num_generations = 100;
n = 100;

clf
rs = linspace(0, 4, 100);
for ridx = 1:length(rs)
    r = rs(ridx);
    %%
    p = rand(1, n);
    for i = 1:num_generations
        p = r * p .* (1 - p);
        % Store the population value for analysis
        hold on
    end
    plot(r, p, 'o', 'MarkerEdgeColor', 'none', 'MarkerFaceColor', 'k');
    ylim([0, 1])
    drawnow
end
hold off