function style_axes(ax)
ax.FontName = plot_font(); ax.FontSize = 9; ax.Color = 'w';
ax.Box = 'off'; ax.TickDir = 'out'; ax.LineWidth = 0.75;
ax.XColor = palette().muted; ax.YColor = palette().muted;
ax.XGrid = 'on'; ax.YGrid = 'on'; ax.GridColor = palette().grid; ax.GridAlpha = 1; ax.GridLineStyle = '-';
ax.Title.Color = palette().ink; ax.Title.FontWeight = 'normal'; ax.Title.FontSize = 10;
ax.XLabel.Color = palette().ink2; ax.YLabel.Color = palette().ink2;
end
