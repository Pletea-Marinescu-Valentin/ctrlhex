function add_legend(ax, labels, h, loc)
% loc: a legend location, or 'tile-north' / 'tile-south' to place it outside all tiles
if nargin < 4, loc = 'northeast'; end
if nargin < 3 || isempty(h)
    lg = legend(ax, labels);
else
    lg = legend(ax, h, labels);
end
if startsWith(loc, 'tile-')
    lg.Layout.Tile = extractAfter(loc, 'tile-');
else
    lg.Location = loc;
end
if any(strcmp(loc, {'southoutside', 'northoutside', 'south', 'north', 'tile-north', 'tile-south'})), lg.Orientation = 'horizontal'; end
lg.Box = 'off'; lg.TextColor = palette().ink2; lg.FontSize = 8; lg.FontName = plot_font();
lg.ItemTokenSize = [18 8];
end
