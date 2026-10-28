function save_figure(f, folder, name, enabled)
if ~enabled, return; end
for a = findall(f, 'Type', 'axes')', if ~isempty(a.Toolbar), a.Toolbar.Visible = 'off'; end, end
exportgraphics(f, fullfile(folder, [name '.png']), 'Resolution', 200);
exportgraphics(f, fullfile(folder, [name '.pdf']), 'ContentType', 'vector');
end
