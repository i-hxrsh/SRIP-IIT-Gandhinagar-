clear s;
s.units = 'inches';
s.format = 'pdf';
s.Preview= 'none';
s.Width= '16'; % Figure width on canvas
s.Height= '9'; % % Figure height on canvas
s.Units= 'inches';
s.Color= 'rgb';
s.Background= 'w';
s.FixedfontSize= '12';
s.ScaledfontSize= 'auto';
s.FontMode= 'scaled';
s.FontSizeMin= '12';
s.FixedLineWidth= '1';
s.ScaledLineWidth= 'auto';
s.LineMode= 'none';
s.LineWidthMin= '0.1';
s.FontName= 'Times New Roman';% Might change this to something that is available
s.FontWeight= 'auto';
s.FontAngle= 'auto';
s.FontEncoding= 'latin1';
s.PSLevel= '3';
s.Renderer= 'painters';
s.Resolution= '300';
s.LineStyleMap= 'none';
s.ApplyStyle= '0';
s.Bounds= 'tight';
s.LockAxes= 'off';
s.LockAxesTicks= 'off';
s.ShowUI= 'off';
s.SeparateText= 'off';

chosenfigure=gcf;
set(chosenfigure,'PaperUnits','inches');
set(chosenfigure,'PaperPositionMode','auto');
set(chosenfigure,'PaperSize',[16 9]); % Canvas Size
set(chosenfigure,'Units','inches');
hgexport(gcf,'ContribBaseline.pdf',s);

