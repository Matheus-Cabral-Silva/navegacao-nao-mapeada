clc;
clear all;

heatmapAuxMat = zeros(300);
radius = 15;
posicaoLin = 150;
posicaoCol = 150;

for i = 1:0.5:radius
    for j = 0:0.1/radius:2*pi
        linha = round(i*sin(j));
        coluna = round(i*cos(j));
        if posicaoLin+linha > 1 && posicaoCol+coluna > 1 
            if mat(posicaoLin+linha, posicaoCol+coluna) >= 0 
                mat(posicaoLin+linha, posicaoCol+coluna) = 1;
            end
        end
        
    end
end

heatmap(mat);
Ax = gca;
Ax.XDisplayLabels = nan(size(Ax.XDisplayData));
Ax.YDisplayLabels = nan(size(Ax.YDisplayData));
grid off;