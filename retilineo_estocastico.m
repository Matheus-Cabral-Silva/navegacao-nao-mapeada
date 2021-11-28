clc;
clear all;

%% Robô de movimento retilíneo-estocástico
% Matheus Cabral da Silva
% 2021

[minAngle(graus) maxAngle(graus) iniX(m) iniY(m) iniAngle(rad) Rounds]
conditions = [  0 120 0.3 0.3 0 10;
                30 120 0.3 0.3 0 10;
                60 120 0.3 0.3 0 10;
                90 120 0.3 0.3 0 10;
                0 90 0.3 0.3 0 10;
                0 60 0.3 0.3 0 10
             ];

        
startFrom = 1;
startRound = 1;
folder = 'comum_teste_ret_est_final';


sizeConditions = size(conditions);
numConditions = sizeConditions(1,1);

for conditionLoop = startFrom:numConditions 
    
    for roundLoop = startRound:conditions(conditionLoop,6)
        %% Criando ambiente
        load('sala_comum.mat')
        %load('sala_vazia.mat')
        heatmapMat = double(occupancyMatrix(map));
        [heatmapMatNumRows,heatmapMatNumCols] = size(heatmapMat);
        cleanableArea = 0;
        percentageCleaned = [];
        instant = [];
        for i = 1:heatmapMatNumRows
            for j = 1:heatmapMatNumCols
                if heatmapMat(i,j) == 1
                    heatmapMat(i,j) = -10;
                else
                    cleanableArea = cleanableArea + 1;
                end
            end
        end

        %% Criando Sensor
        numberOfSensors = 11;       % Número de sensores

        lidar = LidarSensor;
        lidar.sensorOffset = [0,0];
        lidar.scanAngles = linspace(-pi/3,pi/3,numberOfSensors);
        lidar.maxRange = 0.3;
        extraSpace = 0.05; %Espaço entre robô e obstáculo para detecção

        %% Criando visualização
        viz = Visualizer2D;
        viz.hasWaypoints = false;
        viz.mapName = 'map';
        attachLidarSensor(viz,lidar);

        %% Definindo o veículo
        R = 0.036;                    % Raio da roda [m]
        L = 0.235;                   % Tamanho do eixo [m]
        MaxV = 0.4;                % Velocidade máxima[m/s]
        MaxW = 0.8;                   % Velocidade máxima de rotação [rad/s]
        minimumRotationAngle = conditions(conditionLoop,1);   % Ângulo mínimo em caso de colisão [Degrees]
        maximumRotationAngle = conditions(conditionLoop,2); % Ângulo máximo em caso de colisão [Degrees]

        viz.robotRadius = 0.17425;  
        dd = DifferentialDrive(R,L);

        %% Parâmetros da simulação
        sampleTime = 0.05;              % Tempo de amostragem [s]
        simulationTime = 900;            % Tempo de simulação [s]
        initialAngle = conditions(conditionLoop,5); % Ângulo inicial
        
        tVec = 0:sampleTime:simulationTime;   
        initPose = [conditions(conditionLoop,3); conditions(conditionLoop,4); initialAngle]; % Posição inicial (x y theta)
        pose = zeros(3,numel(tVec));    % Matriz de posição
        pose(:,1) = initPose;

        %% Laço de simulação
        close all
        r = rateControl(1/sampleTime);

        vRef = MaxV;
        wRef = 0;
        cooldown = 0;
        ranges = NaN(1,3,'single');
        desiredAngle = initialAngle;
        clockwiseRotation = false;
   
        for idx = 2:numel(tVec)
            %Computando colisão
            rightMinimumRange = min(ranges(2:floor(length(ranges)/2)));
            if mod(numberOfSensors,2) == 0  
                leftMinimumRange = min(ranges(floor(length(ranges)/2) + 1:length(ranges)-1));
            else
                leftMinimumRange = min(ranges(ceil(length(ranges)/2) + 1:length(ranges)-1));
            end

            currentAngle = pose(3,idx-1)*180/pi;
            if cooldown <= 0
                if any(ranges <= viz.robotRadius + extraSpace)
                    vRef = 0;
                    angleIncrementDegrees = (minimumRotationAngle + (maximumRotationAngle-minimumRotationAngle)*rand);
                    angleIncrementRadians = angleIncrementDegrees*pi/180;
                    cooldown = 1/MaxW * angleIncrementRadians;
                    if round(rightMinimumRange,4) > round(leftMinimumRange,4) || (isnan(rightMinimumRange)) && not(isnan(leftMinimumRange))
                        %Rotação horária
                        wRef = -MaxW;
                        clockwiseRotation = true;
                    else
                        %Rotação anti-horária
                        wRef = MaxW;
                        clockwiseRotation = false;
                    end
                else
                    vRef = MaxV;
                    wRef = 0;
                end
            else
                cooldown = cooldown - sampleTime;
            end

            %Cinemática inversa
            [wL,wR] = inverseKinematics(dd,vRef,wRef);

            %Cinemática direta
            [v,w] = forwardKinematics(dd,wL,wR);
            velB = [v;0;w]; % Body velocities [vx;vy;w]
            vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world

            % Realiza um passo de integração discreto
            pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

            %% Atualiza heatmap
            if vRef ~= 0
                positionRow = heatmapMatNumRows - round(pose(2,idx)*100);
                positionColumn = round(pose(1,idx)*100);
                heatmapMat(positionRow,positionColumn) =  heatmapMat(positionRow,positionColumn) + 10;
                heatmapAuxMat = zeros(heatmapMatNumRows, heatmapMatNumCols);
                for i = 1:0.5:100*viz.robotRadius
                    for j = 0:0.01:2*pi
                        CircleRow = positionRow + round(i*sin(j));
                        CircleColumn = positionColumn + round(i*cos(j));
                        if (CircleRow > 1 && CircleColumn > 1) && (CircleRow <= heatmapMatNumRows && CircleColumn <= heatmapMatNumCols)
                            if heatmapAuxMat(CircleRow, CircleColumn) >= 0
                                heatmapAuxMat(CircleRow, CircleColumn) = 10;
                            end
                        end
                    end
                end
                heatmapMat = plus(heatmapMat,heatmapAuxMat);
            end


            %% Análise de eficiência
            cleanedArea = 0;
            for i = 1:heatmapMatNumRows
                for j = 1:heatmapMatNumCols
                    if heatmapMat(i,j) > 0
                        cleanedArea = cleanedArea + 1;
                    end
                end
            end

            percentageCleaned = [percentageCleaned, 100* cleanedArea / cleanableArea];
            instant = [instant, (idx-2) * sampleTime];


            %% Atualiza visualização
            ranges = lidar(pose(:,idx));
            viz(pose(:,idx),ranges)
            grid on;
            waitfor(r);
        end
        %% Heatmap

        % Obtenção do maior valor
        biggestValue = 0;
        heatmapSize = size(heatmapMat);
        for i = 1:heatmapSize(1,1)
            for j = 1:heatmapSize(1,2)
                if heatmapMat(i,j) > biggestValue
                    biggestValue = heatmapMat(i,j);
                end
            end    
        end

        % Compressão do heatmap
        for i = 1:heatmapSize(1,1)
            for j = 1:heatmapSize(1,2)
                    heatmapMat(i,j) = heatmapMat(i,j)/biggestValue;
            end    
        end

        strCondition = num2str(conditionLoop);
        strRound = num2str(roundLoop);

        fileAdress = ['/' folder '/mov_' strCondition '_' strRound '.fig'];
        saveas(gca,[pwd fileAdress]);
        
        fileAdress = ['/' folder '/mov_' strCondition '_' strRound '.png'];
        saveas(gca,[pwd fileAdress]);

        figure;
        heatmap(heatmapMat);
        Ax = gca;
        Ax.XDisplayLabels = nan(size(Ax.XDisplayData));
        Ax.YDisplayLabels = nan(size(Ax.YDisplayData));
        title('Heatmap de movimento');
        grid off;

        fileAdress = ['/' folder '/heatmap_' strCondition '_' strRound '.fig'];
        saveas(gca,[pwd fileAdress]);
        
        fileAdress = ['/' folder '/heatmap_' strCondition '_' strRound '.png'];
        saveas(gca,[pwd fileAdress]);


        figure;
        plot(instant,percentageCleaned);
        xlabel('Tempo');
        ylabel('Área Limpa %');
        title('Área limpa em função do tempo');

        fileAdress = ['/' folder '/area_' strCondition '_' strRound '.fig'];
        saveas(gca,[pwd fileAdress]);
        
        fileAdress = ['/' folder '/area_' strCondition '_' strRound '.png'];
        saveas(gca,[pwd fileAdress]);
        
        
        %% Salvando dados
       
        cinquenta = NaN;
        sessenta = NaN;
        setenta = NaN;
        oitenta = NaN;
        noventa = NaN;
        for i = 1:size(instant,2)
            if percentageCleaned(1,i) >= 50 && isnan(cinquenta)
                cinquenta = instant(1,i);
            elseif percentageCleaned(1,i) >= 60 && isnan(sessenta)
                sessenta = instant(1,i);                
            elseif percentageCleaned(1,i) >= 70 && isnan(setenta)
                setenta = instant(1,i);
            elseif percentageCleaned(1,i) >= 80 && isnan(oitenta)
                oitenta = instant(1,i);
            elseif percentageCleaned(1,i) >= 90 && isnan(noventa)
                noventa = instant(1,i);
            end
        end 
        
        finalCleanedArea = percentageCleaned(1,size(instant,2));
        
        specificConditions = conditions(conditionLoop, 1:5);
        fileName = ['dados_' strCondition '_' strRound '.txt'];
        fileAdress = fullfile(folder, fileName);
        fid = fopen(fileAdress, 'wt');
        fprintf(fid, 'Condições:\n');
        fprintf(fid, '%g ', specificConditions);
        fprintf(fid, '\n\n[50%%, 60%%, 70%%, 80%%, 90%%]: [%f %f %f %f %f]', cinquenta, sessenta, setenta, oitenta, noventa);
        fprintf(fid, '\nÁrea limpa final: %f', finalCleanedArea);
        fclose(fid);
        
        fileName = ['variaveis_' strCondition '_' strRound '.mat'];
        fileAdress = fullfile(folder, fileName);
        fileAdress = ['\' fileAdress];
        save([pwd fileAdress],'percentageCleaned','instant','specificConditions', 'cinquenta', 'sessenta', 'setenta', 'oitenta', 'noventa');
        
        close all;
        
        
        %% Limpar variáveis
        clearvars -except conditions startFrom startRound folder sizeConditions numConditions conditionLoop roundLoop;
        
    end
end

