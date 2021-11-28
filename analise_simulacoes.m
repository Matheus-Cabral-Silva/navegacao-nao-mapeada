close all;
clear all;
% 
% vaz_ret_esto_med = [317.62 330.24 361.69 414.48 319.44 337.67]; 
% vaz_ret_esto_desvPad = [35.34 37.03 59.81 56.93 43.42 55.59];
% errorbar(vaz_ret_esto_med,vaz_ret_esto_desvPad ,'x');
% title('Análise Preliminar: Algoritmo retilíneo-estocástico em sala vazia');
% ylabel('Tempo para conclusão da tarefa (s)');
% xlabel('Configuração');
% axis([0 7 0 900]);
% 
% figure;
% 
% vaz_dupl_esto_med = [390.22 443.67 433.69 493.24 456.32]; 
% vaz_dupl_esto_desvPad = [22.24 39.02 47.84 39.89 38.57];
% errorbar(vaz_dupl_esto_med,vaz_dupl_esto_desvPad ,'x');
% title('Análise Preliminar: Algoritmo duplamente-estocástico em sala vazia');
% ylabel('Tempo para conclusão da tarefa (s)');
% xlabel('Configuração');
% axis([0 6 0 900]);
% 
% figure;
% 
% vaz_caot_esto_med = [366.58 369.97 397.66 376.27 373.24 NaN 528.91 354.89 383.21 348.84 374.24 352.24 469.18]; 
% vaz_caot_esto_desvPad = [30.02 42.92 198.42 81.89 45.75 NaN 112.13 34.06 40.61 30.62 32.46 38.85 90.34];
% errorbar(vaz_caot_esto_med,vaz_caot_esto_desvPad ,'x');
% title('Análise Preliminar: Algoritmo caótico-estocástico em sala vazia');
% ylabel('Tempo para conclusão da tarefa (s)');
% xlabel('Configuração');
% xticks([0:1:14]);
% axis([0 14 0 900]);
% 
% figure;
% 
% com_ret_esto_med = [518.89 541.38 751.95 NaN 494.44 516.71]; 
% com_ret_esto_desvPad = [175.49 154.55 78.70 NaN 185.48 124.36];
% errorbar(com_ret_esto_med,com_ret_esto_desvPad ,'x');
% title('Análise Preliminar: Algoritmo retilíneo-estocástico em sala comum');
% ylabel('Tempo para conclusão da tarefa (s)');
% xlabel('Configuração');
% axis([0 7 0 900]);
% 
% figure;
% 
% com_dupl_esto_med = [641.32 672.41 600.26 632.72 616.42]; 
% com_dupl_esto_desvPad = [171.35 142.18 167.60 144.48 132.50];
% errorbar(com_dupl_esto_med,com_dupl_esto_desvPad ,'x');
% title('Análise Preliminar: Algoritmo duplamente-estocástico em sala comum');
% ylabel('Tempo para conclusão da tarefa (s)');
% xlabel('Configuração');
% axis([0 6 0 900]);
% 
% figure;
% 
% com_caot_esto_med = [616.98 575.91 651.01 582.68 719.97 576.40 650.07 631.72 686.36 627.77 586.69 551.79 646.74]; 
% com_caot_esto_desvPad = [132.20 159.66 129.80 129.86 130.38 95.34 97.87 158.21 200.63 136.73 75.20 144.56 130.00];
% errorbar(com_caot_esto_med,com_caot_esto_desvPad ,'x');
% title('Análise Preliminar: Algoritmo caótico-estocástico em sala comum');
% ylabel('Tempo para conclusão da tarefa (s)');
% xlabel('Configuração');
% xticks([0:1:14]);
% axis([0 14 0 900]);
% 
% figure;
% 
% com_ret_esto_concl = [90 70 60 0 80 70];
% bar(com_ret_esto_concl);
% title('Conclusão de tarefa: Algoritmo retilíneo-estocástico em sala comum');
% ylabel('Taxa de conclusão da tarefa (%)');
% xlabel('Configuração');
% axis([0 7 0 100]);
% 
% figure;
% 
% com_dupl_esto_concl = [90 70 90 50 70];
% bar(com_dupl_esto_concl);
% title('Conclusão de tarefa: Algoritmo duplamente-estocástico em sala comum');
% ylabel('Taxa de conclusão da tarefa (%)');
% xlabel('Configuração');
% axis([0 6 0 100]);
% 
% figure;
% 
% com_caot_esto_concl = [70 80 80 80 100 60 90 90 70 80 70 60 90];
% bar(com_caot_esto_concl);
% title('Conclusão de tarefa: Algoritmo caótico-estocástico em sala comum');
% ylabel('Taxa de conclusão da tarefa (%)');
% xlabel('Configuração');
% xticks([0:1:14]);
% axis([0 14 0 100]);
% 
% 


% final_vaz = [331.86 396.81 371.07]; 
% final_vaz_desvPad = [40.67 44.02 52.85];
% errorbar(final_vaz,final_vaz_desvPad ,'x');
% title('Análise de aprofundamento: Algoritmos estocásticos em sala vazia');
% ylabel('Tempo para conclusão da tarefa (s)');
% xlabel('Algoritmo');
% xticks([1 2 3])
% xticklabels({'Retilíneo estocástico','Duplamente estocástico','Caótico estocástico'})
% axis([0 4 0 900]);
% 
% final_com = [577.66 596.25 510.62]; 
% final_com_desvPad = [159.40 145.50 144.43];
% errorbar(final_com,final_com_desvPad ,'x');
% title('Análise de aprofundamento: Algoritmos estocásticos em sala comum');
% ylabel('Tempo para conclusão da tarefa (s)');
% xlabel('Algoritmo');
% xticks([1 2 3])
% xticklabels({'Retilíneo estocástico','Duplamente estocástico','Caótico estocástico'})
% axis([0 4 0 900]);

final_com_concl = [88 91 87];
bar(final_com_concl);
title('Conclusão de tarefa: Algoritmos caóticos em sala comum');
ylabel('Taxa de conclusão da tarefa (%)');
xlabel('Algoritmo');
xticks([1 2 3])
xticklabels({'Retilíneo estocástico','Duplamente estocástico','Caótico estocástico'})
axis([0 4 0 100]);
% 
