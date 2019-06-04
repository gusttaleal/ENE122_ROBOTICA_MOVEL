%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
close all; clear; clc;
% Coordenadas da posição do robô
Xr = 0; Yr = 0; Tr = deg2rad(180);

% Coordenadas do objetivo do robô
Xf = 5; Yf =  5;

% Parâmetros de Controle
Csi = 0.1;
K = zeros(1,3);
dK = ones(1,length(K));

% Tempo máximo de simulação
Tmax = 100;

% Melhor Erro
dEbest = PID_controller(false, K, Tmax, Xr, Yr, Tr, Xf, Yf);

while(sum(dK) > 1e-6)
    for i = 1:length(K)
        K(i) = K(i) + dK(i);
        dE = PID_controller(false, K, Tmax, Xr, Yr, Tr, Xf, Yf);
        
        if( dE < dEbest)
            dEbest = dE;
            dK(i) = dK(i) * ( 1 + Csi );
            
        else
            K(i) = K(i) - ( 2 * dK(i) );
            dE = PID_controller(false, K, Tmax, Xr, Yr, Tr, Xf, Yf);
            
            if( dE < dEbest)
                dEbest = dE;
                dK(i) = dK(i) * ( 1 + Csi );
                
            else
                K(i) = K(i) + dK(i);
                dK(i) = dK(i) * ( 1 - Csi );
                
            end
        end
    end
fprintf('K1: %.3f\tK2: %.3f\tK3: %.3f\tE:%.6f\n',K(1),K(2),K(3),dEbest);
end
PID_controller(true, K, Tmax, Xr, Yr, Tr, Xf, Yf);
