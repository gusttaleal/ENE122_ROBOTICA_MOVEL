%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################

close all; clear all; clc;

% Escala de tempo dT
dT = 1; % [ 1E0 ]

% Ângulo em radianos a ser percorrido na curva
Th = 2 * pi;

% Raio da curva - Para uma reta o raio R é infinito (inf).
R = 10; % [ m ]

% Velocidade linear do robô
V = 0.1; % [ m / s ]

% Velocidade angular do robô
W = V / R; % [ rad / s ]

% Distância entre a roda e o centro do robô
L = 1; % [ m ]

% Velocidade linear da roda esquerda e direita
if(R == inf)
    Vr = V; % [ m / s ]
    Vl = V; % [ m / s ]
else
    Vr = V * (R + L) / R; % [ m / s ]
    Vl = V * (R - L) / R; % [ m / s ]
end

% Variáveis simbolicas dos modelos cinemáticos
syms xk_1 yk_1 thk_1 dt;

% Matriz do Modelo Cinemático Incremental
M_I = [ xk_1 ; yk_1 ; thk_1 ] + ...
      [ 0.5 * dt * (Vr + Vl) * cos( thk_1 + ( 0.5 * dt * (Vr - Vl) / L ) );
        0.5 * dt * (Vr + Vl) * sin( thk_1 + ( 0.5 * dt * (Vr - Vl) / L ) );
        0.5 * dt * (Vr - Vl) / L  ];

% Matriz do Modelo Cinemático Diferencial
M_D = [ cos(thk_1) 0 ;
        sin(thk_1) 0 ;
        0          1] * [ V ; W ];

% Vetores para armazenar os pontos P(x,y) e o teta da curva e do robô
X  = [ ]; Y  = [ ]; T  = [ ];
XI = [0]; YI = [0]; TI = [0];
XD = [0]; YD = [0]; TD = [0];

% Se o raio da curva for infinito, faça:
if(R == inf)
    R = 20;
    
    % Construção da reta
    for r = 0:1:R
        X = [X, r]; Y = [Y, 0];
    end  
    % Plot da reta
    plot_robot(X, Y, XI, YI, TI, XD, YD, TD, 0, R);
    
    disp('Pressione ENTER para continuar ...');
    pause();
    
    for Time = 0:dT:R
        % Substituição dos valores numericos no modelo incremental
        MI = double(subs( M_I , [ xk_1 yk_1 thk_1 dt ] , [ XI(end) YI(end) TI(end) dT ]));
        % Armazenando os dados gerados pelo modelo
        XI = [XI, MI(1)];   YI = [YI, MI(2)];   TI = [TI, MI(3)];
        
        % Plot do modelo incremental
        plot_robot(X, Y, XI, YI, TI, XD, YD, TD, Time, R);
    end
    for Time = 0:dT:R
        % Substituição dos valores numericos no modelo diferencial
        MD = double( subs( M_D , thk_1 , ( W * Time ) ));
        % Armazenando os dados gerados pelo modelo
        XD = [XD, XD(end) + ( MD(1) * dT ) ];
        YD = [YD, YD(end) + ( MD(2) * dT ) ];
        TD = [TD, TD(end) + ( MD(3) * dT ) ];
        
        % Plot do modelo diferencial
        plot_robot(X, Y, XI, YI, TI, XD, YD, TD, Time, R);
    end
    
% Caso contrário, faça:
else
    
    % Construção da curva
    for th_rad = 0:(pi/180):Th
        x = R*sin(th_rad);
        y = -R*cos(th_rad) + R;
        X = [X, x]; Y = [Y, y]; 
    end
    % Plot da curva
    plot_robot(X, Y, XI, YI, TI, XD, YD, TD, 0, R);
    
    disp('Pressione ENTER para continuar ...');
    pause();
    
    %De W = V/R e W = Th/t, temos que t = ( R * 2 * pi / V ), Th = 2 * pi
    for Time = 1:dT:( R * Th / V ) + dT
        % Substituição dos valores numericos no modelo incremental
        MI = double(subs( M_I , [ xk_1 yk_1 thk_1 dt ] , [ XI(end) YI(end) TI(end) dT ]));
        % Armazenando os dados gerados pelo modelo
        XI = [XI, MI(1)];   YI = [YI, MI(2)];   TI = [TI, MI(3)];
        
        % Plot do modelo incremental
        plot_robot(X, Y, XI, YI, TI, XD, YD, TD, Time, R);

    end
    for Time = 1:dT:( R * Th / V ) + dT
        % Substituição dos valores numericos no modelo diferencial
        MD = double( subs( M_D , thk_1 , ( W * Time ) ));
        % Armazenando os dados gerados pelo modelo
        XD = [XD, XD(end) + ( MD(1) * dT ) ];
        YD = [YD, YD(end) + ( MD(2) * dT ) ];
        TD = [TD, TD(end) + ( MD(3) * dT ) ];
        
        % Plot do modelo diferencial
        plot_robot(X, Y, XI, YI, TI, XD, YD, TD, Time, R);
    end
end

disp('Fim.');