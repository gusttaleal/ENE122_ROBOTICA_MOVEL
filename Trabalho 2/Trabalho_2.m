%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################

close all; clear all; clc;

% Escala de tempo dT
dT = 1; % [ 1E0 ]

% Raio da curva - Para uma reta o raio R é infinito (inf).
R = 10; % [ m ]

% Velocidade linear do robô
V = 1; % [ m / s ]

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

% Vetores para armazenar os pontos P(x,y) e o teta da curva 
X1 = [0]; Y1 = [0]; T1 = [0];
X2 = [0]; Y2 = [5]; T2 = [0];

disp('Diminuindo o valor da escala de tempo ''dT'' aumentamos a acurácia e precisão');
disp('Pressione ENTER para continuar ...');
    

if(R == inf)
    
    R = 20;
    plot_robot(X1, Y1, T1, X2, Y2, T2, 0, R);
    pause();
    %De W = V/R e W = Th/t, temos que t = ( R * 2 * pi / V ), Th = 2 * pi   
    for T = 0:dT:R
        
        MI = double( subs( M_I , [ xk_1 yk_1 thk_1 dt ] , [ X1(end) Y1(end) T1(end) dT ] ));
        X1 = [X1, MI(1)];
        Y1 = [Y1, MI(2)];
        T1 = [T1, MI(3)];
        
        MD = double( subs( M_D , thk_1 , ( W * T ) ));
        X2 = [X2, X2(end) + ( MD(1) * dT ) ];
        Y2 = [Y2, Y2(end) + ( MD(2) * dT ) ];
        T2 = [T2, T2(end) + ( MD(3) * dT ) ];
        
        plot_robot(X1, Y1, T1, X2, Y2, T2, T, R);
    end
else
    
    plot_robot(X1, Y1, T1, X2, Y2, T2, 0, R);
    pause();
    
    %De W = V/R e W = Th/t, temos que t = ( R * 2 * pi / V ), Th = 2 * pi       
    for T = 0:dT:( R * 2 * pi / V )
        
        MI = double( subs( M_I , [ xk_1 yk_1 thk_1 dt ] , [ X1(end) Y1(end) T1(end) dT ] ));
        X1 = [X1, MI(1)];
        Y1 = [Y1, MI(2)];
        T1 = [T1, MI(3)];
        
        MD = double( subs( M_D , thk_1 , ( W * T ) ));
        X2 = [X2, X2(end) + ( MD(1) * dT ) ];
        Y2 = [Y2, Y2(end) + ( MD(2) * dT ) ];
        T2 = [T2, T2(end) + ( MD(3) * dT ) ];
        
        plot_robot(X1, Y1, T1, X2, Y2, T2, T, R);
    end
end

disp('Fim.');