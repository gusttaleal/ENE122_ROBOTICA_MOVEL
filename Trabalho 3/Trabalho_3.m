%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
close all; clear; clc;

% Coordenadas da posição do robô
Xr = 0; Yr = 0; Tr = 0;

% Coordenadas do objetivo do robô
Xf = 5; Yf =  5; Tf = deg2rad(60);

%{
%Parâmetros de Controle que geram Raizes Complexos
Kp =  0.1;
Ka =  0.4;
Kb = -0.3;
%}

% {
%Parâmetros de Controle que geram Raizes Reais
Kp =  0.1;
Ka =  0.4;
Kb = -0.2;
%}

%Tempo
Time = 0; dTime = 1; flag = true;

while( flag )
    Time = Time + dTime;
    
    %Deltas
    dX = ( Xf - Xr(end) );
    dY = ( Yf - Yr(end) );
    dT = ( Tf - Tr(end) );
    
    % Correção do Delta Teta para o intervalo [ -180° , 180° ]
    dT = mod( dT , 2 * pi );
    if(dT > pi)
        dT = dT - 2 * pi;
    end
    
    %Parâmetros em coordenada polar
    R = sqrt( dX^2 + dY^2 );
    G = atan2( dY , dX );
    A = mod( ( G - Tr ) , 2 * pi ); A = A(end);
    B = mod( ( Tf - G ) , 2 * pi ); B = B(end);
    
    % Correção dos ângulos Alpha (A) e Beta (B) para o intervalo [ -180° , 180° ]
    if(A > pi)
        A = A - ( 2 * pi );
    end
    if(B > pi)
        B = B - ( 2 * pi );
    end
    
    % Velocidade Linear
    % Verificar a posição do objeto e corrige o sentido da velocidade linear
    if ( ( A >= ( - pi / 2 ) ) && ( A <= ( pi / 2 ) ) )
        Vr = Kp * R ;
    else
        Vr = -Kp * R;
        A = mod( ( G - Tr ) + pi , 2 * pi ); A = A(end);
        B = mod( ( Tf - G ) + pi , 2 * pi ); B = B(end);
        
        % Correção dos ângulos Alpha (A) e Beta (B) para o intervalo [ -180° , 180° ]
        if(A > pi)
            A = A - ( 2 * pi );
        end
        if(B > pi)
            B = B - ( 2 * pi );
        end
    end
    
    % Velocidade Angular
    Wr = ( Ka * A ) + ( Kb * B );
    
    % Matriz do Modelo Cinemático Diferencial
    M1 = [ cos(Tr(end)) 0 ; sin(Tr(end)) 0 ; 0 1 ] * [ Vr ; Wr ];
    
    Xr = [Xr, Xr(end) + ( M1(1) * dTime ) ];
    Yr = [Yr, Yr(end) + ( M1(2) * dTime ) ];
    Tr = [Tr, Tr(end) + ( M1(3) * dTime ) ];
    
    if( ( R < 0.01 ) && ( abs(dT) < 0.01 ) )
        flag = false;
    end
    
    %  Plot
    plot_robot([], [], Xr, Yr, Tr, Xf, Yf, Tf, Time, sqrt(Xf^2 + Yf^2));
end