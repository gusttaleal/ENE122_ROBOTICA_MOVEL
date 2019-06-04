%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
function dE = PID_controller(S, K, Tmax, Xr, Yr, Tr, Xf, Yf)
    Time = 0; dTime = 1; Alpha = []; flag = true;      

    % Coeficientes da reta
    a = (Yr - Yf);
    b = (Xf - Xr);
    c = (Xr*Yf - Xf*Yr);

    % C�lculo do erro
    E = abs( ( a*Xr(end) + b*Yr(end) + c ) ) / sqrt(a^2 + b^2);
    dE = [];
    dE = [dE , E];
    
    % Velocidades m�ximas do rob�
    Vmax = 0.2 / dTime;
    Wmax = deg2rad(45); %pi / ( 2 * dTime ) ;

    while( flag )
        %  Plot
        if(S == true)
            if( Xf < 0 && Yf < 0 )
            plot_robot(Xf:0, Yf:0, dE, Xr, Yr, Tr, Xf, Yf, deg2rad(225), Time, sqrt(Xf^2 + Yf^2));
            elseif(Xf < 0 && Yf > 0)
                plot_robot(Xf:0, abs(-Yf:0), dE, Xr, Yr, Tr, Xf, Yf, deg2rad(135), Time, sqrt(Xf^2 + Yf^2));
            elseif(Xf > 0 && Yf < 0)
                plot_robot(0:Xf, -(0:abs(Yf)), dE, Xr, Yr, Tr, Xf, Yf, deg2rad(315), Time, sqrt(Xf^2 + Yf^2));
            else
                plot_robot(0:Xf, 0:Yf, dE, Xr, Yr, Tr, Xf, Yf, deg2rad(45), Time, sqrt(Xf^2 + Yf^2));
            end
        end

        % Erro das medidas
        E = abs( ( a*Xr(end) + b*Yr(end) + c ) ) / sqrt(a^2 + b^2);
        dE = [dE , E];

        %Deltas
        dX = ( Xf - Xr(end) );
        dY = ( Yf - Yr(end) );

        % Par�metros em coordenada polar
        R = sqrt( dX^2 + dY^2 );
        G = atan2( dY , dX );
        A = mod( ( G - Tr(end) ) , 2 * pi );

        % Corre��o do �ngulo alpha para o intervalo [ -180� , 180� ]
        if( A > pi )
            A = A - 2 * pi;
        end

        %     % Verificar a posi��o do objeto e permite o movimento de marcha r�
        %     if ( ( A >= ( - pi / 2 ) ) && ( A <= ( pi / 2 ) ) )
        %         % Velocidade Linear
        %         Vr = min( (R / dTime) , (0.2 / dTime) );
        %     else
        %         % Velocidade Linear
        %         Vr = max( (-R / dTime) , (-0.2 / dTime) );
        %
        %         % Corre��o do �ngulo Alpha (A) para o intervalo [ -180� , 180� ]
        %         A = mod( ( G - Tr(end) ) + pi , 2 * pi );
        %     end

        Alpha = [Alpha, A];

        % Velocidade Linear
        Vr = min( (R / dTime) , Vmax );

        % Velocidade Angular
        if( length(Alpha) > 1 )
            Wr = ( K(1) * A ) + ( K(2) * sum(Alpha) ) + ( K(3) * ( A - Alpha(end-1) ) );
            Wr = sign(Wr) * min( abs( Wr ) , Wmax );
        else
            Wr = ( K(1) * A ) + ( K(2) * sum(Alpha) ) + ( K(3) * A );
            Wr = sign(Wr) * min( abs( Wr ) , 0.01 );
        end

        % Matriz do Modelo Cinem�tico Diferencial
        M1 = [ cos(Tr(end)) 0 ; sin(Tr(end)) 0 ; 0 1 ] * [ Vr ; Wr ];

        Xr = [Xr, Xr(end) + ( M1(1) * dTime ) ];
        Yr = [Yr, Yr(end) + ( M1(2) * dTime ) ];
        Tr = [Tr, Tr(end) + ( M1(3) * dTime ) ];

        % Corre��o do �ngulo Tr para o intervalo [ -180� , 180� ]
        Tr(end) = mod( Tr(end) , 2 * pi );
        if( Tr(end) > pi )
            Tr(end) = Tr(end) - 2 * pi;
        end
        
        % Condi��o de parada
        if((R < 0.01) || (Time >= Tmax))
            flag = false;
        end
        
        % Tempo de simula��o
        Time = Time + dTime;
    end
    % M�dia do erro
    dE = mean(dE);
end
