%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################

function plot_robot(X, Y, dE, X1, Y1, T1, X2, Y2, T2, Time, Radius)

    % Dimensões do robô
    Solido = 0.5 * [-1, 1, 2, 1, -1, -1.5, -1.5; -1, -1, 0, 1, 1, 1, -1];
    Roda_L = 0.5 * [-1 0 0 -1 -1; 1 1 1.5 1.5 1];
    Roda_R = 0.5 * [-1 0 0 -1 -1; -1 -1 -1.5 -1.5 -1];
    FrameX = 0.5 * [0 3; 0 0];
    FrameY = 0.5 * [0 0; 0 3];
    
    % Rotação do sólido e dos frames no eixo-z bidimensional
    SolidoRot1 = [ cos(T1(end)) -sin(T1(end)) ; sin(T1(end)) cos(T1(end)) ] * Solido;
    Roda_L_Rt1 = [ cos(T1(end)) -sin(T1(end)) ; sin(T1(end)) cos(T1(end)) ] * Roda_L;
    Roda_R_Rt1 = [ cos(T1(end)) -sin(T1(end)) ; sin(T1(end)) cos(T1(end)) ] * Roda_R;
    FrameXRot1 = [ cos(T1(end)) -sin(T1(end)) ; sin(T1(end)) cos(T1(end)) ] * FrameX;
    FrameYRot1 = [ cos(T1(end)) -sin(T1(end)) ; sin(T1(end)) cos(T1(end)) ] * FrameY;
    
    SolidoRot2 = [ cos(T2(end)) -sin(T2(end)) ; sin(T2(end)) cos(T2(end)) ] * Solido;
    Roda_L_Rt2 = [ cos(T2(end)) -sin(T2(end)) ; sin(T2(end)) cos(T2(end)) ] * Roda_L;
    Roda_R_Rt2 = [ cos(T2(end)) -sin(T2(end)) ; sin(T2(end)) cos(T2(end)) ] * Roda_R;
    FrameXRot2 = [ cos(T2(end)) -sin(T2(end)) ; sin(T2(end)) cos(T2(end)) ] * FrameX;
    FrameYRot2 = [ cos(T2(end)) -sin(T2(end)) ; sin(T2(end)) cos(T2(end)) ] * FrameY;
    
    % Translação do sólido e dos frames nos eixo-x e eixo-y bidimensional
    A1 = [ X1(end) + SolidoRot1(1,:) ; Y1(end) + SolidoRot1(2,:) ];   
    B1 = [ X1(end) + FrameXRot1(1,:) ; Y1(end) + FrameXRot1(2,:) ];
    C1 = [ X1(end) + FrameYRot1(1,:) ; Y1(end) + FrameYRot1(2,:) ];
    
    R1 = [ X1(end) + Roda_L_Rt1(1,:) ; Y1(end) + Roda_L_Rt1(2,:) ];
    R2 = [ X1(end) + Roda_R_Rt1(1,:) ; Y1(end) + Roda_R_Rt1(2,:) ];
    
    A2 = [ X2(end) + SolidoRot2(1,:) ; Y2(end) + SolidoRot2(2,:) ];
    B2 = [ X2(end) + FrameXRot2(1,:) ; Y2(end) + FrameXRot2(2,:) ];
    C2 = [ X2(end) + FrameYRot2(1,:) ; Y2(end) + FrameYRot2(2,:) ];
    
    R3 = [ X2(end) + Roda_L_Rt2(1,:) ; Y2(end) + Roda_L_Rt2(2,:) ];
    R4 = [ X2(end) + Roda_R_Rt2(1,:) ; Y2(end) + Roda_R_Rt2(2,:) ];
    
    % Plot da curva
    a = figure (1);
    a.WindowState = 'maximized';
    subplot(1,2,1)
    plot(X1, Y1, '-b', X2, Y2, '-g', 'linewidth', 3)
    grid on; hold on; axis equal; axis([-Radius-5 Radius+5 -Radius-5 Radius+5]);
    plot(X, Y, 'r', 'linewidth', 1); 
    xlabel(['Xr: ', num2str(X1(end), '%10.3f'), ' [ ud ] | ' , 'Xg: ', ...
            num2str(X2(end), '%10.3f'), ' [ ud ]']);
    ylabel(['Yr: ', num2str(Y1(end), '%10.3f'), ' [ ud ] | ' , 'Yg: ', ...
            num2str(Y2(end), '%10.3f'), ' [ ud ]']);
    
    title(['Thr: ', num2str(rad2deg(T1(end)), '%10.3f'), '° | ' , ...
           'Thg: ', num2str(rad2deg(T2(end)), '%10.3f'), '° | ' , ...
           'Tempo: ', num2str(Time(end), '%10.3f'), ' [ ut ]']);
   
    
    % Plot do sólido
    fill(A1(1,:), A1(2,:) , 'y');
    fill(R1(1,:), R1(2,:) , 'k', R2(1,:), R2(2,:) , 'k');
    
    
    % plot do frame
    plot(B1(1,:) , B1(2,:) , 'b', 'linewidth', 2); 
    text(B1(1,2) , B1(2,2) , 'x');
    
    plot(C1(1,:) , C1(2,:) , 'r', 'linewidth', 2); 
    text(C1(1,2) , C1(2,2) , 'y');
    
    %plot(B1(1,1) , B1(2,1) , 'ok', 'linewidth', 5);
    
    % Plot do sólido
    fill(A2(1,:), A2(2,:) , 'g');
    fill(R3(1,:), R3(2,:) , 'k', R4(1,:), R4(2,:) , 'k');
    
    % plot do frame
    plot(B2(1,:) , B2(2,:) , 'b', 'linewidth', 2); 
    text(B2(1,2) , B2(2,2) , 'x_{\{ Goal \}}');
    
    plot(C2(1,:) , C2(2,:) , 'r', 'linewidth', 2); 
    text(C2(1,2) , C2(2,2) , 'y_{\{ Goal \}}');
    
    %plot(B2(1,1) , B2(2,1) , 'ok', 'linewidth', 5);
    
    drawnow; hold off;
    subplot(1,2,2)
    plot(dE, 'b', 'linewidth', 2);
    grid on; %axis equal;
    
    xlabel(['Tempo: ', num2str(Time(end), '%10.3f')]);
    
    ylabel(['Erro: ', num2str(dE(end), '%10.3f')]);
    
    title(['Erro: ', num2str(rad2deg(dE(end)), '%10.3f'), ' | ' , ...
           'Tempo: ', num2str(Time(end), '%10.3f'), ' [ ut ]']);
end