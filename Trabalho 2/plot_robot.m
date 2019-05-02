%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################

function plot_robot(X1, Y1, T1, X2, Y2, T2, T, R)

    % Dimensões do robô
    Solido = [-1, 1, 2, 1, -1, -1.5, -1.5; -1, -1, 0, 1, 1, 0.5, -0.5];
    FrameX = [0 3; 0 0];
    FrameY = [0 0; 0 3];
    
    % Rotação do sólido e dos frames no eixo-z bidimensional
    SolidoRot1 = [ cos(T1(end)) -sin(T1(end)) ; sin(T1(end)) cos(T1(end)) ] * Solido;
    FrameXRot1 = [ cos(T1(end)) -sin(T1(end)) ; sin(T1(end)) cos(T1(end)) ] * FrameX;
    FrameYRot1 = [ cos(T1(end)) -sin(T1(end)) ; sin(T1(end)) cos(T1(end)) ] * FrameY;
    
    SolidoRot2 = [ cos(T2(end)) -sin(T2(end)) ; sin(T2(end)) cos(T2(end)) ] * Solido;
    FrameXRot2 = [ cos(T2(end)) -sin(T2(end)) ; sin(T2(end)) cos(T2(end)) ] * FrameX;
    FrameYRot2 = [ cos(T2(end)) -sin(T2(end)) ; sin(T2(end)) cos(T2(end)) ] * FrameY;
    
    % Translação do sólido e dos frames nos eixo-x e eixo-y bidimensional
    A1 = [ X1(end) + SolidoRot1(1,:) ; Y1(end) + SolidoRot1(2,:) ];
    B1 = [ X1(end) + FrameXRot1(1,:) ; Y1(end) + FrameXRot1(2,:) ];
    C1 = [ X1(end) + FrameYRot1(1,:) ; Y1(end) + FrameYRot1(2,:) ];
    
    A2 = [ X2(end) + SolidoRot2(1,:) ; Y2(end) + SolidoRot2(2,:) ];
    B2 = [ X2(end) + FrameXRot2(1,:) ; Y2(end) + FrameXRot2(2,:) ];
    C2 = [ X2(end) + FrameYRot2(1,:) ; Y2(end) + FrameYRot2(2,:) ];
    
    % Plot da curva
    a = figure (1);
    a.Position = [2 42 681 642];
    plot(X1, Y1, '-y', X2, Y2, '-g', 'linewidth', 3)
    grid on; hold on; axis equal; axis([-R-10 R+10 -10 2*R+10]);
    
    xlabel(['Xi: ', num2str(X1(end), '%10.3f'), ' m | ' , 'Xd: ', ...
            num2str(X2(end), '%10.3f'), ' m']);
    ylabel(['Yi: ', num2str(Y1(end), '%10.3f'), ' m | ' , 'Yd: ', ...
            num2str(Y2(end), '%10.3f'), ' m']);
    
    title(['Thi: ', num2str(rad2deg(T1(end)), '%10.3f'), '° | ' , ...
           'Thd: ', num2str(rad2deg(T2(end)), '%10.3f'), '° | ' , ...
           'Tempo: ', num2str(T(end), '%10.3f'), 's']);
   
    
    % Plot do sólido
    fill(A1(1,:), A1(2,:) , 'y', A2(1,:), A2(2,:) , 'g');
    
    % plot do frame
    plot(B1(1,:) , B1(2,:) , 'b', 'linewidth', 2); 
    text(B1(1,2) , B1(2,2) , 'x_{\{ incremental \}}');
    
    plot(C1(1,:) , C1(2,:) , 'r', 'linewidth', 2); 
    text(C1(1,2) , C1(2,2) , 'y_{\{ incremental \}}');
    
    plot(B1(1,1) , B1(2,1) , 'ok', 'linewidth', 5);
    
    plot(B2(1,:) , B2(2,:) , 'b', 'linewidth', 2); 
    text(B2(1,2) , B2(2,2) , 'x_{\{ diferencial \}}');
    
    plot(C2(1,:) , C2(2,:) , 'r', 'linewidth', 2); 
    text(C2(1,2) , C2(2,2) , 'y_{\{ diferencial \}}');
    
    plot(B2(1,1) , B2(2,1) , 'ok', 'linewidth', 5);
    
    drawnow; hold off;
end