%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################

close all; clear all; clc;

disp('Program started');
vrep=remApi('remoteApi'); % using the proAStotype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

% Inicialização de um Timer para solucionar o problema do getkey
t = timer;
set(t, 'executionMode', 'fixedRate');
set(t,'TimerFcn', 'paraRobo_VREP(vrep, clientID)');
set(t,'Period', 0.1);
start(t);

% Função para capturar uma tecla prescionada no teclado
control = 'a';

if (clientID>-1)
    disp('Connected to remote API server');
    
    % Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage...
        (clientID,'Hello V-REP!',vrep.simx_opmode_oneshot);
    
    % Handle Pioneer P3DX Motors
    [returnCode_Motor_Left, Pioneer_p3dx_leftMotor]=...
        vrep.simxGetObjectHandle...
        (clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    
    [returnCode_Motor_Right, Pioneer_p3dx_rightMotor]=...
        vrep.simxGetObjectHandle...
        (clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    
    % Loop para controle do robô
    while(control ~='p')
        % Função para capturar uma tecla prescionada no teclado
        control = getkey();
        
        % Frente
        if control == 'w'
            [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
                (clientID,Pioneer_p3dx_rightMotor, 1, vrep.simx_opmode_blocking);
            [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
                (clientID,Pioneer_p3dx_leftMotor, 1, vrep.simx_opmode_blocking);
            % Before closing the connection to V-REP, make sure that the last
            % command sent out had time to arrive. You can guarantee this with
            % (for example):
            vrep.simxGetPingTime(clientID);
            
            % Ré
        elseif control == 's'
            [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
                (clientID,Pioneer_p3dx_rightMotor, -1, vrep.simx_opmode_blocking);
            [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
                (clientID,Pioneer_p3dx_leftMotor, -1, vrep.simx_opmode_blocking);
            % Before closing the connection to V-REP, make sure that the last
            % command sent out had time to arrive. You can guarantee this with
            % (for example):
            vrep.simxGetPingTime(clientID);
            
            % Direita
        elseif control == 'd'
            [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
                (clientID,Pioneer_p3dx_leftMotor, 1, vrep.simx_opmode_blocking);
            [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
                (clientID,Pioneer_p3dx_rightMotor, -1, vrep.simx_opmode_blocking);
            % Before closing the connection to V-REP, make sure that the last
            % command sent out had time to arrive. You can guarantee this with
            % (for example):
            vrep.simxGetPingTime(clientID);
            
            % Esquerda
        elseif control == 'a'
            [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
                (clientID,Pioneer_p3dx_rightMotor, 1, vrep.simx_opmode_blocking);
            [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
                (clientID,Pioneer_p3dx_leftMotor, -1, vrep.simx_opmode_blocking);
            % Before closing the connection to V-REP, make sure that the last
            % command sent out had time to arrive. You can guarantee this with
            % (for example):
            vrep.simxGetPingTime(clientID);
            
            % Parar
        elseif control == 'q' || control == 'p'
            [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
                (clientID,Pioneer_p3dx_rightMotor, 0, vrep.simx_opmode_blocking);
            [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
                (clientID,Pioneer_p3dx_leftMotor, 0, vrep.simx_opmode_blocking);
            % Before closing the connection to V-REP, make sure that the last
            % command sent out had time to arrive. You can guarantee this with
            % (for example):
            vrep.simxGetPingTime(clientID);
        end
    end
else
    disp('Failed connecting to remote API server');
end

% Pausa o Timer
stop(t);

% Now send some data to V-REP in a non-blocking fashion:
vrep.simxAddStatusbarMessage...ww
    (clientID,'Goodbye V-REP!',vrep.simx_opmode_oneshot);

% Now close the connection to V-REP:
vrep.simxFinish(clientID);

% call the destructor!
vrep.delete();

disp('Program ended');