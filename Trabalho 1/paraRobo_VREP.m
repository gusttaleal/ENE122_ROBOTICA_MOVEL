%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
% Função criada para previnir colisões frontais com o robô
function paraRobo_VREP(vrep, clientID)

% Handle Pioneer P3DX Motors
[returnCode_Motor_Left, Pioneer_p3dx_leftMotor]=...
    vrep.simxGetObjectHandle...
    (clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);

[returnCode_Motor_Right, Pioneer_p3dx_rightMotor]=...
    vrep.simxGetObjectHandle...
    (clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);

% Handle Pioneer P3DX Lidar
[returnCode_ProximitySensor, Pioneer_p3dx_ultrasonicSensor5]=...
    vrep.simxGetObjectHandle...
    (clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);

% Função para leitura do Lidar
[returnCode_ProximitySensor, detectionState, detectedPoint,~,~]=...
    vrep.simxReadProximitySensor...
    (clientID, Pioneer_p3dx_ultrasonicSensor5, vrep.simx_opmode_streaming);

% Função para leitura do Lidar
[returnCode_ProximitySensor, detectionState, detectedPoint,~,~]=...
    vrep.simxReadProximitySensor...
    (clientID, Pioneer_p3dx_ultrasonicSensor5, vrep.simx_opmode_buffer);

% Se a leitura a frente do robô indicar uma colisão, faça
if ((norm(detectedPoint) <= 0.25) && (norm(detectedPoint) >= 0.1))
    % Para o robô
    [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
        (clientID,Pioneer_p3dx_rightMotor, 0, vrep.simx_opmode_blocking);
    
    [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
        (clientID,Pioneer_p3dx_leftMotor, 0, vrep.simx_opmode_blocking);
    
    
    [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
        (clientID,Pioneer_p3dx_rightMotor, -1, vrep.simx_opmode_blocking);
    
    [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
        (clientID,Pioneer_p3dx_leftMotor, -1, vrep.simx_opmode_blocking);
    
    %pause(0.5);
    
    [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
        (clientID,Pioneer_p3dx_rightMotor, 0, vrep.simx_opmode_blocking);
    
    [returnCode_TargetVel]=vrep.simxSetJointTargetVelocity...
        (clientID,Pioneer_p3dx_leftMotor, 0, vrep.simx_opmode_blocking);
end
end


