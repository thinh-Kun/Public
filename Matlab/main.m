clc;
close all;
clear ;
disp('Program started');
%% Khoi tao API
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
%% Bat dau chuong trinh
if (clientID>-1)
    disp('Connected to remote API server');
    while 1
        dlgtitle = 'UR10 kinematic';
        prompt = {'Enter 1 for Forward Enter 2 for Inverse Enter 0 for Cancel'};
        command = inputdlg(prompt,dlgtitle);
        command = str2double(command(1));
        if command == 1
            for i = 1 : 6
                [r,joint(i)]=sim.simxGetObjectHandle(clientID, strcat('UR10_joint', int2str(i)) , sim.simx_opmode_blocking);
            end
            promptForward = {'Enter q1','Enter q2', 'Enter q3', 'Enter q4', 'Enter q5','Enter q6'};
            dlgtitleForward = 'Forward Kinematic for UR10';
            inputAngle = inputdlg(promptForward,dlgtitleForward);
            inputAngle = str2double(inputAngle);
            q = [inputAngle(1), inputAngle(2), inputAngle(3), inputAngle(4), inputAngle(5), inputAngle(6)];
            for j = 1 : 6
                sim.simxSetJointTargetPosition(clientID, joint(j), q(j), sim.simx_opmode_blocking);
            end
            disp("Position of end effector");
            [x, y, z] = UR10_forwardKinematic(q(1), q(2), q(3), q(4), q(5), q(6));
            disp("xE = " + num2str(x));
            disp("yE = " + num2str(y));
            disp("zE = " + num2str(z));
        elseif command == 2
            %Diem goc cho dong hoc nguoc
            xx_0 = -0.852;
            yy_0 = -0.2437;
            zz_0 = 0.03336;
            X_0 = [xx_0; yy_0; zz_0]; % Vector vi tri E
            % Gia tri gan dung cua cac goc khop ban dau
            q1_0 = -5.6065e-05;
            q2_0 = 0.7854;
            q3_0 = 1.5708;
            q4_0 = 1.0472;
            q5_0 = 0.5236;
            q6_0 = 3.1416;
            dlgtitleInverse = 'Inverse Kinematic for UR10';
            promptInverse = {'Enter 1 for circle trajectory Enter 2 for rectangle trajactory'};
            trajectoryCommand = inputdlg(promptInverse, dlgtitleInverse);
            trajectoryCommand = str2double(trajectoryCommand(1));
            if trajectoryCommand == 1
                circleDlgtitle = 'Circle trajectory';
                circlePrompt = {'Enter radius', 'Enter height'};
                circleCommand = inputdlg(circlePrompt, circleDlgtitle);
                radius = str2double(circleCommand(1));
                height = str2double(circleCommand(2));
                for i = 1 : 6
                    [r,joint(i)]=sim.simxGetObjectHandle(clientID, strcat('UR10_joint', int2str(i)) , sim.simx_opmode_blocking);
                end
                for n = 1: 1: 10^5
                    Jnd_0 = computeJnd(q1_0, q2_0, q3_0, q4_0, q5_0, q6_0);
                    [xE_0, yE_0, zE_0] = UR10_forwardKinematic(q1_0, q2_0, q3_0, q4_0, q5_0, q6_0);% tinh lai xx_0, yy_0 theo q_0
                    XX_0 = [xE_0; yE_0; zE_0];
                    delta_q_0 = Jnd_0*(X_0 - XX_0);% Tinh gia tri hieu chinh delta_q_0
                    % Tinh lai cac gia tri q_0 hieu chinh
                    q1_0 = q1_0 + delta_q_0(1, 1);
                    q2_0 = q2_0 + delta_q_0(2, 1);
                    q3_0 = q3_0 + delta_q_0(3, 1);
                    q4_0 = q4_0 + delta_q_0(4, 1);
                    q5_0 = q5_0 + delta_q_0(5, 1);
                    q5_0 = q5_0 + delta_q_0(6, 1);
                    % Khai bao do chinh xac can thiet va ta vong lap tinh toan
                    ss = 10^(-10);
                    if abs(delta_q_0(1, 1)) < ss
                        if abs(delta_q_0(2, 1)) < ss
                            if abs(delta_q_0(3, 1)) < ss
                                if abs(delta_q_0(4, 1)) < ss
                                    if abs(delta_q_0(5, 1)) < ss
                                        if abs(delta_q_0(6, 1)) < ss
                                            break
                                        end
                                    end
                                end
                            end
                        end
                    end
                    n;
                end
                % Xac nhan cac gia tri q_0 chinh xac sau khi hieu chinh
                q1 = q1_0;
                q2 = q2_0;
                q3 = q3_0;
                q4 = q4_0;
                q5 = q5_0;
                q6 = q6_0;
                % Bien thoi gian
                dt = 0.05; % Khai bao buoc thoi gian chay
                t_max = 8; % Khai bao thoi gian chay
                %Tinh dong hoc nguoc
                for t = 0:dt:t_max
                    [Xd, dXd] = circleTracjectory(t, radius, height); % Vi tri va van toc diem E cho truoc theo thoi gian t
                    Jnd = computeJnd(q1, q2, q3, q4, q5, q6);
                    dX = [dXd(1); dXd(2); dXd(3)];
                    q = [q1; q2; q3; q4; q5; q6];
                    dq = Jnd*dX; % Van toc goc khop
                    for k = 1:1:10000
                        q_k = q + dq*dt;
                        q1 = q_k(1, 1);
                        q2 = q_k(2, 1);
                        q3 = q_k(3, 1);
                        q4 = q_k(4, 1);
                        q5 = q_k(5, 1);
                        q6 = q_k(6, 1);
                        Jnd_real = computeJnd(q1, q2, q3, q4, q5, q6); % Tinh lai gia tri ma tran Jacobian
                        [xE, yE, zE] = UR10_forwardKinematic(q1, q2, q3, q4, q5, q6);%Tinh lai quy dao diem E tu q vua tim duoc
                        Xq = [xE; yE; zE];
                        [Xd, dXd] = circleTracjectory(t, radius, height);% Goi lai quy dao mong muon
                        Xm = [Xd(1);Xd(2); Xd(3)];
                        Delta_q = Jnd_real*(Xm - Xq); % Tinh sai lech goc khop
                        % khai bao do chinh xac can thiet
                        ss1 = 10^(-5);
                        if abs(Delta_q(1,1)) < ss1
                            if abs(Delta_q(2,1)) < ss1
                                if abs(Delta_q(3,1)) < ss1
                                    if abs(Delta_q(4,1)) < ss1
                                        if abs(Delta_q(5,1)) < ss1
                                            if abs(Delta_q(6,1)) < ss1
                                                break
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                    k;
                    % Tinh lai gia tri goc khop chinh xac
                    q1 = q1 + Delta_q(1,1);
                    q2 = q2 + Delta_q(2,1);
                    q3 = q3 + Delta_q(3,1);
                    q4 = q4 + Delta_q(4,1);
                    q5 = q5 + Delta_q(5,1);
                    q6 = q6 + Delta_q(6,1);
                    q = [q1; q2; q3; q4; q5; q6];
                    % So sanh ket qua
                    % Tinh lai quy dao lan nua
                    [xE_tinhlai, yE_tinhlai,zE_tinhlai] = UR10_forwardKinematic(q1, q2, q3, q4, q5, q6);
                    for j = 1 : 6
                        sim.simxSetJointTargetPosition(clientID, joint(j), q(j), sim.simx_opmode_blocking);
                    end
                    % Thiet lap vector sai so quy dao
                    eX = Xd(1) - xE_tinhlai;
                    eY = Xd(2) - yE_tinhlai;
                    eZ = Xd(3) - zE_tinhlai;
                    % Ve do thi
                    %Do thi cac bien khop - ket qua bai toan dong hoc nguoc
                    figure(1)
                    plot(t, q1, 'r.',t, q2, 'g.', t, q3, 'b.', t, q4, 'y.',t, q5, 'm.', t, q6, 'c.' );
                    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
                    xlabel('time(sec)');
                    ylabel('Bien khop q1, q2, q3 va q4');
                    hold on
                    grid on
                    % Do thi quy dao thao tac
                    figure(2)
                    plot(t, xE_tinhlai, 'r.',t, yE_tinhlai, 'g.', t, zE_tinhlai, 'b.');
                    legend('x', 'y', 'z');
                    xlabel('time(sec)');
                    ylabel('Do thi quy dao thao tac tinh lai');
                    hold on
                    grid on
                    % Do thi sai so quy dao thao tac
                    figure(3)
                    plot(t, eX, 'r.',t, eY, 'g.', t, eZ, 'b.');
                    legend('errorX', 'errorY', 'errorZ');
                    xlabel('time(sec)');
                    ylabel('Sai so quy dao thao tac');
                    hold on
                    grid on
                    
                end
            elseif trajectoryCommand == 2
                squareDlgtitle = 'Square tracjectory';
                squarePrompt = {'Enter length of edge', 'Enter height'};
                squareCommand = inputdlg(squarePrompt, squareDlgtitle);
                lengthOfEdge = str2double(squareCommand(1));
                height = str2double(squareCommand(2));
                for i = 1 : 6
                    [r,joint(i)]=sim.simxGetObjectHandle(clientID, strcat('UR10_joint', int2str(i)) , sim.simx_opmode_blocking);
                end
                for n = 1: 1: 10^5
                    Jnd_0 = computeJnd(q1_0, q2_0, q3_0, q4_0, q5_0, q6_0);
                    [xE_0, yE_0, zE_0] = UR10_forwardKinematic(q1_0, q2_0, q3_0, q4_0, q5_0, q6_0);% tinh lai xx_0, yy_0 theo q_0
                    XX_0 = [xE_0; yE_0; zE_0];
                    delta_q_0 = Jnd_0*(X_0 - XX_0);% Tinh gia tri hieu chinh delta_q_0
                    % Tinh lai cac gia tri q_0 hieu chinh
                    q1_0 = q1_0 + delta_q_0(1, 1);
                    q2_0 = q2_0 + delta_q_0(2, 1);
                    q3_0 = q3_0 + delta_q_0(3, 1);
                    q4_0 = q4_0 + delta_q_0(4, 1);
                    q5_0 = q5_0 + delta_q_0(5, 1);
                    q5_0 = q5_0 + delta_q_0(6, 1);
                    % Khai bao do chinh xac can thiet va ta vong lap tinh toan
                    ss = 10^(-10);
                    if abs(delta_q_0(1, 1)) < ss
                        if abs(delta_q_0(2, 1)) < ss
                            if abs(delta_q_0(3, 1)) < ss
                                if abs(delta_q_0(4, 1)) < ss
                                    if abs(delta_q_0(5, 1)) < ss
                                        if abs(delta_q_0(6, 1)) < ss
                                            break
                                        end
                                    end
                                end
                            end
                        end
                    end
                    n;
                end
                % Xac nhan cac gia tri q_0 chinh xac sau khi hieu chinh
                q1 = q1_0;
                q2 = q2_0;
                q3 = q3_0;
                q4 = q4_0;
                q5 = q5_0;
                q6 = q6_0;
                % Bien thoi gian
                dt = 0.05; % Khai bao buoc thoi gian chay
                t_max = 8; % Khai bao thoi gian chay
                %Tinh dong hoc nguoc
                for t = 0:dt:t_max
                    [Xd, dXd] = squareTracjectory(t, lengthOfEdge, height); % Vi tri va van toc diem E cho truoc theo thoi gian t
                    Jnd = computeJnd(q1, q2, q3, q4, q5, q6);
                    dX = [dXd(1); dXd(2); dXd(3)];
                    q = [q1; q2; q3; q4; q5; q6];
                    dq = Jnd*dX; % Van toc goc khop
                    for k = 1:1:10000
                        q_k = q + dq*dt;
                        q1 = q_k(1, 1);
                        q2 = q_k(2, 1);
                        q3 = q_k(3, 1);
                        q4 = q_k(4, 1);
                        q5 = q_k(5, 1);
                        q6 = q_k(6, 1);
                        Jnd_real = computeJnd(q1, q2, q3, q4, q5, q6); % Tinh lai gia tri ma tran Jacobian
                        [xE, yE, zE] = UR10_forwardKinematic(q1, q2, q3, q4, q5, q6);%Tinh lai quy dao diem E tu q vua tim duoc
                        Xq = [xE; yE; zE];
                        [Xd, dXd] = squareTracjectory(t, lengthOfEdge, height);% Goi lai quy dao mong muon
                        Xm = [Xd(1);Xd(2); Xd(3)];
                        Delta_q = Jnd_real*(Xm - Xq); % Tinh sai lech goc khop
                        % khai bao do chinh xac can thiet
                        ss1 = 10^(-5);
                        if abs(Delta_q(1,1)) < ss1
                            if abs(Delta_q(2,1)) < ss1
                                if abs(Delta_q(3,1)) < ss1
                                    if abs(Delta_q(4,1)) < ss1
                                        if abs(Delta_q(5,1)) < ss1
                                            if abs(Delta_q(6,1)) < ss1
                                                break
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                    k;
                    % Tinh lai gia tri goc khop chinh xac
                    q1 = q1 + Delta_q(1,1);
                    q2 = q2 + Delta_q(2,1);
                    q3 = q3 + Delta_q(3,1);
                    q4 = q4 + Delta_q(4,1);
                    q5 = q5 + Delta_q(5,1);
                    q6 = q6 + Delta_q(6,1);
                    q = [q1; q2; q3; q4; q5; q6];
                    % So sanh ket qua
                    % Tinh lai quy dao lan nua
                    [xE_tinhlai, yE_tinhlai,zE_tinhlai] = UR10_forwardKinematic(q1, q2, q3, q4, q5, q6);
                    for j = 1 : 6
                        sim.simxSetJointTargetPosition(clientID, joint(j), q(j), sim.simx_opmode_blocking);
                    end
                    % Thiet lap vector sai so quy dao
                    eX = Xd(1) - xE_tinhlai;
                    eY = Xd(2) - yE_tinhlai;
                    eZ = Xd(3) - zE_tinhlai;
                    % Ve do thi
                    %Do thi cac bien khop - ket qua bai toan dong hoc nguoc
                    figure(1)
                    plot(t, q1, 'r.',t, q2, 'g.', t, q3, 'b.', t, q4, 'y.',t, q5, 'm.', t, q6, 'c.' );
                    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
                    xlabel('time(sec)');
                    ylabel('Bien khop q1, q2, q3 va q4');
                    hold on
                    grid on
                    % Do thi quy dao thao tac
                    figure(2)
                    plot(t, xE_tinhlai, 'r.',t, yE_tinhlai, 'g.', t, zE_tinhlai, 'b.');
                    legend('x', 'y', 'z');
                    xlabel('time(sec)');
                    ylabel('Do thi quy dao thao tac tinh lai');
                    hold on
                    grid on
                    % Do thi sai so quy dao thao tac
                    figure(3)
                    plot(t, eX, 'r.',t, eY, 'g.', t, eZ, 'b.');
                    legend('errorX', 'errorY', 'errorZ');
                    xlabel('time(sec)');
                    ylabel('Sai so quy dao thao tac');
                    hold on
                    grid on
                    
                end
            end
            
        else
            break
        end
    end
else
    disp('Failed connecting to remote API server');
end
sim.delete();% call the destructor!
disp('Program ended');
