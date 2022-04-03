% Make sure to have the server side running in CoppeliaSim: 
% in a child script of a CoppeliaSim scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)

classdef DDRobot
    properties (Access = private)
        sim;
        clientID;

        jointLeftHandle;
        jointRightHandle;

%         r = 0.04;
%         d = 0.16;
        r = 0.045;
        d = 0.16;
    end
    
    methods
        function obj = DDRobot()
            obj.sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            obj.sim.simxFinish(-1); % just in case, close all opened connections
            obj.clientID = obj.sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

            if obj.clientID < 0
                obj.finish();
                error("Cannot connect to client. Be sure Coppelia is listening to port 19999.")
            else
                [res_l, obj.jointLeftHandle] = obj.sim.simxGetObjectHandle( ...
                    obj.clientID, 'joint_left', ...
                    obj.sim.simx_opmode_blocking ...
                );
                [res_r, obj.jointRightHandle] = obj.sim.simxGetObjectHandle( ...
                    obj.clientID, ...
                    'joint_right', ...
                    obj.sim.simx_opmode_blocking ...
                );

                if res_l ~= obj.sim.simx_return_ok || res_r ~= obj.sim.simx_return_ok
                    obj.finish();
                end
            end
        end

        function startSimulation(obj)
            obj.sim.simxStartSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
        end

        function stopSimulation(obj)
            obj.sim.simxStopSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
        end

        function setControlInput(obj, v, w)
            wR = v / obj.r + w * obj.d / (2 * obj.r);
            wL = v / obj.r - w * obj.d / (2 * obj.r);

            obj.setRightJointVelocity(-wR);
            obj.setLeftJointVelocity(-wL);
        end

        function finish(obj)
            fprintf("Simulation finished.")
            obj.sim.simxFinish(obj.clientID);
            obj.sim.delete();
        end
    end

    methods (Access = private)
        function setRightJointVelocity(obj, velocity)
            obj.setJointVelocity(obj.jointRightHandle, velocity);
        end

        function setLeftJointVelocity(obj, velocity)
            obj.setJointVelocity(obj.jointLeftHandle, velocity);
        end

        function setJointVelocity(obj, joint, velocity)
            obj.sim.simxSetJointTargetVelocity( ...
                obj.clientID, ...
                joint, ...
                velocity, ...
                obj.sim.simx_opmode_oneshot ...
            );
        end
    end
end