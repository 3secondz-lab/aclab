classdef ac_server < handle
    %AC_SERVER �� Ŭ������ ��� ���� ��ġ
    %   �ڼ��� ���� ��ġ
    
    properties
        ai
        data
        u
        joy
    end
    
    methods
        function obj = ac_server()
            obj.u = udpport('LocalHost','127.0.0.1');
            obj.joy = mvJoy(1);
        end
        
        function obj = getMap(obj)
            writeline(obj.u, 'init','127.0.0.1',4420);
            path = readline(obj.u);
            try
                aiFile = fopen(path,'rb');
                header = fread(aiFile, 4, 'int32');
                disp(['acServer:ParsingAiFile:',num2str(header(2)),' Lines']);
                dlen = header(2);
                obj.ai.pathX = zeros(dlen,1);
                obj.ai.pathY = zeros(dlen,1);
                obj.ai.pathZ = zeros(dlen,1);
                obj.ai.travel = zeros(dlen,1);
                obj.ai.speed = zeros(dlen,1);
                obj.ai.accel = zeros(dlen,1);
                obj.ai.yawRate = zeros(dlen,1);
                obj.ai.radius = zeros(dlen,1);
                obj.ai.offLeft = zeros(dlen,1);
                obj.ai.offRight = zeros(dlen,1);
                obj.ai.nPidx = zeros(dlen,1);
                obj.ai.direction = zeros(dlen,1);
                obj.ai.pathLeftX = zeros(dlen,1);
                obj.ai.pathLeftY = zeros(dlen,1);
                obj.ai.pathRightX = zeros(dlen,1);
                obj.ai.pathRightY = zeros(dlen,1);
                
                for i = 1:dlen
                    readrow = fread(aiFile, 4, 'float32');
                    obj.ai.pathX(i) = readrow(1);
                    obj.ai.pathY(i) = readrow(2);
                    obj.ai.pathZ(i) = readrow(3);
                    obj.ai.travel(i) = readrow(4);
                    readrow = fread(aiFile,1, 'int32');
                    obj.ai.nPidx(i) = readrow(1);
                end
                for i = 1:dlen
                    readrow = fread(aiFile, 18, 'float32');
                    obj.ai.speed(i) = readrow(2);
                    obj.ai.accel(i) = readrow(3);
                    obj.ai.yawRate(i) = readrow(5);
                    obj.ai.radius(i) = readrow(6);
                    obj.ai.offLeft(i) = readrow(7);
                    obj.ai.offRight(i) = readrow(8);
                    obj.ai.direction(i) = atan2(readrow(17),readrow(15));
                    obj.ai.pathLeftX(i) = obj.ai.pathX(i) + sin(obj.ai.direction(i))*readrow(7);
                    obj.ai.pathLeftY(i) = obj.ai.pathY(i) - cos(obj.ai.direction(i))*readrow(7);
                    obj.ai.pathRightX(i) = obj.ai.pathX(i) - sin(obj.ai.direction(i))*readrow(7);
                    obj.ai.pathRightY(i) = obj.ai.pathY(i) + cos(obj.ai.direction(i))*readrow(7);     
                end
                disp(['acServer:ParsingAiFile:',' Finished']);
                
            catch
                disp(['acServer:NoMatchingFile:fast_lane.ai path:',char(path)]);
            end
        end
        
        function obj = poll(obj)
            writeline(obj.u, 'query','127.0.0.1',4420);
            obj.data = jsondecode(readline(obj.u));
        end
        
        function obj = push(obj,cmd_str,cmd_gas,cmd_brk)
            obj.joy.data.wAxisX = int32((cmd_str+1)/2*(32768));
            obj.joy.data.wAxisY = int32(cmd_gas*32768);
            obj.joy.data.wAxisZ = int32(cmd_brk*32768);
            
            resp = obj.joy.update();
            if resp == 0
                disp(['acServer:mvJoy:CannotSetJoystick']);
            end
        end
        
        function delete(obj)
            delete(obj.u)
            delete(obj.joy)
        end
    end
end

