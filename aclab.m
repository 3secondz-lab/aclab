classdef aclab < handle
    
    properties
        ai
        data
        u
        joy
        img
        prev
        filepath
    end
    
    methods
        function obj = aclab()
            if verLessThan('matlab','9.9')
                obj.u = udp('127.0.0.1',4420);
                fopen(obj.u);
            else
                obj.u = udpport('LocalHost','127.0.0.1');
            end
                
            obj.joy = mvJoy(1);
        end
        
        function obj = getMap(obj)
            obj.send('init');
            obj.filepath = obj.recv();
            try
                if isfile(obj.filepath+"_bak")
                    aiFile = fopen(obj.filepath+"_bak");
                else
                    aiFile = fopen(obj.filepath,'rb');
                end
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
                obj.ai.dX = zeros(dlen,1);
                obj.ai.dY = zeros(dlen,1);
                obj.ai.dZ = zeros(dlen,1);
                
                for i = 1:dlen
                    readrow = fread(aiFile, 4, 'float32');
                    obj.ai.pathX(i) = readrow(1);
                    obj.ai.pathY(i) = readrow(3);
                    obj.ai.pathZ(i) = readrow(2);
                    obj.ai.travel(i) = readrow(4);
                    readrow = fread(aiFile,1, 'int32');
                    obj.ai.nPoints(i) = readrow(1);
                end
                for i = 1:dlen
                    readrow = fread(aiFile, 18, 'float32');
                    obj.ai.speed(i) = readrow(2);
                    obj.ai.accel(i) = readrow(3);
                    obj.ai.yawRate(i) = readrow(5);
                    obj.ai.radius(i) = readrow(6);
                    obj.ai.offLeft(i) = readrow(7);
                    obj.ai.offRight(i) = readrow(8);
                    obj.ai.dX(i) = readrow(17);
                    obj.ai.dY(i) = readrow(15);
                    obj.ai.dZ(i) = readrow(16);
                    obj.ai.direction(i) = atan2(readrow(17),readrow(15));
                    obj.ai.pathLeftX(i) = obj.ai.pathX(i) + sin(obj.ai.direction(i))*readrow(7);
                    obj.ai.pathLeftY(i) = obj.ai.pathY(i) - cos(obj.ai.direction(i))*readrow(7);
                    obj.ai.pathRightX(i) = obj.ai.pathX(i) - sin(obj.ai.direction(i))*readrow(8);
                    obj.ai.pathRightY(i) = obj.ai.pathY(i) + cos(obj.ai.direction(i))*readrow(8);     
                end
                obj.ai.totalLength = obj.ai.travel(end) + sqrt((obj.ai.pathX(end) - obj.ai.pathX(1)).^2+(obj.ai.pathY(end) - obj.ai.pathY(1)).^2);
                obj.ai.nPidx = obj.ai.travel/obj.ai.totalLength;
                disp(['acServer:ParsingAiFile:',' Finished']);
                
            catch
                disp(['acServer:NoMatchingFile:fast_lane.ai path:',char(path)]);
            end
            fclose(aiFile);
        end
        
        function obj = setMap(obj, pathX, pathY, speed)
            
            
            [cl, lo, ro, dif, sp, rd] = obj.calcpath([pathX,pathY], speed);
            
            fclose('all');
            if ~isfile(obj.filepath+"_bak")
                movefile(obj.filepath, obj.filepath+"_bak", 'f');
            end
            
            bakFile = fopen(obj.filepath+"_bak",'rb');
            aiFile = fopen(obj.filepath,'wb');
            
            header = fread(bakFile, 4, 'int32');            
            dlen = header(2);            
            
            fwrite(aiFile, header, 'int32');
            for i = 1:dlen
                readrow = fread(bakFile, 4, 'float32');
                readrow(1) = cl(i,1);
                readrow(3) = cl(i,2);
                fwrite(aiFile, readrow, 'float32');
                readrow = fread(bakFile,1, 'int32');
                fwrite(aiFile, readrow, 'int32');
            end
            for i = 1:dlen
                readrow = fread(bakFile, 18, 'float32');
                readrow(2) = sp(i);
                readrow(5) = sp(i)/rd(i);
                readrow(6) = rd(i);
                readrow(7) = lo(i);
                readrow(8) = ro(i);
                readrow(17) = dif(i,1);
                readrow(15) = dif(i,2);
                fwrite(aiFile,readrow,'float32');
            end
            while 1
                readrow=fread(bakFile, 1, 'int8');
                if ~isempty(readrow)
                    fwrite(aiFile, readrow, 'int8');
                else
                    break;
                end
            end
            fclose(aiFile);
            fclose(bakFile);
            
        end
        
        function obj = poll(obj)
            obj.send('query')            
            obj.data = jsondecode(obj.recv());
        end
        
        function obj = impoll(obj)
            obj.send('imquery')
            obj.img = imread(obj.recv());
        end
        function obj = push(obj,cmd_gas,cmd_brk,cmd_str)
            obj.joy.data.wAxisX = int32((cmd_str+1)/2*(32768));
            obj.joy.data.wAxisY = int32(cmd_gas*32768);
            obj.joy.data.wAxisZ = int32(cmd_brk*32768);
            
            resp = obj.joy.update();
            if resp == 0
                disp(['acServer:mvJoy:CannotSetJoystick']);
            end
        end
        
        function obj = preview(obj,nstep)
            
            effstep = 30;
            
            [~,index] = min(abs(obj.ai.travel-obj.data.normalizedCarPosition*obj.ai.totalLength));
            offset = obj.ai.travel(index)-obj.data.normalizedCarPosition*obj.ai.totalLength;
            if(index+nstep > obj.ai.nPoints(end)+1)
                window = [index:obj.ai.nPoints(end)+1, 1:nstep-(obj.ai.nPoints(end)-index+2)];
            else
                window = [index:index+nstep];
            end
            
            originX = obj.data.carCoordinates(1);
            originY = obj.data.carCoordinates(3);
            pathX = obj.ai.pathX(window)- originX;
            pathY = obj.ai.pathY(window)- originY;
            pathLeftX = obj.ai.pathLeftX(window)- originX;
            pathLeftY = obj.ai.pathLeftY(window)- originY;
            pathRightX = obj.ai.pathRightX(window)- originX;
            pathRightY = obj.ai.pathRightY(window)- originY;
            psi = obj.data.heading;
            obj.prev.travel = obj.ai.travel(window)-obj.ai.travel(index)-offset;
            obj.prev.pathY = cos(psi)*pathX + sin(psi)*pathY;
            obj.prev.pathX = -sin(psi)*pathX + cos(psi)*pathY;
            obj.prev.pathLeftY = cos(psi)*pathLeftX + sin(psi)*pathLeftY;
            obj.prev.pathLeftX = -sin(psi)*pathLeftX + cos(psi)*pathLeftY;
            obj.prev.pathRightY = cos(psi)*pathRightX + sin(psi)*pathRightY;
            obj.prev.pathRightX = -sin(psi)*pathRightX + cos(psi)*pathRightY;
            obj.prev.speed = obj.ai.speed(window);
            obj.prev.heading = unwrap(obj.ai.direction(window)-pi/2-psi);
            obj.prev.path = polyfit(obj.prev.pathX(1:min(nstep,effstep)), obj.prev.pathY(1:min(nstep,effstep)), 3);
            obj.prev.pathLeft = polyfit(obj.prev.pathLeftX(1:min(nstep,effstep)), obj.prev.pathLeftY(1:min(nstep,effstep)), 3);
            obj.prev.pathRight = polyfit(obj.prev.pathRightX(1:min(nstep,effstep)), obj.prev.pathRightY(1:min(nstep,effstep)), 3);
        end
        
        function plotpreview(obj)
            plot(-obj.prev.pathY, obj.prev.pathX,'.',-obj.prev.pathLeftY, obj.prev.pathLeftX,'.',-obj.prev.pathRightY, obj.prev.pathRightX,'.');
        end
        
        function setcontroller(obj,axis)
            switch axis
                case 1
                    for i = 0:0.01:1
                        obj.push(i,0,0);
                        pause(0.05);
                    end
                case 2
                    for i = 0:0.01:1
                        obj.push(0,i,0);
                        pause(0.05);
                    end
                case 3
                    for i = 0:0.01:1
                        obj.push(0,0,i);
                        pause(0.05);
                    end
            end
        end
        
        function send(obj, msg)
            if verLessThan('matlab','9.9')
                fwrite(obj.u,msg)
            else
                writeline(obj.u, msg,'127.0.0.1',4420);
            end
        end
        
        function resp = recv(obj)
            if verLessThan('matlab','9.9')
                resp = fscanf(obj.u);
            else
                resp = readline(obj.u);
            end
        end
        function delete(obj)
            delete(obj.u)
            delete(obj.joy)
        end
        
        function [cl, lo, ro, dif, sp, rd] = calcpath(obj, pp, speed)
            spline = [obj.ai.pathX, obj.ai.pathY];
            d = zeros(max(size(spline)),1);
            dsp = [diff(spline); spline(1,:)-spline(end,:)];
            for i = 1:max(size(pp))
                pt = pp(i,:);
                dpt = vecnorm(pt' - (spline+dsp/2)');
                [~,sind] = sort(dpt);
                idx = [min(sind(1:2)), max(sind(1:2))];
                if idx(2)<idx(1)
                    if ~idx(1) == max(size(spline))
                        idx = flip(idx);
                    else
                        idx = [max(size(spline)), 1];
                    end
                elseif idx(2) == max(size(spline))
                    idx = flip(idx);
                end
                d1 = norm(pt - spline(idx(1),:));
                d2 = norm(pt - spline(idx(2),:));
                
                if idx(2)-idx(1) > 0
                    s = norm(obj.ai.travel(idx(2))-obj.ai.travel(idx(1)));
                else
                    s = norm(spline(idx(1),:)-spline(idx(2),:));
                end
                
                s1 = d1*s/(d1+d2);
                s2 = d2*s/(d1+d2);

                d(i) = obj.ai.travel(idx(1))+s1;
            end
            d(1) = 0;
            clx = interp1(d(2:end-1), pp((2:end-1),1), obj.ai.travel, 'spline');
            cly = interp1(d(2:end-1), pp((2:end-1),2), obj.ai.travel, 'spline');
            sp = interp1(d, speed, obj.ai.travel, 'spline');
%             dcl = [obj.ai.dX, obj.ai.dY, 0*obj.ai.dX];
%             ocl = [clx-obj.ai.pathX, cly-obj.ai.pathY, 0*cly];
%             ocl(end) = ocl(end-1);
%             ocl(1) = ocl(2);
            lo = vecnorm([clx-obj.ai.pathLeftX, cly-obj.ai.pathLeftY],2,2);
            ro = vecnorm([clx-obj.ai.pathRightX, cly-obj.ai.pathRightY],2,2);
            lo(1) = lo(2);
            lo(end) = lo(end-1);
            ro(1) = ro(1);
            ro(end) = ro(end-1);
            dif = [gradient(clx), gradient(cly), obj.ai.dZ];
            cl = [clx, cly, obj.ai.pathZ];
            rd = obj.calcradius(clx, cly);
            rd(1) = rd(2);
        end
        
        function radius  = calcradius(obj, pathX, pathY) 

            x = pathX;
            y = pathY;

            dx = [0;diff(x)];
            dy = [0;diff(y)];

            s = [0; sqrt(dx.^2 + dy.^2)];

            k = zeros(length(x),1);
            for i= 2:(length(x)-1)
                if (dx(i)==0 || dy(i)==0 || dx(i+1)==0 || dy(i+1)==0)
                    k(i) = k(i-1);
                    continue;
                end
                k(i)=((x(i)-x(i-1))/sqrt((x(i)-x(i-1))^2+(y(i)-y(i-1))^2) +...
                (x(i+1)-x(i))/sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2)) *...
                ((y(i+1)-y(i))/sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2) -...
                (y(i)-y(i-1))/sqrt((x(i)-x(i-1))^2+(y(i)-y(i-1))^2))/...
                (sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2)+sqrt((x(i)-x(i-1))^2+...
                (y(i)-y(i-1))^2)) - ((y(i)-y(i-1))/sqrt((x(i)-x(i-1))^2+...
                (y(i)-y(i-1))^2) + (y(i+1)-y(i))/sqrt((x(i+1)-x(i))^2+...
                (y(i+1)-y(i))^2)) * ((x(i+1)-x(i))/sqrt((x(i+1)-x(i))^2+...
                (y(i+1)-y(i))^2) - (x(i)-x(i-1))/sqrt((x(i)-x(i-1))^2+...
                (y(i)-y(i-1))^2))/(sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2)+...
                sqrt((x(i)-x(i-1))^2+(y(i)-y(i-1))^2));
            end

            k(1) = k(2);
            k(length(k)) = k(length(k)-1);

            k(k>0.5) = 0.5;
            k(k<-0.5) = -0.5;

            radius = 1./medfilt1(movmean(k,10),10);
        end
    end
end

