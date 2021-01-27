classdef acdriver
    %ACDRIVER 이 클래스의 요약 설명 위치
    %   자세한 설명 위치
    
    properties
        conf
        state
        x0
    end
    
    methods
        function obj = acdriver(paramfile)
            %ACDRIVER 이 클래스의 인스턴스 생성
            %   자세한 설명 위치
            obj.conf = jsondecode(fileread(paramfile));
            obj.state = casadi.SX.sym('x', obj.conf.nstep*6 + (obj.conf.nstep-1)*2);
            obj.conf.xi = 1;
            obj.conf.yi = obj.conf.xi + obj.conf.nstep;
            obj.conf.psii = obj.conf.yi + obj.conf.nstep;
            obj.conf.veli = obj.conf.psii + obj.conf.nstep;
            obj.conf.ctei = obj.conf.veli + obj.conf.nstep;
            obj.conf.epsii = obj.conf.ctei + obj.conf.nstep;
            obj.conf.deltai = obj.conf.epsii + obj.conf.nstep;
            obj.conf.acceli = obj.conf.deltai + obj.conf.nstep-1;
        end
        
        function f = calF(obj,preview)
            %METHOD1 이 메서드의 요약 설명 위치
            %   자세한 설명 위치
            f = casadi.SX(0);
            coeffvel = polyfit(preview.pathX, preview.speed, 3);
            vel = casadi.SX(zeros(obj.conf.nstep,1));
%             vel = interp1(preview.pathX, preview.speed, obj.state(obj.conf.xi:obj.conf.xi+obj.conf.nstep));
            for i = 0:(obj.conf.nstep-1)
                tmp = obj.state(obj.conf.xi+i);
                vel(obj.conf.xi+i) = coeffvel(1)*tmp^3 + coeffvel(2)*tmp^2 + coeffvel(3)*tmp + coeffvel(4);
                if i <1
                    continue
                end
                f = f + obj.conf.coeff.cte * obj.state(obj.conf.ctei + i)^2;
                f = f + obj.conf.coeff.epsi * obj.state(obj.conf.epsii + i)^2;
%                 f = f + obj.conf.coeff.cte_epsi * obj.state(obj.conf.ctei + i)^2 * obj.state(obj.conf.epsii + i)^2;
                f = f + obj.conf.coeff.vel * (obj.state(obj.conf.veli + i) - vel(i+1)).^2;
            end
            for i = 0:(obj.conf.nstep-2)
                f = f + obj.conf.coeff.delta * obj.state(obj.conf.deltai + i)^2;
                f = f + obj.conf.coeff.accel * (obj.state(obj.conf.acceli + i)*obj.state(obj.conf.deltai + i))^2/vel(i+1);
            end
            for i = 0:(obj.conf.nstep-3)
                f = f + obj.conf.coeff.ddelta * (obj.state(obj.conf.deltai + i + 1)-obj.state(obj.conf.deltai + i))^2;
            end
        end
        
        function g = calG(obj,preview)
            x0 = obj.state(obj.conf.xi);
            y0 = obj.state(obj.conf.yi);
            psi0 = obj.state(obj.conf.psii);
            vel0 = obj.state(obj.conf.veli);
            cte0 = obj.state(obj.conf.ctei);
            epsi0 = obj.state(obj.conf.epsii);
            
            f0 = polyval(preview.path, x0);
            psides0 = atan(preview.path(3) + 2*preview.path(2)*x0 + 3*preview.path(1)*x0^2);
            
            g = [x0;y0;psi0;vel0;cte0;epsi0];
            
            for i = 1:(obj.conf.nstep-1)
                x0 = obj.state(obj.conf.xi-1+i);
                y0 = obj.state(obj.conf.yi-1+i);
                psi0 = obj.state(obj.conf.psii-1+i);
                vel0 = obj.state(obj.conf.veli-1+i);
                cte0 = obj.state(obj.conf.ctei-1+i);
                epsi0 = obj.state(obj.conf.epsii-1+i);
                delta0 = obj.state(obj.conf.deltai-1+i);
                accel0 = obj.state(obj.conf.acceli-1+i);
                
                
                x1 = obj.state(obj.conf.xi+i);
                y1 = obj.state(obj.conf.yi+i);
                psi1 = obj.state(obj.conf.psii+i);
                vel1 = obj.state(obj.conf.veli+i);
                cte1 = obj.state(obj.conf.ctei+i);
                epsi1 = obj.state(obj.conf.epsii+i);
                
                f0 = polyval(preview.path, x1);
                psides0 = atan(preview.path(3) + 2*preview.path(2)*x1 + 3*preview.path(1)*x1^2);
                
                g = [g;
                     x1 - (x0 + vel0 * cos(psi0) * obj.conf.dt);
                     y1 - (y0 + vel0 * sin(psi0) * obj.conf.dt);
                     psi1 - (psi0 + vel0 * (obj.conf.car.steerratio*delta0/obj.conf.car.lf) * obj.conf.dt);
                     vel1 - (vel0 + accel0 * obj.conf.dt);
                     cte1 - (-(f0-y0) + vel0 * sin(epsi0) * obj.conf.dt);
                     epsi1 - ((psi1 - psides0) + vel0 * (obj.conf.car.steerratio*delta0/obj.conf.car.lf) * obj.conf.dt);];
            end
        end
        
        function test = testmodel(obj, data, preview)
            x = zeros(10,1);
            y = zeros(10,1);
            psi = zeros(10,1);
            vel = zeros(10,1);
            epsi = zeros(10,1);
            cte = zeros(10,1);
            psides = zeros(10,1);
            x(1) = 0;
            y(1) = 0;
            psi(1) = 0;
            vel(1) = data.localVelocity(3);
            epsi(1) = preview.path(3);
            cte(1) = preview.path(4);
            psides(1) = atan(preview.path(3) + 2*preview.path(2)*x(1) + 3*preview.path(1)*x(1)^2);
            for i = 2:10
                x(i) = x(i-1) + cos(psi(i-1))*vel(i-1)*0.1;
                y(i) = y(i-1) + sin(psi(i-1))*vel(i-1)*0.1;
                psi(i) = psi(i-1) - vel(i-1)*0.52*data.steerAngle/2.7 * 0.1;
                vel(i) = vel(i-1) + data.accG(3)*9.8*0.1;
                cte(i) = -(polyval(preview.path, x(i-1)) - y(i-1)) + vel(i-1)*sin(epsi(i-1))*0.1;
                epsi(i) = psi(i-1) - atan(preview.path(3) + 2*preview.path(2)*x(i-1) + 3*preview.path(1)*x(i-1)^2) - vel(i-1)*0.52*data.steerAngle/2.7 * 0.;
                psides(i) = atan(preview.path(3) + 2*preview.path(2)*x(i-1) + 3*preview.path(1)*x(i-1)^2);
            end
            test.x = x;
            test.y = y;
            test.psi = psi;
            test.vel = vel;
            test.cte = cte;
            test.epsi = epsi;
            test.psides = psides;
        end
        function [lbx, ubx, lbg, ubg] = calB(obj, data, preview)
            lbx = zeros(obj.conf.nstep*6 + (obj.conf.nstep-1)*2,1);
            ubx = zeros(obj.conf.nstep*6 + (obj.conf.nstep-1)*2,1);
            lbg = zeros(obj.conf.nstep*6,1);
            ubg = zeros(obj.conf.nstep*6,1);
            for i=1:(obj.conf.deltai-1)
                lbx(i) = -2^32;
                ubx(i) = 2^32;
            end
            for i=obj.conf.deltai:(obj.conf.acceli-1)
                lbx(i) = - 1;
                ubx(i) = 1;
            end
            lbx(obj.conf.deltai) = data.steerAngle;
            ubx(obj.conf.deltai) = data.steerAngle;
            lbx(obj.conf.acceli) = data.accG(3)*obj.conf.car.g;
            ubx(obj.conf.acceli) = data.accG(3)*obj.conf.car.g;
            
            for i=1:(obj.conf.nstep-2)
                lbx(obj.conf.acceli+i) = -15;
                ubx(obj.conf.acceli+i) = 15;
            end
            lbx(obj.conf.xi) = 0;
            ubx(obj.conf.xi) = 0;
            lbx(obj.conf.yi) = 0;
            ubx(obj.conf.yi) = 0;
            lbx(obj.conf.psii) = 0;
            ubx(obj.conf.psii) = 0;
            lbx(obj.conf.veli) = data.localVelocity(3);
            ubx(obj.conf.veli) = data.localVelocity(3);
            lbx(obj.conf.ctei) = preview.path(4);
            ubx(obj.conf.ctei) = preview.path(4);
            lbx(obj.conf.epsii) = preview.path(3);
            ubx(obj.conf.epsii) = preview.path(3);
            
            for i = 1:obj.conf.nstep*6
                lbg(i) = 0;
                ubg(i) = 0;
            end
            
            lbg(obj.conf.ctei) = preview.path(4);
            ubg(obj.conf.ctei) = preview.path(4);
            lbg(obj.conf.epsii) = preview.path(3);
            ubg(obj.conf.epsii) = preview.path(3);
        end
        
        function [cmd_gas, cmd_brake, cmd_steer] = update(obj, data, preview)
            obj.x0 = zeros(obj.conf.nstep*6 + (obj.conf.nstep-1)*2,1);
            for i = 0:(obj.conf.nstep-1)
                obj.x0(obj.conf.xi+i) = i*data.localVelocity(3)*obj.conf.dt;
                obj.x0(obj.conf.yi+i) = 0;
                obj.x0(obj.conf.psii+i) = 0;
                obj.x0(obj.conf.veli+i) = data.localVelocity(3);
                obj.x0(obj.conf.ctei) = preview.path(4);
                obj.x0(obj.conf.epsii) = preview.path(3);
            end
            obj.x0(obj.conf.deltai) = data.steerAngle;
            obj.x0(obj.conf.acceli) = obj.conf.car.g*data.accG(3);
            
            f = obj.calF(preview);
            g = obj.calG(preview);
            [lbx,ubx,lbg,ubg] = obj.calB(data,preview);
            ipopt = struct('print_level', 0, 'max_cpu_time',5, 'max_iter', 100);
            opts = struct('verbose', false, 'ipopt', ipopt,'print_time', false);
            nlp = struct('x', obj.state, 'f', f, 'g', g);
            S = casadi.nlpsol('S', 'ipopt', nlp, opts);
            
            
%             res = 0;
            res = S('x0', obj.x0, 'ubx', ubx, 'lbx', lbx, 'ubg', ubg, 'lbg',lbg);
%             disp(obj.conf.car.g*data.accG(3))
%             disp(res.x(obj.conf.deltai:obj.conf.acceli-1))
%             disp(res.x(obj.conf.acceli:end));
            res.x = full(res.x);
            if res.x(obj.conf.acceli+1)>0
                cmd_gas = min(res.x(obj.conf.acceli+1)/obj.conf.car.g,1.0);
                cmd_brake = 0;
            else
                cmd_brake = min(abs(res.x(obj.conf.acceli+1)/obj.conf.car.g),1.0);
                cmd_gas = 0;
            end
            cmd_steer = res.x(obj.conf.deltai+1);
        end
            
        
    end
end

