%
% MATLAB vJoy interface
% 
% Author: Yuujin Hwang <yoonjinh@kaist.ac.kr>
%                      <yoonjinh@3secondz.com>
% Repository: github.com/3secondz-lab/mvJoy

% This program is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.
%

classdef mvJoy < handle
    %MVJOY - Matlab Wrapper for vJoy Feeder
    %   Yuujin Hwang
    %   github.com/3secondz-lab/mvJoy
    
    properties
        libName = 'vJoyInterface'
        libPath
        dllPath
        rID
        data
        notfound
        warnings
    end
    
    methods
        function obj = mvJoy(rID)
            %MVJOY - Matlab Wrapper for vJoy Feeder
            %   Yuujin Hwang
            %   github.com/3secondz-lab/mvJoy
            obj.rID = rID;
            
            obj.libPath = fileparts(mfilename('fullpath'));
            if(computer('arch') == "win64")
                obj.dllPath = fullfile(obj.libPath,'utils','x64',...
                    'vJoyInterface.dll');
            else
                obj.dllPath = fullfile(obj.libPath,'utils','x86',...
                    'vJoyInterface.dll');
            end
            
            if ~libisloaded('vJoyInterface')
                [obj.notfound, obj.warnings] = loadlibrary(obj.dllPath);
            end
            if ~isempty(obj.notfound)
                disp(obj.notfound)
            end
            if ~isempty(obj.warnings)
                disp(obj.warnings)
            end
            
            if calllib(obj.libName,'vJoyEnabled')
                disp('Some error message boxes can be generated from DLL');
                disp('This will be fixed in further update');
                if calllib(obj.libName,'AcquireVJD',obj.rID)
                    disp(['vJoyInterface' obj.rID 'connected']);
                else
                    disp(['Cannot open vJoyInterface' obj.rID]);
                end
            else
                disp('vJoy is not running')
            end
            
            obj.data = libstruct('s_JOYSTICK_POSITION_V2');
            obj.data.bDevice = rID;
            
            disp(['mvJoy:running on vJoyDevice ',obj.rID]);
            
            
        end
        
        function resp = setButton(obj,buttonID, state)
            %Set a given button
            %   buttonID - numbered from 1
            %   State - true or false
            obj.data.lButtons = bitset(obj.data.lButtons, buttonID, state);
            resp = callib(obj.libName, 'SetBtn', state, obj.rID, buttonID);
        end
        
        function resp = setAxis(obj, axisID, axisValue)
            %Set a given axis
            %   axisID - ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz', 'sl0', 'sl1']
            %   axisValue - 0 to 32768
            switch axisID
                case 'X'
                    axisID = 48;
                    obj.data.wAxisX = axisValue;
                case 'Y'
                    axisID = 49;
                    obj.data.wAxisY = axisValue;
                case 'Z'
                    axisID = 50;
                    obj.data.wAxisZ = axisValue;
                case 'Rx'
                    axisID = 51;
                    obj.data.wAxisXRot = axisvalue;
                case 'Ry'
                    axisID = 52;
                    obj.data.wAxisYRot = axisvalue;
                case 'Rz'
                    axisID = 53;
                    obj.data.wAxisZRot = axisvalue;
                case 'sl0'
                    axisID = 54;
                    obj.data.wSlider = axisvalue;
                case 'sl1'
                    axisID = 55;
                    obj.data.wDial = axisvalue;
                case 1
                    axisID = 48;
                    obj.data.wAxisX = axisValue;
                case 2
                    axisID = 49;
                    obj.data.wAxisY = axisValue;
                case 3
                    axisID = 50;
                    obj.data.wAxisZ = axisValue;
                case 4'
                    axisID = 51;
                    obj.data.wAxisXRot = axisvalue;
                case 5
                    axisID = 52;
                    obj.data.wAxisYRot = axisvalue;
                case 6
                    axisID = 53;
                    obj.data.wAxisZRot = axisvalue;
                case 7
                    axisID = 54;
                    obj.data.wSlider = axisvalue;
                case 8
                    axisID = 55;
                    obj.data.wDial = axisvalue;
                otherwise
                    e = MException('mvJoy:noMatchingAxis',...
                    'axisID %s is not valid',axisID);
                    throw(e);
            end
            
            if~(axisID>=48 && axisID<56)
                e = MException('mvJoy:AxisOutofRange',...
                'axisID %d mube be in range [48,55]',axisID);
                throw(e);
            else
                resp = calllib(obj.libName,'SetAxis',...
                    axisValue,obj.rID,axisID);
            end
        end
        
        function resp = setDiscPov(obj, povID, povValue)
            %Write value to a given discrete POV
            if (povValue<-1 || povValue>3)
                e = MException('mvJoy:povValue %d must be in range [-1,3]',...
                    povValue);
                throw(e);
            end
            if (povID<1 || povID>4)
                e = MException('mvJoy:povID %d must be in range [1,4]',povID);
                throw(e);
            end
            resp = calllib(obj.libName,'setDiscPov',...
                povValue,obj.rID,povID);
        end
        
        function resp = setContPov(obj, povID, povValue)
            %Write value to a given discrete POV
            if (povValue<-1 || povValue>35999)
                e = MException('mvJoy:povValue %d must be in range [-1,35999]',...
                    povValue);
                throw(e);
            end
            if (povID<1 || povID>4)
                e = MException('mvJoy:povID %d must be in range [1,4]',povID);
                throw(e);
            end
            resp = calllib(obj.libName,'setContPov',...
                povValue,obj.rID,povID);
        end
        
        function resp = reset(obj)
            obj.data = libstruct('s_JOYSTICK_POSITION_V2');
            obj.data.bDevice = obj.rID;
            resp = calllib(obj.libName,'ResetVJD',obj.rID);
        end
        
        function resp = resetButtons(obj)
            obj.data.lButtons = 0;
            resp = calllib(obj.libName,'ResetButtons',obj.rID);
        end
        
        function resp = resetPovs(obj)
            resp = calllib(obj.libName,'ResetPovs',obj.rID);
        end
        
        function resetAll(obj)
            calllib(obj.libName,'ResetAll');
        end
        
        function resp = update(obj)
            resp = calllib(obj.libName,'UpdateVJD',obj.rID,...
                libpointer('s_JOYSTICK_POSITION_V2', obj.data));
        end
        
        function delete(obj)
            calllib(obj.libName, 'RelinquishVJD', obj.rID);
            r = calllib(obj.libName, 'GetVJDStatus', obj.rID);
            if(r=="VJD_STAT_OWN")
                e = MException('mvJoy:cannotRelinquishVJD',...
                    'release vJoy manually');
                throw(e);
            else
                disp(['mvJoy:release vJoyDevice',obj.rID]);
            end
        end
    end
end

