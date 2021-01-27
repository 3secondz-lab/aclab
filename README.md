# aclab - matlab interface for Assetto Corsa

## Requirements
1. install https://sourceforge.net/projects/vjoystick/

2. install assetto corsa @ steam https://store.steampowered.com/app/244210/Assetto_Corsa/

3. copy /aclab_py into $(SteamInstallDir)/steamapps/common/assettocorsa/apps/python/

## Features
- Parse Assetto Corsa ai drive path, and warp into car oriented top view
- Repack fast_lane.ai with custom path
- Capture game image for vision process
- Control cars with MATLAB
- (future) Advanced controller with slip control
- (future) Human-compatitable racing strategy
- (future) Make driver's own avartar (driving mimic)

## More info

https://3secondz.com


## Usage

__MatlabExample__

```
% Create aclab object

s = aclab()


% After launch AC, get ai path
% ai path will be saved in s.path

s.getMap()


% Apply custom path (this effect after restart game)
% this affects files in game dir (fast_lane.ai), 
% the original file will be moved into $TRACKPATH/fast_lane.ai_bak

s.setMap( YOUR_GLOBAL_PATH_X, YOUR_GLOBAL_PATH_Y, YOUR_GLOBAL_SPEED )


% Get current game data
% data will be saved in s.data

s.poll()


% Get road preview
% PREVIEW_LENGTH is the number of points in original ai path from current position
% preview data will be saved in s.prev

s.prev(PREVIEW_LENGTH)

% Check preview data on plot

s.plotpreview()


% Push control set

s.push(CMD_GAS, CMD_BRAKE, CMD_STEER)


% In game setting, set control vJoy axis 1, 2, 3
% setcontroller(axis) is simple swipper to help setting

% Choose controller axis in game, then run setcontroller
% 1 : GAS PEDAL, 2 : BRAKE PEDAL, 3, STEERING

s.setcontroller(obj,AXIS_YOU_WANT) 


% Poll current game image
% the image will be saved in s.img
s.impoll()


```