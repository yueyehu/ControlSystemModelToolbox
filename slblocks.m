function blkStruct = slblocks
% This function specifies that the library should appear
% in the Library Browser
% and be cached in the browser repository

% The function that will be called when the user double-clicks on
% this icon.
% Example:  blkStruct.OpenFcn = 'dsplib';
blkStruct.OpenFcn = 'ExampleTF=tf([1 0],[1 1]);ctsmodel;';%.mdl file

% Define the library list for the Simulink Library browser.
% Return the name of the library model and the name for it
Browser.Library = 'ctsmodel';
Browser.Name = 'Control System Model Toolbox';

blkStruct.Browser = Browser; 