function slBusOut = Float32(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    slBusOut.Data = single(msgIn.Data);
end
