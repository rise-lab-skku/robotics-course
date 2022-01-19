function slBusOut = Bool(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    slBusOut.Data = logical(msgIn.Data);
end
