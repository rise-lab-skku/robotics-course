function rosmsgOut = Float32(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.Data = single(slBusIn.Data);
end
