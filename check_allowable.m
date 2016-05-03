function var = check_allowable(original, sd, lowerlim, upperlim)
% DESCRIPTION:
%   This file checks if any deviated lengths are negative (not possible)
%
% INPUT:
%   original: original variable
%   sd: how much deviation
%   lowerlim: absolute minimum of variable
%   upperlim: absolute maximum of variable
%
% OUTPUT:
%   var: correct possible variable
%
% REVISION HISTORY:
%   02/27: File created.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (original+sd <= lowerlim)
        var = lowerlim + 0.000001;
    elseif (original+sd >= upperlim)
        var = upperlim - 0.000001;
    else
        var = original+sd;
    end
end