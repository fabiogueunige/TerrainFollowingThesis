function printDebug(formatString, varargin)
    global DEBUG
    if DEBUG
        fprintf(formatString, varargin{:});
    end
end