function out = import_state(filename)
opts = delimitedTextImportOptions("NumVariables", 14);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["time", "x", "y", "z", "vx", "vy", "vz", "q1", "q2", "q3", "q4", "om1", "om2", "om3"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
out = readtable(filename, opts);


%% Clear temporary variables
clear opts
end
