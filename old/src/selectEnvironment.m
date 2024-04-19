function env = selectEnvironment(shape, res)
    % Define the environments
    % Borders
    B1 = [0 0; 0 9; 7 9; 7 6; 3 6; 3 3; 7 3; 7 0];      % C Environment
    B2 = [0 0; 0 7; 4 7; 4 3; 6 3; 6 0];                % L Environment 
    B3 = [0 0; 0 6; 5 6; 5 0];                          % Rectangular Environment (rect_XxY)

    % Large Scale Environment
    B4 = [0 0; 0 9; 2 9; 2 14; 14 14; 14 0; 9 0; 9 1; 3 1; 3 4; 2 4; 2 0];

    % Define the No Fly Zones
    NFZ1 = {};
    NFZ2 = {2*[1 1;1 4;4 4;4 1]};
    NFZ3 = {[6 4; 7 4; 7 5; 6 5],[10 6; 12 6; 12 11; 5 11; 5 8; 10 8]};
    NFZ4 = {[10 6; 12 6; 12 11; 5 11; 5 8; 10 8]};

    % Select the environment and no fly zones based on the input
    switch shape
        case "C"
            env = Environment(B1, NFZ1, res);
        case "L"
            env = Environment(B2, NFZ1, res);
        case "R" % rectangular
            env = Environment(B3, NFZ1, res);
        case "Square1"
            b = [0 0; 4 0; 4 8; 0 8];
            env = Environment(b, NFZ1, res);
        case "Square2"
            b = 2*([0 0; 5 0; 5 5; 0 5]);
            env = Environment(b, NFZ2, res); 
        case "Large"
            env = Environment(B4, NFZ3, res);
        case "bho"
            env = Environment(B4, NFZ4, res);
        otherwise
            "dio2";        
    end
end