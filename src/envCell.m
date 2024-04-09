function [V, Vm, Vmap] = envCell(B, NFZ, res)
    %  ===================== Given the environment borders and No Fly Zones returns a ======================
    %  ===================== cellularized version of the environment with the desired resolution. ==========
    %  Inputs:    B      - Environment Borders
    %            NFZ     - No Fly Zones as list of borders {[...],[...]}
    %            res     - Cell resolution
    %  Outputs:   V      - Cells vertices
    %            Vm      - motion (connected) vertices
    %            Vmap    - Map of motion (connected) vertices

    % V: array memorizing cells vertices. is a point inside the region,
    % not inside a NFZ

    % Vm: array memorizing motion vertices. motion vertex is a point inside the region,
    % not inside a NFZ and not on the region board. These vertices/points represent
    % where an object can move.

    % Vmap: array mapping motion vertices to the corresponding cells vertices.

    % Consider the whole rectangle containing the environment
    sortedBorders = sort(B); % sort the elements of B
    Xrange = sortedBorders(1,1):res:sortedBorders(end,1);
    Yrange = sortedBorders(1,2):res:sortedBorders(end,2);
    gridPoints = combvec(Xrange, Yrange)'; % generate all combinations of the elements in vectors Xrange and Yrange

    % Check if inside NFZ
    for i=1:length(NFZ)
        % if a point is inside the i-th NFZ, the corresponding element in NFin will be set to 1
        % if a point is on the boundary the i-th NFZ, the corresponding element in NFin will be set to 1
        [NFin, NFon] = inpolygon(gridPoints(:,1), gridPoints(:,2), NFZ{i}(:,1), NFZ{i}(:,2));            % Check if each point is inside or on the border of the NFZ
    end

    % Check if inside Borders
    [in,on] = inpolygon(gridPoints(:,1), gridPoints(:,2), B(:,1), B(:,2));

    % Construct Vertices
    V = [];         % cells vertices           
    Vm = [];        % motion vertices
    Vmap = [];      % motion vertices map

    for i = 1:length(gridPoints)
        if(in(i) && ~(NFin(i) - NFon(i))) % checks if the ith point is inside the region/polygon defined by B and not inside a NFZ
            V = [V; gridPoints(i,:)];
            if(~on(i) && ~NFin(i))        % not on boundary of region and not inside a NFZ
                Vm = [Vm; gridPoints(i,:)];
                Vmap = [Vmap; length(V)];
            end
        end
    end

end

