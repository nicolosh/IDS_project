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
    sB = sort(B); % sort the elements of B
    X = sB(1,1):res:sB(end,1);
    Y = sB(1,2):res:sB(end,2);
    comb = combvec(X,Y)'; % generate all combinations of the elements in vectors X and Y.

    % Check if inside NFZ
    NFin = zeros(length(comb),1); % is inside a NFZ?
    NFon = zeros(length(comb),1); % is on the boundary of NFZ?
    for i=1:size(NFZ,2)
        % check which points in comb are inside (in) or on the boundary (on) of the i-th NFZ. 
        % comb(:,1) and comb(:,2) are the x and y coordinates of the points,
        % and NFZ{i}(:,1) and NFZ{i}(:,2) are the x and y coordinates of the vertices of the i-th NFZ.
        [in,on] = inpolygon(comb(:,1), comb(:,2), NFZ{i}(:,1), NFZ{i}(:,2));
        NFin = sign(NFin+in); % if a point is inside the i-th NFZ, the corresponding element in NFin will be set to 1
        NFon = sign(NFon+on); % if a point is on the boundary the i-th NFZ, the corresponding element in NFin will be set to 1
    end

    % Check if inside Borders
    [in,on] = inpolygon(comb(:,1), comb(:,2), B(:,1), B(:,2));

    % Construct Vertices
    V = [];         % cells vertices           
    Vm = [];        % motion vertices
    Vmap = [];      % motion vertices map

    for i = 1:length(comb)
        if(in(i) && ~(NFin(i) - NFon(i))) % checks if the ith point is inside the region/polygon defined by B and not inside a NFZ
            V = [V; comb(i,:)];
            if(~on(i) && ~NFin(i))        % not on boundary of region and not inside a NFZ
                Vm = [Vm; comb(i,:)];
                Vmap = [Vmap; length(V)];
            end
        end
    end

end

