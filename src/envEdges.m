function [Em, Am, A] = envEdges(V, Vm, Vmap, res)
    % ==================== Build the edges of the graph representing an environment =======================
    % Em: An edge is added to Em if there is a vertex in Vm (the array of motion vertices) 
    % that is res units away from the current vertex in either the x or y direction.
    
    % Ev: an edge is added to Ev if there is a vertex in V (the array of all vertices) that is res
    % units away from the current vertex in the x or y direction, and is either directly to the left or right,
    % or directly above or below the current vertex. This represents the fact that the current vertex
    % can "see" the other vertex.

    % N.B. Adjacency matrix is a mathematical representation of a graph. 
    % In this matrix, rows and columns correspond to graph nodes. 
    % The value in a specific cell indicates if exist a link between correspondent nodes (1) and is (0) if does not.

    % Construct Edges for building graphs
    Em = [];         % stores the motion edges of the graph
    Ev = [];         % stores the vision edges of the graph
        
    % Construct edges for movement and vision
    for i = 1:size(Vm,1)
        % MOVEMENT/MOTION in x-direction
        index_x = find(ismembertol(Vm, [Vm(i, 1) + res, Vm(i, 2)], 'ByRows', true));
        if sum(index_x) > 0
            Em = [Em; i, index_x];
        end
        % Check for movement in y-direction
        index_y = find(ismembertol(Vm, [Vm(i, 1), Vm(i, 2) + res], 'ByRows', true));
        if sum(index_y) > 0
            Em = [Em; i, index_y];
        end
            
        % VISION (in all directions)
        for dx = -res:res
            for dy = -res:res
                if dx == 0 && dy == 0
                    continue;           % Skip the case of no displacement
                end
                index_vision = find(ismembertol(V, [Vm(i, 1) + dx, Vm(i, 2) + dy], 'ByRows', true));
                if sum(index_vision) > 0
                    Ev = [Ev; Vmap(i), index_vision];
                end
            end
        end
    end

    % Get adjacency matrices for G and Gm graphs
    G = graph(Ev(:,1), Ev(:,2));
    Gm = graph(Em(:,1), Em(:,2));
    
    % A and Am represent the connectivity between vertices in the vision and movement graphs, respectively. 
    % They are used to quickly check if a path exists between two vertices without having
    % to traverse the graph
    A = adjacency(G);
    Am = adjacency(Gm);

end

