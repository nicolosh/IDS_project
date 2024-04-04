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
        
    for i = 1:size(Vm,1)
        % MOVEMENT/MOTION
        index = find(ismembertol(Vm,[Vm(i,1)+res Vm(i,2)],'ByRows',true));
        if(sum(index) > 0)        
            Em = [Em; i index];
        end
        index = find(ismembertol(Vm,[Vm(i,1) Vm(i,2)+res],'ByRows',true));
        if(sum(index) > 0)
            Em = [Em; i index];
        end
            
        % VISION
        index = find(ismembertol(V,[Vm(i,1)-res Vm(i,2)],'ByRows',true));
        if(sum(index) > 0)
            Ev = [Ev; Vmap(i) index];
        end
        index = find(ismembertol(V,[Vm(i,1)+res Vm(i,2)],'ByRows',true));
        if(sum(index) > 0)
            Ev = [Ev; Vmap(i) index];
        end
        index = find(ismembertol(V,[Vm(i,1) Vm(i,2)-res],'ByRows',true));
        if(sum(index) > 0)
            Ev = [Ev; Vmap(i) index];
        end
        index = find(ismembertol(V,[Vm(i,1) Vm(i,2)+res],'ByRows',true));
        if(sum(index) > 0)
            Ev = [Ev; Vmap(i) index];
        end
        index = find(ismembertol(V,[Vm(i,1)-res Vm(i,2)-res],'ByRows',true));
        if(sum(index) > 0)
            Ev = [Ev; Vmap(i) index];
        end
        index = find(ismembertol(V,[Vm(i,1)-res Vm(i,2)+res],'ByRows',true));
        if(sum(index) > 0)
            Ev = [Ev; Vmap(i) index];
        end
        index = find(ismembertol(V,[Vm(i,1)+res Vm(i,2)+res],'ByRows',true));
        if(sum(index) > 0)
            Ev = [Ev; Vmap(i) index];
        end
        index = find(ismembertol(V,[Vm(i,1)+res Vm(i,2)-res],'ByRows',true));
        if(sum(index) > 0)
            Ev = [Ev; Vmap(i) index];
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

