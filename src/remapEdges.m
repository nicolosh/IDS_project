function E = remapEdges(E, Vmap)
    % ================ Used to change the vertices of a set of edges based on a given mapping of vertices =============
    % Input:     E     - Set of edges (represented as a pair of vertices)
    %           Vmap   - Map of connected vertices: change the vertices of each edge to the corresponding ones in the new mapping
    % Output:    Em    - Set of remapped edges

    Em = [Vmap(E(:,1)), Vmap(E(:,2))]; % remapping
    E = Em;
end

