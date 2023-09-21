function does_communicate = does_communicate(adjMatrix, node_i, node_j)
    visited = false(size(adjMatrix, 1), 1);
    does_communicate = dfs(adjMatrix, node_i, node_j, visited);
end

function does_communicate = dfs(adjMatrix, current_node, target_node, visited)
    visited(current_node) = true;

    % If the current node communicates with the target node directly
    if adjMatrix(current_node, target_node)
        does_communicate = true;
        return;
    end

    % Traverse neighbors and check for communication
    neighbors = find(adjMatrix(current_node, :));
    for neighbor = neighbors
        if ~visited(neighbor) && dfs(adjMatrix, neighbor, target_node, visited)
            does_communicate = true;
            return;
        end
    end

    does_communicate = false;
end