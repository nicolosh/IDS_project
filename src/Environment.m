classdef Environment < handle
    properties
        B         % Borders
        NFZ       % No Fly Zones
        Res       % Resolution of Grid (A higher resolution (smaller grid cells in which the space is discretized) would give more detailed results,
                  % but would also require more computational resources)
        V         % Vertices Coordinates of the graph 
        Vm        % Movement Vertices Coordinates
        Vmap      % Map: Movement Domain -> Environment Domain
        E         % Edges of graph
        Em        % Movement Edges
        A         % Adjacency Matrix of E (vision)
        Am        % Adjacency Matrix of Em (movement)
        H         % Heat Map of Vertices
        
        % For SEBS
        tl        % Last vision time of vertices
        Iv        % Idleness of movement vertices
        % An adjacent vertex to another vertex if there is an edge (or link) connecting them directly
    end
    methods
        % CONSTRUCTOR
        function env = Environment(b, nfz)
            env.B = b;
            env.NFZ = nfz;
        end
        
        % CELLULARIZATION
        function cellularize(env, res)
            env.Res = res;
            % Vertices
            [env.V,env.Vm,env.Vmap] = envCell(env.B, env.NFZ, res);
            % Initialize Heat
            env.heatReset();
            % Build Edges of the graph representing the env
            [env.Em,env.Am,env.A] = envEdges(env.V, env.Vm, env.Vmap, res);
            env.E = remapEdges(env.Em,env.Vmap); % Remap from movement to environment domain
            % Initialize Idleness
            env.IvReset();
        end
        
        % RESET HEAT
        function heatReset(env)
            env.H = ones(1,length(env.V));
        end
        
        % ============ ADD / UPDATE HEAT MAP ================
        % Define 2D Gaussian distribution Kernel in particular:
        % adds a Gaussian heat distribution to a grid at specified locations. 
        % The heat distribution is represented by a kernel, and the size, shape, 
        % and intensity of the kernel are determined by the parameters ksize, sigma, and alpha, respectively.
        function addHeat(env, ksize, alpha, sigma, v)    % sigma is stdDev, alpha = Amplitude of the distribution
            [R,C] = ndgrid(1:2*ksize-1, 1:2*ksize-1);
            exponent = ((R-ksize).^2 + (C-ksize).^2)./(2*sigma); % exponent based on the distance from the kernel center
            kernel = alpha*(exp(-exponent)); % Gaussian kernel computation
            % Allign the kernel to Grid
            if (ksize == 1) % kernel aligned at the location specified by v
                Vk = v;
            elseif (ksize > 1) % kernel aligned at the location specified by v and its neighbors in A
                Av = env.A;
                for k=2:ksize-1
                    Av = sign(Av + (Av*env.A)); % update adjacency matrix A
                end
                Vk = [v, find(Av(v,:))]; % heat added at the location(s) specified by v and its neighbors.
            else
                disp('Kernel size must be a positive number!')
            end
            % Updates Heat distribution on the grid
            for vk = Vk
                d = [env.V(vk,1) - env.V(v,1), env.V(vk,2) - env.V(v,2)]; % distance from the location to the kernel center
                env.H(vk) = env.H(vk) + kernel(ksize + d(1), ksize + d(2)); % heat at location vk + value of kernel at distance d
            end
        end
        % ==================================================
        
        % DEHEAT
        function deheat(env, v)
            env.H(logical(env.A(env.Vmap(v),:))) = 1; % sets the heat at the location(s) specified by v and its neighbors in the A adjacency matrix
        end
        
        % RESET IDLENESS OF VERTICES
        function IvReset(env)
            env.tl = zeros(1,length(env.V));
            env.Iv = zeros(1,length(env.V));
        end
        
        % UPDATE IDLENESS OF VERTICES
        function IvUpdate(env, t, v)
            env.tl(env.Vmap(v)) = t;                    % Set the vertex as viewed
            env.tl(logical(env.A(env.Vmap(v),:))) = t;  % Set all adjacent vertexes as viewed
            % Update Idleness
            for i = 1:length(env.V)
                env.Iv(i) = t - env.tl(i); % formula 9
            end
        end
        
        % PLOT BORDERS of the environment
        function plotBorders(env)
            plot(polyshape(env.B(:,1),env.B(:,2)),'FaceAlpha',0.2);
            hold on
            for n = 1:length(env.NFZ)
                plot(polyshape(env.NFZ{n}(:,1),env.NFZ{n}(:,2)),'FaceAlpha',0.2);
            end
            hold off
            axis equal
        end

        % PLOT GRAPHS of the environment
        function plotGraphs(env)
            plot(polyshape(env.B(:,1),env.B(:,2)),'FaceColor','cyan','FaceAlpha',0.2); % borders of env
            hold on
            plot(graph(env.A),'XData',env.V(:,1),'YData',env.V(:,2), 'EdgeColor', 'magenta', 'NodeColor','magenta','LineWidth', 1.5); % vision graph
            plot(graph(env.E(:,1),env.E(:,2),[],length(env.V)),'XData',env.V(:,1),'YData',env.V(:,2),'LineWidth', 2, 'EdgeColor', 'blue', 'NodeColor','blue'); % movement graph
            for n = 1:length(env.NFZ)
                plot(polyshape(env.NFZ{n}(:,1),env.NFZ{n}(:,2)),'FaceColor','red','FaceAlpha', 0.3);
            end
            hold off
            axis equal
            legend('Environment borders','Vision graph','Movement graph','NFZ')
        end
        
        % PLOT WEIGHT
        function plotWeight(env, res)
            [X,Y] = meshgrid(min(min(env.V)):1:max(max(env.V))); % used to create a 2-D grid of coordinates covering range of vertices in env.V
            Z = ones(size(X));
            % checks if each point on the grid is a member of the vertices of the environment within a
            % certain tolerance, and if so, it sets the corresponding element of the weight distribution Z to the
            % weight of the point in env.H 
            for x = 1:size(X,1)
                for y = 1:size(Y,1)
                    z = ismembertol(env.V,[X(x,x),Y(y,y)], 'ByRows', true);
                    if sum(z) > 0
                        Z(x,y) = env.H(z);
                    end
                end
            end
            Xq = min(min(env.V)):res:max(max(env.V)); % vector of x-coordinates with a step size of res that covers the range of the vertices in env.V
            Yq = Xq;
            [Xq,Yq] = meshgrid(Xq, Yq);
            Zq = interp2(X, Y, Z, Xq, Yq, 'spline'); % interpolation of a grid of points covering vertices range in env
            for x = 1:size(Xq,1)
                for y = 1:size(Yq,1)
                    if ~inpolygon(Xq(x,x), Yq(y,y), env.B(:,1), env.B(:,2)) % the point is inside the boundary of the environment ? 
                        Zq(x,y) = NaN;
                    end
                end
            end
            surf(Xq, Yq, Zq) % creates a 3-D surface plot of the weight distribution Zq over the grid Xq,Yq in the env
        end   
    end
end
