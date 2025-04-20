
function [Best_pos,Best_score,curve]=PSO(pop,Max_iter,lb,ub,dim,fobj)

if(max(size(ub)) == 1)
   ub = ub.*ones(1,dim);
   lb = lb.*ones(1,dim);  
end

% Extra variables for data visualization
average_objective = zeros(1, Max_iter);
curve = zeros(1, Max_iter);
FirstP_D1 = zeros(1 , Max_iter);

% Define the PSO's paramters
wMax = 0.9;
wMin = 0.5;
c1 = 2;
c2 = 2;
vMax = (ub - lb) .* 0.2;
vMin  = -vMax;

% The PSO algorithm

% Initialize the particles
for k = 1 : pop
    Swarm.Particles(k).X = (ub-lb) .* rand(1,dim) + lb;
    Swarm.Particles(k).V = zeros(1, dim);
    Swarm.Particles(k).PBEST.X = zeros(1,dim);
    Swarm.Particles(k).PBEST.O = inf;
    
    Swarm.GBEST.X = zeros(1,dim);
    Swarm.GBEST.O = inf;
end


% Main loop
for t = 1 : Max_iter
    
    % Calcualte the objective value
    for k = 1 : pop
        
        currentX = Swarm.Particles(k).X;
 
        Swarm.Particles(k).O = fobj(currentX);
        average_objective(t) =  average_objective(t)  + Swarm.Particles(k).O;
        
        % Update the PBEST
        if Swarm.Particles(k).O < Swarm.Particles(k).PBEST.O
            Swarm.Particles(k).PBEST.X = currentX;
            Swarm.Particles(k).PBEST.O = Swarm.Particles(k).O;
        end
        
        % Update the GBEST
        if Swarm.Particles(k).O < Swarm.GBEST.O
            Swarm.GBEST.X = currentX;
            Swarm.GBEST.O = Swarm.Particles(k).O;
        end
    end
    
    % Update the X and V vectors
    w = wMax - t .* ((wMax - wMin) / Max_iter);
    
    FirstP_D1(t) = Swarm.Particles(1).X(1);
    
    for k = 1 : pop
        Swarm.Particles(k).V = w .* Swarm.Particles(k).V + c1 .* rand(1,dim) .* (Swarm.Particles(k).PBEST.X - Swarm.Particles(k).X) ...
            + c2 .* rand(1,dim) .* (Swarm.GBEST.X - Swarm.Particles(k).X);
        
        
        % Check velocities
        index1 = find(Swarm.Particles(k).V > vMax);
        index2 = find(Swarm.Particles(k).V < vMin);
        
        Swarm.Particles(k).V(index1) = vMax(index1);
        Swarm.Particles(k).V(index2) = vMin(index2);
        
        Swarm.Particles(k).X = Swarm.Particles(k).X + Swarm.Particles(k).V;
        
        % Check positions
        index1 = find(Swarm.Particles(k).X > ub);
        index2 = find(Swarm.Particles(k).X < lb);
        
        Swarm.Particles(k).X(index1) = ub(index1);
        Swarm.Particles(k).X(index2) = lb(index2);
        
    end
    
    Best_pos=Swarm.GBEST.X;   
    curve(t) = Swarm.GBEST.O;
  
end
Best_score=curve(end);

end