
function [Best_pos,bestever,curve]=SLPSO(m,Max_iter,lb,ub,dim,fobj)

if(max(size(ub)) == 1)
   ub = ub.*ones(1,dim);
   lb = lb.*ones(1,dim);  
end

    %parameter initiliaztion
    M = 100;
    m = M + floor(dim/10);
    c3 = dim/M*0.01;
    PL = zeros(m,1);

    for i = 1 : m
        PL(i) = (1 - (i - 1)/m)^log(sqrt(ceil(dim/M)));
    end

   % initialization
    Range = ones(m,1)*(ub-lb);
    rand('seed', sum(100 * clock));
    p= rand(m,dim).*Range + ones(m,1)*lb;    % 初始化粒子群
    bestever = inf;
    
    FES=0;
    for i=1:m
        fitness(i)=fobj(p(i,:));
    end

    v = zeros(m,dim);
    curve(1)=bestever;
    
    gen = 2;

    % main loop
    while(gen <= Max_iter)
        % population sorting
        [fitness,rank] = sort(fitness, 'descend');
        p = p(rank,:);
        v = v(rank,:);
        
        % center position
        center = ones(m,1)*mean(p);
        
        %random matrix 
        randco1 = rand(m, dim);
        randco2 = rand(m, dim);
        randco3 = rand(m, dim);
        winidxmask = repmat([1:m]', [1 dim]); %Generate a m by dim matrix, where the i-th row is [i,i,…,i]
        winidx = winidxmask + ceil(rand(m, dim).*(m - winidxmask));
        pwin = p;
        for j = 1:dim
                pwin(:,j) = p(winidx(:,j),j);
        end
        
        % social learning
         lpmask = repmat(rand(m,1) < PL, [1 dim]);
         lpmask(m,:) = 0;
         v1 =  1*(randco1.*v + randco2.*(pwin - p) + c3*randco3.*(center - p));
         p1 =  p + v1;   
         
         
         v = lpmask.*v1 + (~lpmask).*v;         
         p = lpmask.*p1 + (~lpmask).*p;
         
         % boundary control
        for i = 1:m - 1
            p(i,:) = max(p(i,:), lb);
            p(i,:) = min(p(i,:), ub);
        end
        
        
        % fitness evaluation
        for i=1:m-1
            fitness(i)=fobj(p(i,:));
             if fitness(i)<bestever
                bestever=fitness(i);
                Best_pos=p(i,:);
             end
        end
        
        curve(gen)=bestever;
        gen = gen + 1;
    end

