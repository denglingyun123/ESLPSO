
function [Best_pos,bestever,curve]=ESLPSO(m,maxgen,lb,ub,dim,fobj)
tic
if(max(size(ub)) == 1)
   ub = ub.*ones(1,dim);
   lb = lb.*ones(1,dim);  
end

    %parameter initiliaztion
    M = 30;
    m = M ;
    c3 = dim/M*0.01;
    PL = zeros(m,1);
    
    uCR=0.5;%初始化交叉概率
    uF=0.5;%初始化缩放因子
    c=0.1;
    
    for i = 1 : m
        PL(i) = (1 - (i - 1)/m)^log(sqrt(ceil(dim/M)));
    end

   % initialization
    Range = ones(m,1)*(ub-lb);
    rand('seed', sum(100 * clock));
    p= rand(m,dim).*Range + ones(m,1)*lb;    % 初始化粒子群
    
    bestever = inf;
    for i=1:m
        fitness(i)=fobj(p(i,:));
        if fitness(i)<bestever
            bestever=fitness(i);
            Best_pos=p(i,:);
        end
    end

    v = zeros(m,dim);
    curve(1)=bestever;
    
    gen = 2;

    % main loop
    while(gen <= maxgen)
        
        %% ********Nonlinear control parameter********
        c3=(2+eps)-(2+2/3)*((exp(gen/maxgen)-1)/(exp(1)-1))^1;
        %% ********Nonlinear control parameter********
        
        Scr=[];%初始成功参加变异的个体的交叉概率为空集-------初始交叉概率设为空集
        Sf=[];%初始成功参加变异的个体的缩放因子为空集---------初始变异概率设为空集
        diff=[];%记录适应度之差
        n1=1;%记录Scr中的元素个数----
        n2=1;%记录Sf中的元素个数-----
        
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
        winidxmask = repmat([1:m]', [1 dim]);
        winidx = winidxmask + ceil(rand(m, dim).*(m - winidxmask));
        pwin = p;
        for j = 1:dim
                pwin(:,j) = p(winidx(:,j),j);
        end
        
        % social learning
         lpmask = repmat(rand(m,1) < PL, [1 dim]);
%          disp(lpmask)
         lpmask(m,:) = 0;
         v1 =  1*(randco1.*v + randco2.*(pwin - p) + c3*randco3.*(center - p));
         p1 =  p + v1;   
         
         v = lpmask.*v1 + (~lpmask).*v;         
         p_new = lpmask.*p1 + (~lpmask).*p;
         
         % boundary control
        for i = 1:m - 1
            p_new(i,:) = max(p_new(i,:), lb);
            p_new(i,:) = min(p_new(i,:), ub);
        end
        
        % fitness evaluation
        for i=1:m-1
            fitness_new(i)=fobj(p_new(i,:));
            if fitness_new(i)<fitness(i)
                p(i,:)=p_new(i,:);
                fitness(i)=fitness_new(i);
            end
            if fitness_new(i)<fitness(m)
                p(m,:)=p_new(i,:);
                fitness(m)=fitness_new(i);
            end
            if fitness(i)<bestever
                bestever=fitness(i);
                Best_pos=p(i,:);
            end
        end
        
        %% ******** Iteractive learning mechanism ********
        %Step1: 找到适应度最差的一半个体
        [fitness,idx]=sort(fitness);
        p=p(idx,:);
        
    for i=1:ceil(m/2)
       CR(i)=normrnd(uCR,0.1);%服从正态分布-----种群对应的每一个交叉概率
       F(i)=cauchyrnd(uF,0.1);%服从柯西分布-----种群对应的每一个变异概率
       while (CR(i)>1||CR(i)<0)
           CR(i)=normrnd(uCR,0.1);  %防止CR越界
       end
       while(F(i)<=0)
           F(i)=cauchyrnd(uF,0.1);
       end
       if (F(i)>1)                   %防止F越界
           F(i)=1;
       end
       
       r = generateR(ceil(m/2),i);
       jrand=randi([1,dim]); 
       for j=1:dim
           if (rand<=CR(i)||j==jrand)
              X_new1(i,j) = Best_pos(j) + F(i)*(p(r(2),j)-p(r(3),j)); % DE/rand/1
           else
              X_new1(i,j)=p(i,j)+F(i)*(Best_pos(j)-p(i,j))-F(i)*(p(r(2)+m/2,j)-p(r(3)+m/2,j)); %DE/current-to-rand/1
           end
       end
   end
   
   % Bound control
   for i = 1:ceil(m/2)
       for a = 1: dim
           if(X_new1(i,a)>ub(a))
               X_new1(i,a) =ub(a);
           end
           if(X_new1(i,a)<lb(a))
               X_new1(i,a) =lb(a);
           end
       end
   end
         
        %Greedy selection
   for i=1:ceil(m/2)
       fitness_new1(i) = fobj(X_new1(i,:));
       
       if fitness_new1(i)<fitness(i+m/2)
           p(i+m/2,:) = X_new1(i,:);
           fitness(i+m/2) = fitness_new1(i);
           Scr(n1)=CR(i);
           Sf(n2)=F(i);
           diff(n1)=abs(fitness_new1(i)-fitness(i+m/2));
           n1=n1+1;
           n2=n2+1;
       end 
       
       if(fitness_new1(i) < bestever)
           bestever = fitness_new1(i);
           Best_pos = X_new1(i,:);   
       end
   end
   
   %自适应参数，更新uCR和uF-------采用自适应参数的控制
        [~,ab]=size(Scr);
        sum1=0;
        for k=1:length(Scr)
            sum1=sum1+(diff(k)/sum(diff))*Scr(k);
        end
            
        if ab~=0
            newSf=(sum(Sf.^2))/(sum(Sf));
%             uCR=(1-c)*uCR+c.*mean(Scr);
            uCR=(1-c)*uCR+c.*sum1;
            uF=(1-c)*uF+c.*newSf;
        end
         %% ******** Iteractive learning mechanism ********
        curve(gen)=bestever;
        gen = gen + 1;
    end

