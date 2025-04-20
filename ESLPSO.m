
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
    
    uCR=0.5;%��ʼ���������
    uF=0.5;%��ʼ����������
    c=0.1;
    
    for i = 1 : m
        PL(i) = (1 - (i - 1)/m)^log(sqrt(ceil(dim/M)));
    end

   % initialization
    Range = ones(m,1)*(ub-lb);
    rand('seed', sum(100 * clock));
    p= rand(m,dim).*Range + ones(m,1)*lb;    % ��ʼ������Ⱥ
    
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
        
        Scr=[];%��ʼ�ɹ��μӱ���ĸ���Ľ������Ϊ�ռ�-------��ʼ���������Ϊ�ռ�
        Sf=[];%��ʼ�ɹ��μӱ���ĸ������������Ϊ�ռ�---------��ʼ���������Ϊ�ռ�
        diff=[];%��¼��Ӧ��֮��
        n1=1;%��¼Scr�е�Ԫ�ظ���----
        n2=1;%��¼Sf�е�Ԫ�ظ���-----
        
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
        %Step1: �ҵ���Ӧ������һ�����
        [fitness,idx]=sort(fitness);
        p=p(idx,:);
        
    for i=1:ceil(m/2)
       CR(i)=normrnd(uCR,0.1);%������̬�ֲ�-----��Ⱥ��Ӧ��ÿһ���������
       F(i)=cauchyrnd(uF,0.1);%���ӿ����ֲ�-----��Ⱥ��Ӧ��ÿһ���������
       while (CR(i)>1||CR(i)<0)
           CR(i)=normrnd(uCR,0.1);  %��ֹCRԽ��
       end
       while(F(i)<=0)
           F(i)=cauchyrnd(uF,0.1);
       end
       if (F(i)>1)                   %��ֹFԽ��
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
   
   %����Ӧ����������uCR��uF-------��������Ӧ�����Ŀ���
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

