clear all
clc
close all
SearchAgents_no=30; % Number of search agents
Max_iteration=1000; % Maximum number of iterations  

Function_name=3; %�趨���Ժ�����1-30.���д���11��Ҫά�ȴ���10
dim=30; %ά���趨��ά�ȿɹ�ѡ��Χ[2,10,20,30,50,100]������Function_name>=11�����ά������Ϊ10.
lb=-100;%�±߽�
ub=100;%�ϱ߽�
fobj = @(x) cec17_func(x',Function_name);

Max_test=10;
for i=1:Max_test
    disp(['��',num2str(i),'��ʵ��']);
    [Best_pos1(i,:),Best_score1(i),curve1(i,:)]=PSO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj); %��ʼ�Ż�
    [Best_pos2(i,:),Best_score2(i),curve2(i,:)]=SLPSO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj); %��ʼ�Ż�
    [Best_pos3(i,:),Best_score3(i),curve3(i,:)]=ESLPSO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj); %��ʼ�Ż�
end
% % %����Ա�
figure
semilogy(mean(curve1),'color','r','linewidth',2.0,'Marker','s','MarkerIndices',1:100:length(mean(curve1)))
hold on
semilogy(mean(curve2),'color','b','linewidth',2.0,'Marker','o','MarkerIndices',1:100:length(mean(curve2)))
semilogy(mean(curve3),'color','k','linewidth',2.0,'Marker','d','MarkerIndices',1:100:length(mean(curve3)))
title('Convergence curve of F2017_{3}')
xlabel('Iteration');
ylabel('Fitness');
axis tight%�� axis tight�����������������������յ���ʾͼ������ߣ������߽�Ŀհ�
grid off%��ʾ gca ����صĵ�ǰ��������ͼ���������ߡ��������ߴ�ÿ���̶������졣
box on %��ʾ��������Χ������
legend('PSO','SLPSO','ESLPSO')

disp('-------------------------------------------------')
display(['PSO 10��ʵ��������Ӧ��ֵ(Best) : ', num2str(min(Best_score1))]);
display(['PSO 10��ʵ�����Ž��Ӧ��ƽ����Ӧ��ֵ(mean) : ', num2str(mean(Best_score1))]);
display(['PSO 10��ʵ�������Ӧ��ֵ(worst) : ', num2str(max(Best_score1))]);
display(['PSO 10��ʵ���׼�std�� : ', num2str(std(Best_score1))]);

disp('-------------------------------------------------')
display(['SLPSO 10��ʵ��������Ӧ��ֵ(Best) : ', num2str(min(Best_score2))]);
display(['SLPSO 10��ʵ�����Ž��Ӧ��ƽ����Ӧ��ֵ(mean) : ', num2str(mean(Best_score2))]);
display(['SLPSO 10��ʵ�������Ӧ��ֵ(worst) : ', num2str(max(Best_score2))]);
display(['SLPSO 10��ʵ���׼�std�� : ', num2str(std(Best_score2))]);

disp('-------------------------------------------------')
display(['ESLPSO 10��ʵ��������Ӧ��ֵ(Best) : ', num2str(min(Best_score3))]);
display(['ESLPSO 10��ʵ�����Ž��Ӧ��ƽ����Ӧ��ֵ(mean) : ', num2str(mean(Best_score3))]);
display(['ESLPSO 10��ʵ�������Ӧ��ֵ(worst) : ', num2str(max(Best_score3))]);
display(['ESLPSO 10��ʵ���׼�std�� : ', num2str(std(Best_score3))]);



