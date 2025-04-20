clear all
clc
close all
SearchAgents_no=30; % Number of search agents
Max_iteration=1000; % Maximum number of iterations  

Function_name=3; %设定测试函数，1-30.其中大于11的要维度大于10
dim=30; %维度设定，维度可供选择范围[2,10,20,30,50,100]，其中Function_name>=11的最低维度设置为10.
lb=-100;%下边界
ub=100;%上边界
fobj = @(x) cec17_func(x',Function_name);

Max_test=10;
for i=1:Max_test
    disp(['第',num2str(i),'次实验']);
    [Best_pos1(i,:),Best_score1(i),curve1(i,:)]=PSO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj); %开始优化
    [Best_pos2(i,:),Best_score2(i),curve2(i,:)]=SLPSO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj); %开始优化
    [Best_pos3(i,:),Best_score3(i),curve3(i,:)]=ESLPSO(SearchAgents_no,Max_iteration,lb,ub,dim,fobj); %开始优化
end
% % %结果对比
figure
semilogy(mean(curve1),'color','r','linewidth',2.0,'Marker','s','MarkerIndices',1:100:length(mean(curve1)))
hold on
semilogy(mean(curve2),'color','b','linewidth',2.0,'Marker','o','MarkerIndices',1:100:length(mean(curve2)))
semilogy(mean(curve3),'color','k','linewidth',2.0,'Marker','d','MarkerIndices',1:100:length(mean(curve3)))
title('Convergence curve of F2017_{3}')
xlabel('Iteration');
ylabel('Fitness');
axis tight%用 axis tight命令可以让坐标轴调整到紧凑地显示图像或曲线，不留边界的空白
grid off%显示 gca 命令返回的当前坐标区或图的主网格线。主网格线从每个刻度线延伸。
box on %显示坐标区周围的轮廓
legend('PSO','SLPSO','ESLPSO')

disp('-------------------------------------------------')
display(['PSO 10次实验最优适应度值(Best) : ', num2str(min(Best_score1))]);
display(['PSO 10次实验最优解对应的平均适应度值(mean) : ', num2str(mean(Best_score1))]);
display(['PSO 10次实验最差适应度值(worst) : ', num2str(max(Best_score1))]);
display(['PSO 10次实验标准差（std） : ', num2str(std(Best_score1))]);

disp('-------------------------------------------------')
display(['SLPSO 10次实验最优适应度值(Best) : ', num2str(min(Best_score2))]);
display(['SLPSO 10次实验最优解对应的平均适应度值(mean) : ', num2str(mean(Best_score2))]);
display(['SLPSO 10次实验最差适应度值(worst) : ', num2str(max(Best_score2))]);
display(['SLPSO 10次实验标准差（std） : ', num2str(std(Best_score2))]);

disp('-------------------------------------------------')
display(['ESLPSO 10次实验最优适应度值(Best) : ', num2str(min(Best_score3))]);
display(['ESLPSO 10次实验最优解对应的平均适应度值(mean) : ', num2str(mean(Best_score3))]);
display(['ESLPSO 10次实验最差适应度值(worst) : ', num2str(max(Best_score3))]);
display(['ESLPSO 10次实验标准差（std） : ', num2str(std(Best_score3))]);



