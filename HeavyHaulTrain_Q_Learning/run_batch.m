
clear;clc

%% 总体参数

TotalLength = 40e3;
train_steps = 1e4;
learning_rate = 1e-5;

%% 
% reward_weigh = [0.3 0 0];   % 速度 车钩力， 总能耗 % ------------- weight 多个目标之间的相互权重大小
% save_data_string = ['AllResult_20181011_100_' '0_0' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 0)
% 
% save_data_string = ['AllResult_20181011_100_' '0_1' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 1)
% 
% save_data_string = ['AllResult_20181011_100_' '1_0' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 0)
% 
% save_data_string = ['AllResult_20181011_100_' '1_1' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 1)
% 
% 
% %% 
% reward_weigh = [0.3 2/1e6 5/1e6*1.5];   % 速度 车钩力， 总能耗 % ------------- weight 多个目标之间的相互权重大小
% save_data_string = ['AllResult_20181011_111_' '0_0' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 0)
% 
% save_data_string = ['AllResult_20181011_111_' '0_1' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 1)

% save_data_string = ['AllResult_20181011_111_' '1_0' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 0)
% 
% save_data_string = ['AllResult_20181011_111_' '1_1' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 1)


%% 
% reward_weigh = [0.3  4/1e6 1/1e5*1.5];   % 速度 车钩力， 总能耗 % ------------- weight 多个目标之间的相互权重大小
% save_data_string = ['AllResult_20181011_122_' '0_0' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 0)
% 
% save_data_string = ['AllResult_20181011_122_' '0_1' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 1)
% 
% save_data_string = ['AllResult_20181011_122_' '1_0' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 0)
% 
% save_data_string = ['AllResult_20181011_122_' '1_1' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 1)



%% 
% reward_weigh = [0.3  2/1e6 1/1e5*1.5];   % 速度 车钩力， 总能耗 % ------------- weight 多个目标之间的相互权重大小
% save_data_string = ['AllResult_20181011_112_' '0_0' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 0)
% 
% save_data_string = ['AllResult_20181011_112_' '0_1' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 1)
% 
% save_data_string = ['AllResult_20181011_112_' '1_0' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 0)
% 
% save_data_string = ['AllResult_20181011_112_' '1_1' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 1)




%% 
% reward_weigh = [0.3  4/1e6 5/1e6*1.5];   % 速度 车钩力， 总能耗 % ------------- weight 多个目标之间的相互权重大小
% save_data_string = ['AllResult_20181011_121_' '0_0' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 0)
% 
% save_data_string = ['AllResult_20181011_121_' '0_1' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 1)
% 
% save_data_string = ['AllResult_20181011_121_' '1_0' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 0)
% 
% save_data_string = ['AllResult_20181011_121_' '1_1' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 1)



%% 
reward_weigh = [0.3 0 5/1e6*1.5];   % 速度 车钩力， 总能耗 % ------------- weight 多个目标之间的相互权重大小
% save_data_string = ['AllResult_20181011_101_' '0_0' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 0)
% 
% save_data_string = ['AllResult_20181011_101_' '0_1' '.mat'];
% HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 1)

save_data_string = ['AllResult_20181011_101_' '1_0' '.mat'];
HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 0)

save_data_string = ['AllResult_20181011_101_' '1_1' '.mat'];
HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 1)




%% 
reward_weigh = [0.3 2/1e6 0];   % 速度 车钩力， 总能耗 % ------------- weight 多个目标之间的相互权重大小
save_data_string = ['AllResult_20181011_110_' '0_0' '.mat'];
HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 0)

save_data_string = ['AllResult_20181011_110_' '0_1' '.mat'];
HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 0, 1)

save_data_string = ['AllResult_20181011_110_' '1_0' '.mat'];
HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 0)

save_data_string = ['AllResult_20181011_110_' '1_1' '.mat'];
HeavyHaulTrain_OptimizeRL_DNN(save_data_string, TotalLength, train_steps, learning_rate, reward_weigh, 1, 1)









