
# coding: utf-8

# In[1]:


# import tensorflow as tf


import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import HaulTrainSimulator as sim
from deep_q_network4train_control_minibatch import DeepQNetwork


# ## Initialize the Envorionment 

# In[2]:



TotalRailwayLength = 40e3
Ne = 32
actionT = 10
#hHpeed_Limits = np.array([100, 100, 100])/3.6
hHpeed_Limits = np.array([70, 85, 75, 95])/3.6
lowSpeedLim = 15/3.6
speed_ChangePoint = np.array([0.2, 0.5, 0.7])
percentage_posRange = 0.9

rampList = np.array([[-10000, 0], 
                     [0, 0.001], 
                     [5000, 0.002], 
                     [10000, -0.001], 
                     [15000, 0.001], 
                     [21000, -0.002], 
                     [24000, 0.001], 
                     [32000, 0.001], 
                     [38000, 0.000], 
                     [40000, 0], 
                     [50000, 0], 
                     [60000, 0], 
                     [100000, 0]]) # 坡道
rampList[:, 1] *= 4

sim_env = sim.TrainSimulator(Ne = Ne, 
                             actionT = actionT, 
                             hHpeed_Limits = hHpeed_Limits,  
                             speed_ChangePoint = speed_ChangePoint, 
                             TotalLength = TotalRailwayLength, 
                             rampList = rampList, 
                             percentage_posRange = percentage_posRange,
                             is_enable_tensorflow = True,
                            )


# ## Initialize the DQN brain

# In[3]:


rangS = np.array([0, TotalRailwayLength], dtype = float)
rangV = np.array([0, 120], dtype = float)   # unit: km/h
rangA = np.array([-12, 12], dtype = int)
detaS = 2000.0
detaV = 40.0        # unit: km/h
nTiling = int(30)   # state  的编码
nTile = int(25)     # action 的编码

n_hiden_units = 256    # 单层，256 个节点

dqn_brain = DeepQNetwork(n_hiden_units = n_hiden_units, 
                         batch_size = 2^8,
                         exploration_size = Ne,
                         memory_batch_size = 400,
                         replace_target_iter = 10,
                         learning_rate = 2e-3,
                         rangS = rangS,
                         rangV = rangV,
                         rangA = rangA,
                         detaS = detaS,
                         detaV = detaV,
                         nTiling = nTiling,
                         detaA = nTile,
                         e_greedy_init = 0.4,
                         e_greedy_min = 0.1,
                         e_greedy_decrement = 2e-4,
                         output_graph = True,
                        )

DimSV = [150, 40]
state_s = np.linspace(rangS[0], rangS[1], DimSV[0])
state_v = np.linspace(rangV[0], rangV[1], DimSV[1])

Ms, Mv = np.meshgrid(state_s, state_v)
eval_state = np.concatenate((Ms.reshape([1, Ms.size]), Mv.reshape([1, Mv.size])), axis = 0)
state_coding = dqn_brain.state_coding_transform(eval_state)

Np = np.array([5, 5], dtype = int).reshape([2, 1])
action_coding = dqn_brain.action_coding_transform(Np)

# action_coding = dqn_brain.AllActionCode

sv_coding_0 = dqn_brain.state_coding_transform(np.array([6000, 40/3.6]).reshape(2, 1))
sv_coding_1 = dqn_brain.state_coding_transform(np.array([20000, 40/3.6]).reshape(2, 1))
sv_coding_2 = dqn_brain.state_coding_transform(np.array([32500, 40/3.6]).reshape(2, 1))
Ax, Ay = np.meshgrid(np.linspace(-12, 12, 25), np.linspace(-12, 12, 25))


# In[4]:



print(dqn_brain.n_total_actions)
print(dqn_brain.nAG)
print(dqn_brain.nTile)
print(dqn_brain.nNotch)

print(dqn_brain.n_total_features)
print(dqn_brain.epsilon)


# ## Begin Exploration 

# In[5]:


# 总探索步长
Total_simulation_step = 20000
display_time_step = 10
display_plot_step = 100

# 获取初始 action
action = sim_env.MotorNotch

time_start = time.time()

# 当前状态
state_ = sim_env.get_observation()
for itr in range(Total_simulation_step):
    
    if itr % display_time_step == 0:
        print('exploring step: %d, time comsumed: %f '%(itr, time.time()-time_start))
    if itr % (display_time_step*2) == 0:
        sim_env.show_phase_figure()
    if itr % (display_plot_step*2) == 0:
        Val = dqn_brain.sess.run(dqn_brain.q_eval, 
                                 feed_dict={dqn_brain.s: state_coding, dqn_brain.a: action_coding})
        Val_ = Val.reshape(DimSV[1], DimSV[0])
        fig = plt.figure(7)
        fig.set_size_inches(20, 5)
        ct = plt.contourf(Ms, Mv, Val_, 20, cmap=plt.get_cmap('rainbow'))
        plt.colorbar(ct)
        plt.show()
    if itr % display_plot_step == 0:
        val_s0 = dqn_brain.sess.run(dqn_brain.q_eval, 
                                    feed_dict={dqn_brain.s: sv_coding_0, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
        val_s1 = dqn_brain.sess.run(dqn_brain.q_eval, 
                                    feed_dict={dqn_brain.s: sv_coding_1, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
        val_s2 = dqn_brain.sess.run(dqn_brain.q_eval, 
                                    feed_dict={dqn_brain.s: sv_coding_2, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
        ind_0 = np.where(val_s0 == np.max(val_s0))
        ind_1 = np.where(val_s1 == np.max(val_s1))
        ind_2 = np.where(val_s2 == np.max(val_s2))
        fig0 = plt.figure(figsize=(20, 5))
        plt.subplot(1,3,1)
        cs0 = plt.contourf(Ax, Ay, val_s0, 10, cmap=plt.get_cmap('rainbow'))
        plt.colorbar(cs0)
        plt.plot(ind_0[1]-12, ind_0[0]-12, 'wo',markersize=15)
        plt.subplot(1,3,2)
        cs1 = plt.contourf(Ax, Ay, val_s1, 10, cmap=plt.get_cmap('rainbow'))
        plt.colorbar(cs1)
        plt.plot(ind_1[1]-12, ind_1[0]-12, 'wo',markersize=15)
        plt.subplot(1,3,3)
        cs2 = plt.contourf(Ax, Ay, val_s2, 10, cmap=plt.get_cmap('rainbow'))
        plt.plot(ind_2[1]-12, ind_2[0]-12, 'wo',markersize=15)
        plt.colorbar(cs2)
        plt.show()
    
        
    state, reward, bool_restart = sim_env.step(action)
    
    sim_env.set_initial_pos_par(dqn_brain.epsilon2)
    
    # 保存 transition，在 experience replay 中使用
    dqn_brain.store_transition(state_, np.transpose(action), reward, state)
    
    # 重新选择 action, action_ is a backup
    action_ = action
    action = dqn_brain.choose_action(np.transpose(state), action)
    
    # 覆盖当前状态
    state_ = state
    
    # 每一步都学习一次
    dqn_brain.learn()
    
    if np.any(bool_restart):
        sim_env.MotorNotch = action
        sim_env.init_simulation(bool_restart)    # 重置部分仿真器
        state_ = sim_env.get_observation()       # 重新获取当前状态
        action = sim_env.MotorNotch      # 获取初始 action
    

time_end = time.time()
print('\r\n totally cost',time_end-time_start)
print('\r\n time cost per step',(time_end-time_start)/itr)


# In[ ]:


print(sim_env.phase_buffer)


# In[ ]:


print(dqn_brain.memory_counter)
print(dqn_brain.memory_batch_size)
print(dqn_brain.memory_size)
print(dqn_brain.memory.shape)

sample_index = np.random.choice(dqn_brain.memory_size, size=dqn_brain.batch_size)
print(sample_index.shape)

print(dqn_brain.memory_counter)
print(dqn_brain.epsilon)


# In[ ]:



sv_coding_0 = dqn_brain.state_coding_transform(np.array([4000, 40/3.6]).reshape(2, 1))
sv_coding_1 = dqn_brain.state_coding_transform(np.array([9000, 40/3.6]).reshape(2, 1))
sv_coding_2 = dqn_brain.state_coding_transform(np.array([12500, 40/3.6]).reshape(2, 1))
val_s0 = dqn_brain.sess.run(dqn_brain.q_eval, 
                            feed_dict={dqn_brain.s: sv_coding_0, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
val_s1 = dqn_brain.sess.run(dqn_brain.q_eval, 
                            feed_dict={dqn_brain.s: sv_coding_1, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
val_s2 = dqn_brain.sess.run(dqn_brain.q_eval, 
                            feed_dict={dqn_brain.s: sv_coding_2, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
ind_0 = np.where(val_s0 == np.max(val_s0))
ind_1 = np.where(val_s1 == np.max(val_s1))
ind_2 = np.where(val_s2 == np.max(val_s2))
fig0 = plt.figure(figsize=(17, 5))
plt.subplot(1,3,1)
plt.contourf(Ax, Ay, val_s0, 10, cmap=plt.get_cmap('rainbow'))
plt.plot(ind_0[1]-12, ind_0[0]-12, 'wo',markersize=15)
plt.subplot(1,3,2)
plt.contourf(Ax, Ay, val_s1, 10, cmap=plt.get_cmap('rainbow'))
plt.plot(ind_1[1]-12, ind_1[0]-12, 'wo',markersize=15)
plt.subplot(1,3,3)
plt.contourf(Ax, Ay, val_s2, 10, cmap=plt.get_cmap('rainbow'))
plt.plot(ind_2[1]-12, ind_2[0]-12, 'wo',markersize=15)


# In[ ]:



ind_ = np.where(val_s1 == np.max(val_s1))
print(ind_[0])


# In[ ]:


sv_coding_0 = dqn_brain.state_coding_transform(np.array([5000, 40/3.6]).reshape(2, 1))
sv_coding_1 = dqn_brain.state_coding_transform(np.array([10000, 40/3.6]).reshape(2, 1))
Ax, Ay = np.meshgrid(np.linspace(-12, 12, 25), np.linspace(-12, 12, 25))

val_s0 = dqn_brain.sess.run(dqn_brain.q_eval, 
                            feed_dict={dqn_brain.s: sv_coding_0, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
val_s1 = dqn_brain.sess.run(dqn_brain.q_eval, 
                            feed_dict={dqn_brain.s: sv_coding_1, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
ind_0 = np.where(val_s0 == np.max(val_s0))
ind_1 = np.where(val_s1 == np.max(val_s1))

fig0 = plt.figure(figsize=(14, 6))
plt.subplot(1,2,1)
plt.contourf(Ax, Ay, val_s0, 10, cmap=plt.get_cmap('rainbow'))
plt.plot(ind_0[1]-12, ind_0[0]-12, 'wo',markersize=12)
plt.subplot(1,2,2)
plt.contourf(Ax, Ay, val_s1, 10, cmap=plt.get_cmap('rainbow'))
plt.plot(ind_1[1]-12, ind_1[0]-12, 'wo',markersize=12)


# In[ ]:



sv_coding_0 = dqn_brain.state_coding_transform(np.array([5000, 40/3.6]).reshape(2, 1))
sv_coding_1 = dqn_brain.state_coding_transform(np.array([10000, 40/3.6]).reshape(2, 1))

Ax, Ay = np.meshgrid(np.linspace(-12, 12, 25), np.linspace(-12, 12, 25))

val_s0 = dqn_brain.sess.run(dqn_brain.q_eval, 
                            feed_dict={dqn_brain.s: sv_coding_0, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
val_s1 = dqn_brain.sess.run(dqn_brain.q_eval, 
                            feed_dict={dqn_brain.s: sv_coding_1, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
fig0 = plt.figure(figsize=(16, 5))
ax1 = fig0.add_subplot(121, projection='3d')
ax1.plot_surface(Ax, Ay, val_s0, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))

ax2 = fig0.add_subplot(122, projection='3d')
ax2.plot_surface(Ax, Ay, val_s1, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))


# In[ ]:


Val = dqn_brain.sess.run(dqn_brain.q_eval, 
                         feed_dict={dqn_brain.s: state_coding, dqn_brain.a: action_coding})
Val_ = Val.reshape(DimSV[1], DimSV[0])
fig = plt.figure(7)
fig.set_size_inches(20, 8)
ct = plt.contourf(Ms, Mv, Val_, 50, cmap=plt.get_cmap('rainbow'))
plt.colorbar(ct)
plt.show()


# In[ ]:


Val = dqn_brain.sess.run(dqn_brain.q_eval, 
                         feed_dict={dqn_brain.s: state_coding, dqn_brain.a: action_coding})
max_val = np.max(np.max(Val_, axis = 1), axis = 0)
min_val = np.min(np.min(Val_, axis = 1), axis = 0)

Val_ = Val.reshape(DimSV[1], DimSV[0])
fig = plt.figure(7)
fig.set_size_inches(16, 9)
ax = Axes3D(fig) 
ax.plot_surface(Ms, Mv, -Val_, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
ax.contourf(Ms, Mv, -Val_, zdir='z', offset = -max_val*1.2-30, cmap=plt.get_cmap('coolwarm'))
ax.set_zlim(-max_val*1.2 -30, -min_val*1.2)
plt.show()


# In[ ]:


a = dqn_brain.current_transition_batch
print(a.shape)


# In[ ]:


#print(dqn_brain.memory)
#print(dqn_brain.current_transition_batch)

a = dqn_brain.current_transition_batch
#a = dqn_brain.memory


fig2 = plt.figure(2)
#plt.plot(a[:, 2], a[:, 3], '.')
plt.plot(a[:, 0], a[:, 1], '.')
plt.show()

fig3 = plt.figure(3)
plt.plot(a[:, 2], a[:, 3], '.') 
plt.show()


# In[ ]:


val_s0 = dqn_brain.sess.run(dqn_brain.q_eval, 
                            feed_dict={dqn_brain.s: sv_coding_0, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
val_s1 = dqn_brain.sess.run(dqn_brain.q_eval, 
                            feed_dict={dqn_brain.s: sv_coding_1, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
val_s2 = dqn_brain.sess.run(dqn_brain.q_eval, 
                            feed_dict={dqn_brain.s: sv_coding_2, dqn_brain.a: dqn_brain.AllActionCode}).reshape([25, 25])
ind_0 = np.where(val_s0 == np.max(val_s0))
ind_1 = np.where(val_s1 == np.max(val_s1))
ind_2 = np.where(val_s2 == np.max(val_s2))
fig0 = plt.figure(figsize=(20, 5))
plt.subplot(1,3,1)
cs0 = plt.contourf(Ax, Ay, val_s0, 10, cmap=plt.get_cmap('rainbow'))
plt.colorbar(cs0)
plt.plot(ind_0[1]-12, ind_0[0]-12, 'wo',markersize=15)
plt.subplot(1,3,2)
cs1 = plt.contourf(Ax, Ay, val_s1, 10, cmap=plt.get_cmap('rainbow'))
plt.colorbar(cs1)
plt.plot(ind_1[1]-12, ind_1[0]-12, 'wo',markersize=15)
plt.subplot(1,3,3)
cs2 = plt.contourf(Ax, Ay, val_s2, 10, cmap=plt.get_cmap('rainbow'))
plt.plot(ind_2[1]-12, ind_2[0]-12, 'wo',markersize=15)
plt.colorbar(cs2)
plt.show()


# In[ ]:



#print('\r\n time cost per step',(time_end-time_start)/itr)


# In[ ]:


dqn_brain.plot_cost()


# In[ ]:



V = np.mat([52.686, 52.818]).reshape([2, 1])
MotorNotch_ = np.array([-2, -5], dtype = int).reshape([2, 1])

print(V.shape)
print(MotorNotch_.shape)

print(sim.GetLocomotiveF(V, MotorNotch_, sim_env.TBcl))


# In[ ]:


a = np.random.randint(0, 99, [2, 10])
print(a)
print(a[:, 1].reshape([2, 1]))

