
# coding: utf-8

# In[1]:


import numpy as np
import tensorflow as tf

#np.random.seed(1)
#tf.set_random_seed(1)


# In[2]:



# Deep Q Network off-policy
# Double Tile Coding (State and Action coding)

# initialize tile_coding and feature vector
#rangS = np.array([0, 15000], dtype = float)
#rangV = np.array([0, 120], dtype = float)
#rangA = np.array([-12, 12], dtypt = int)
#detaS = 400.0
#detaV = 40.0
#nTiling = int(30)   # state  的编码
#nTile = int(25)     # action 的编码
        
class DeepQNetwork:
    def __init__(
            self,
            n_hiden_units,
            exploration_size,
            batch_size = 64,
            n_states = 2,
            n_action_dim = 2,
            rangS = np.array([0, 15000], dtype = float),
            rangV = np.array([0, 120], dtype = float),
            rangA = np.array([-12, 12], dtype = int),
            detaS = 400.0,
            detaV = 40.0,
            detaA = 25,
            nTiling = int(30),
            nTile = int(25),
            learning_rate = 0.01,
            reward_decay = 0.95,
            e_greedy_init = 0.2,
            e_greedy_min = 0.01,
            replace_target_iter = 300,
            memory_batch_size = 64,
            e_greedy_decrement = None,
            output_graph = False,
    ):
        self.n_hiden_units = n_hiden_units
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon_min = e_greedy_min
        self.replace_target_iter = replace_target_iter
        self.batch_size = batch_size                  
        self.exploration_size = exploration_size      # 并行探索的size
        self.epsilon_decrement = e_greedy_decrement
        self.epsilon = e_greedy_init if e_greedy_decrement is not None else self.epsilon_min
        
        self.epsilon2 = 1.0;
        self.epsilon2_decrement = 2e-4;
        self.epsilon2_min = 0.3;
        
        self.detaA = nTile

        self.n_states = n_states
        self.n_action_dim = n_action_dim
        self.memory_batch_size = memory_batch_size
        self.memory_size = memory_batch_size*self.exploration_size
        
        # 可选 action， 一维
        self.a_101 = np.array([-1, 0, 1], dtype = int)   
        
        # 可选 action， 二维
        self.G = np.array([[0, 0], 
                          [1, 0], 
                          [2, 0], 
                          [0, 1], 
                          [1, 1], 
                          [2, 1], 
                          [0, 2], 
                          [1, 2], 
                          [2, 2]], dtype = int)
        
        self.n_p_actions = self.G.shape[0]
        
        # initialize tile_coding and feature vector
        #rangS = np.array([0, 15000], dtype = float)
        #rangV = np.array([0, 120], dtype = float)
        #rangA = np.array([-12, 12], dtypt = int)
        #detaS = 400.0
        #detaV = 40.0
        #nTiling = int(30)   # state  的编码
        #nTile = int(25)     # action 的编码
        self.initialize_tile_coding(rangS, rangV, detaS, detaV, nTiling, rangA, detaA)
        self.nNegNotch = int(-rangA[0])
        
        # 初始化一系列重要的字典，关于获取 action 以及其编码的
        self.initialize_action_dicts()
        
        # total learning step
        self.learn_step_counter = 0

        # initialize zero memory [s, a, r, s_]
        self.memory = np.zeros((self.memory_size, self.n_states + 1+ self.n_action_dim + self.n_states))

        # consist of [target_net, evaluate_net]
        self._build_net()

        t_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='target_net')
        e_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='eval_net')

        with tf.variable_scope('soft_replacement'):
            self.target_replace_op = [tf.assign(t, e) for t, e in zip(t_params, e_params)]

        self.sess = tf.Session()

        if output_graph:
            # $ tensorboard --logdir=logs
            tf.summary.FileWriter("logs/", self.sess.graph)

        self.sess.run(tf.global_variables_initializer())
        self.cost_his = []
        
    def __del__(self):
        self.sess.close()

    def _build_net(self):
        # ------------------ all inputs ------------------------ , each time a single piece of data
        self.s = tf.placeholder(tf.int32, [self.n_features, None], name='s')  # input State
        self.s_ = tf.placeholder(tf.int32, [self.n_features, None], name='s_')  # input Next State
        self.r = tf.placeholder(tf.float32, [1, None], name='r')  # input Reward
        self.a = tf.placeholder(tf.int32, [self.n_actions, None], name='a')  # input Action
        # a_ -> [n_actions,  next_action_size, batch_size], name='a')  # input Next Action
        self.a_ = tf.placeholder(tf.int32, [self.n_actions, None, None], name='a')  # input Next Action

        w_initializer, b_initializer = tf.random_normal_initializer(0., 0.03), tf.random_normal_initializer(0., 0.003)
        #w_initializer, b_initializer = tf.random_normal_initializer(0., 0.03), tf.constant_initializer(0.01)

        # ------------------ build evaluate_net ------------------
        with tf.variable_scope('eval_net'):
            e_w1 = tf.get_variable('e_w1', [self.n_total_features, self.n_hiden_units], initializer=w_initializer)
            e_b1 = tf.get_variable('e_b1', [1, self.n_hiden_units], initializer=b_initializer)

            e_w2 = tf.get_variable('e_w2', [self.n_hiden_units, self.n_total_actions], initializer=w_initializer) 
            e_b2 = tf.get_variable('e_b2', [self.n_total_actions, 1], initializer=b_initializer)

            #e_l1 = tf.reduce_sum(tf.gather(e_w1, self.s), axis = 0) + e_b1
            e_l1 = tf.nn.relu(tf.reduce_sum(tf.gather(e_w1, self.s), axis = 0) + e_b1)
            self.q_eval = tf.reduce_sum(tf.multiply(e_l1, tf.gather(tf.transpose(e_w2), self.a)) + tf.gather(e_b2, self.a), [0, 2])


        # ------------------ build target_net ------------------
        with tf.variable_scope('target_net'):
            t_w1 = tf.get_variable('t_w1', [self.n_total_features, self.n_hiden_units], initializer=w_initializer)
            t_b1 = tf.get_variable('t_b1', [1, self.n_hiden_units], initializer=b_initializer)

            t_w2 = tf.get_variable('t_w2', [self.n_hiden_units, self.n_total_actions], initializer=w_initializer)
            t_b2 = tf.get_variable('t_b2', [self.n_total_actions, 1], initializer=b_initializer)

            #t_l1 = tf.reduce_sum(tf.gather(e_w1, self.s_), axis = 0) + t_b1
            t_l1 = tf.nn.relu(tf.reduce_sum(tf.gather(t_w1, self.s_), axis = 0) + t_b1)
            self.q_next = tf.reduce_sum(tf.multiply(t_l1, tf.gather(tf.transpose(t_w2), self.a_)) + tf.gather(t_b2, self.a_), [0, 3])
            

        with tf.variable_scope('q_target'):
            q_target = self.r + self.gamma * tf.reduce_max(self.q_next, axis=0, name='Qmax_s_')    # shape=(None, )
            self.q_target = tf.stop_gradient(q_target)
            self.q_target_ = tf.where(tf.greater(self.r, 0), -self.r, self.q_target)
            
        #with tf.variable_scope('q_eval'):
        #    a_indices = tf.stack([tf.range(tf.shape(self.a)[0], dtype=tf.int32), self.a], axis=1)
        #    self.q_eval_wrt_a = tf.gather_nd(params=self.q_eval, indices=a_indices)    # shape=(None, )
            
        with tf.variable_scope('loss'):
            self.loss = tf.reduce_mean(tf.squared_difference(self.q_target_, self.q_eval, name='TD_error'))
            
        with tf.variable_scope('train'):
            # AdamOptimizer
            # RMSPropOptimizer
            # GradientDescentOptimizer
            # MomentumOptimizer,momentum
            # GradientDescentOptimizer
            #self._train_op = tf.train.MomentumOptimizer(self.lr, momentum = 0.9).minimize(self.loss)
            self._train_op = tf.train.RMSPropOptimizer(self.lr).minimize(self.loss)

    def store_transition(self, raw_s, raw_a, r, raw_s_):
        if not hasattr(self, 'memory_counter'):
            self.memory_counter = 0
        self.current_transition_batch = np.hstack((raw_s, raw_a, r, raw_s_))
        # replace the old memory with new memory
        index = self.memory_counter % self.memory_batch_size
        #if index == 0:
        #    print('A new turn from index = 0, %d/%d/%d'%(self.memory_counter, self.memory_batch_size, index))
        self.memory[index*self.exploration_size:(index+1)*self.exploration_size, :] = self.current_transition_batch
        self.memory_counter += 1


    def learn(self):
        # check to replace target parameters
        if self.learn_step_counter % self.replace_target_iter == 0:
            self.sess.run(self.target_replace_op)
            # print('\ntarget_params_replaced\n')
        
        # sample batch memory from all memory
        for i in range(10):
            if self.memory_counter > self.memory_batch_size:
                sample_index = np.random.choice(self.memory_size, size=self.batch_size)
            else:
                sample_index = np.random.choice(self.memory_counter*self.exploration_size, size=self.batch_size)
                
            batch_memory = self.memory[sample_index, :]
            state_coding, action_coding, r, state_coding_, action_coding_ = self.tile_encoding_transitions(batch_memory)
            _, cost = self.sess.run(
                [self._train_op, self.loss],
                feed_dict={
                    self.s: state_coding,
                    self.a: action_coding,
                    self.r: r,
                    self.s_: state_coding_,
                    self.a_: action_coding_,
                })
            self.cost_his.append(cost)

        # increasing epsilon
        self.epsilon = self.epsilon - self.epsilon_decrement if self.epsilon > self.epsilon_min else self.epsilon_min
        self.epsilon2 = self.epsilon2 - self.epsilon2_decrement if self.epsilon2 > self.epsilon2_min else self.epsilon2_min
        self.learn_step_counter += 1

        
    def tile_encoding_transitions(self, batch_memory):
        # 将保存的transition 转化为 tile-coding 编码
        batch_memory = np.transpose(batch_memory)
        
        state = batch_memory[:self.n_states, :]
        action = batch_memory[self.n_states:self.n_states+self.n_action_dim, :].astype(int)
        r = batch_memory[self.n_states+self.n_action_dim, :].reshape([1, self.batch_size])
        state_ = batch_memory[-self.n_states:, :]
        
        # 状态编码，下一步状态编码，action 编码
        state_coding = self.state_coding_transform(state)
        state_coding_ = self.state_coding_transform(state_)
        
        # 转换为以 0 为起点的索引号
        action += self.nNegNotch
        action_coding = self.ActionCoding_Set[action[0, :], action[1, :], :].transpose()    # 转置一下 
        action_coding_ = self.NextActionCoding_Set[action[0, :], action[1, :], :, :].transpose([1, 2, 0])   # 转置一下 
        
        return state_coding, action_coding, r, state_coding_, action_coding_
    
    def choose_action(self, observation, c_action):
        # to have batch dimension when feed into tf placeholder
        # observation = observation[np.newaxis, :]
        observation_coding = self.state_coding_transform(observation)
        action_set = np.zeros(c_action.shape, dtype = int)
        
        c_action = c_action + self.nNegNotch    # 转换为以 0 为起点的索引号 
        for i in range(c_action.shape[1]):         # 为了执行 随机筛选， 这里一个一个输入
            if np.random.uniform() > self.epsilon:
                # forward feed the observation and get q value for every actions
                actions_value = self.sess.run(self.q_eval, feed_dict ={self.s: observation_coding[:, i].reshape([self.n_features, 1]),
                                                                       self.a: self.NextActionCoding_Set[c_action[0, i], c_action[1, i], :, :]})
                action_id = np.argmax(actions_value)
            else:
                action_id = np.random.randint(0, self.n_p_actions)
            action_set[:, i] = self.NextAction_Set[c_action[0, i], c_action[1, i], :, action_id]
            
        return action_set
        

    # 获得当前可选 action 集合
    def get_possible_notch_set(self, notch):
        notch_set = np.zeros([self.n_action_dim, self.n_p_actions], dtype = int)
        for i in range(self.n_action_dim):
            a = self.a_101 + notch[i]
            a[a < self.rangA[0]] = self.rangA[0]
            a[a > self.rangA[1]] = self.rangA[1]
            notch_set[i] = a[self.G[:, i]]
            
        return notch_set

    def initialize_action_dicts(self):
        # 创建一个与action相关的字典，记录每个可能action，以及对应的下一步action和编码组合
        self.ActionCoding_Set = np.zeros([self.nNotch, self.nNotch, self.n_actions])  # 当前 action 对应编码 
        self.NextAction_Set = np.zeros([self.nNotch, self.nNotch, self.n_action_dim, self.n_p_actions]) # 一步可选action集合 
        self.NextActionCoding_Set = np.zeros([self.nNotch, self.nNotch, self.n_actions, self.n_p_actions]) # 一步可选 action 对应编码
        
        for i in range(self.ActionSet.shape[1]):
            notch = self.ActionSet[:, i]
            notch_ = notch + self.nNegNotch   # 转换为以 0 为起点的索引号 
            self.ActionCoding_Set[notch_[0], notch_[1], :] = self.action_coding_transform(notch.reshape([2, 1])).reshape(self.n_actions) 
            self.NextAction_Set[notch_[0], notch_[1], :, :] = self.get_possible_notch_set(notch.reshape([2, 1]))
            self.NextActionCoding_Set[notch_[0], notch_[1], :, :] = self.action_coding_transform(self.NextAction_Set[notch_[0], notch_[1], :, :])
        
        
    def initialize_tile_coding(self, rangS, rangV, detaS, detaV, nTiling, rangA, detaA):
        # ------------------ state coding 
        self.nTiling = nTiling
        self.rangS = rangS
        self.rangV = rangV    # unit: km/h
        self.rangA = rangA
        self.detaS = detaS
        self.detaV = detaV    # unit: km/h
        self.detaA = detaA
        tile_rand = np.tile(np.linspace(0, 1, self.nTiling), (2, 1))
        tile_rand += (np.random.rand(2, self.nTiling)*2 - 1)/2/self.nTiling
        tile_rand[:, 0] = np.array([0, 0])
        tile_rand = np.transpose(tile_rand)
        # np.random.shuffle(tile_rand)
        
        self.tile_rand0 = tile_rand[:, 0].reshape([self.nTiling, 1])
        self.tile_rand1 = tile_rand[:, 1].reshape([self.nTiling, 1])
        
        self.tilingIndex = np.arange(self.nTiling, dtype = int).reshape([self.nTiling, 1])
        self.tile_rand0 *= detaS
        self.tile_rand1 *= detaV
        
        self.nS = np.ceil((rangS[1]-rangS[0])/detaS + 1).astype(int)
        self.nV = np.ceil((rangV[1]-rangV[0])/detaV + 1).astype(int)
        self.nSV = self.nS*self.nV
        
        # ------------------ action coding 
        self.action_vec = np.arange(rangA[0], rangA[1] + 1, dtype = int)
        self.nNotch = self.action_vec.size
        
        self.nTile = detaA
        self.nnA = np.ceil(self.nNotch/self.detaA + 1).astype(int)
        self.nAG = self.nnA**2
        
        tmp = np.arange(self.nTile, dtype = float) + 0.5
        self.code_offset0 = tmp.reshape([self.nTile, 1])
        self.code_offset1 = tmp[::-1].reshape([self.nTile, 1]) 
        
        self.action_tilingIndex = np.arange(self.nTile, dtype = int).reshape([self.nTile, 1])
        
        # 获取所有 action 集合
        Ax, Ay = np.meshgrid(self.action_vec, self.action_vec)
        n_tmp = Ax.size
        self.ActionSet = np.zeros([2, n_tmp], dtype = int)
        self.ActionSet[0, :] = Ax.reshape(1, n_tmp)
        self.ActionSet[1, :] = Ay.reshape(1, n_tmp)
        self.AllActionCode = self.action_coding_transform(self.ActionSet)
        
        # 编码后, 单个编码的元素个数
        self.n_actions = self.nTile
        self.n_features = nTiling
        
        # 编码后, 总编码特征的元素个数
        self.n_total_features = self.nSV * self.nTiling
        self.n_total_actions = self.nAG * self.nTile
    
    # state 编码 encoding 
    def state_coding_transform(self, state):    # unit: m and km/h 
        s = np.ceil((np.tile(state[0, :], (self.nTiling, 1)) - self.tile_rand0 - self.rangS[0])/self.detaS).astype(np.int)
        v = np.ceil((np.tile(state[1, :], (self.nTiling, 1)) - self.tile_rand1 - self.rangV[0])/self.detaV).astype(np.int)
        ind_sv =  np.tile(self.tilingIndex, (1, state.shape[1]))* self.nSV + s*self.nV + v
        return ind_sv

    # action 编码
    def action_coding_transform(self, Np):
        s = np.floor((np.tile(Np[0, :], (self.nTile, 1)) + self.code_offset0 - self.rangA[0])/self.detaA).astype(np.int)
        v = np.floor((np.tile(Np[1, :], (self.nTile, 1)) + self.code_offset1 - self.rangA[0])/self.detaA).astype(np.int)
        ind_action = np.tile(self.action_tilingIndex, (1, Np.shape[1])) * self.nAG + s*self.nnA + v
        return ind_action
    

    # 坐标转换
    def index_transform(self, notch_set): 
        return (notch_set[0, :] + self.nNegNotch)*self.nNotch + notch_set[1, :]+ self.nNegNotch
    
    def plot_cost(self):
        import matplotlib.pyplot as plt
        plt.plot(np.arange(len(self.cost_his)), self.cost_his)
        plt.ylabel('Cost')
        plt.xlabel('training steps')
        plt.show()


# In[3]:


#if __name__ == '__main__':
#    DQN = DeepQNetwork(3,4, output_graph=True)

