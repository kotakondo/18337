import gym
import sys
import numpy as np
import copy
from gym import spaces
from compression.utils.other import ActionManager, ObservationManager, State, ObstaclesManager
from colorama import init, Fore, Back, Style

class MyEnvironment(gym.Env):
  """
    Custom Environment that follows gym interface
    Provides a nonlinear quadcopter gym environment by
    wrapping the mav_sim simulator. 
  """
  metadata = {'render.modes': ['human']}

  def __init__(self):
    super(MyEnvironment, self).__init__()

    self.verbose = False

    # Load quadcopter simulator.
    # self.mpc_state_size = 8
    # self.mpc_act_size = 3
    # self.act_size = self.mpc_act_size

    self.len_episode = 200     # Time steps [-] # TODO: Andrea: load from params or set from outside. 

    self.am=ActionManager();
    self.om=ObservationManager();
    self.obsm=ObstaclesManager();

    self.action_shape=self.am.getActionShape();
    self.observation_shape=self.om.getObservationShape();

    self.action_space = spaces.Box(low = -1.0, high = 1.0, shape=self.action_shape)
    self.observation_space = spaces.Box(low = -1.0, high = 1.0, shape=self.observation_shape)
    self.w_gterm_pos=np.array([[5], [0.0], [0.0]])
    # self.max_act = 12.0 # Todo: make sure this is the same value as used in the MPC
    # self.max_obs = 10.0 # Todo: make sure this is the same as in the MPC

    self.dt=0.2; #Timestep in seconds
    self.time=0.0;
    
    self.name=Style.BRIGHT+Fore.GREEN+"[Env]"+Style.RESET_ALL
    # print (self.params)

    # print("self.am.getActionShape()= ", self.am.getActionShape())


    self.reset()

  def __del__(self):
    # self.eng.quit()
    pass

  def printwithName(self,data):
    print(self.name+data)

  def seed(self, seed=None):
    """Set seed function in this environment and calls
    the openAi gym seed function"""
    np.random.seed(seed)
    super().seed(seed)

  def get_len_ep(self):
    return self.len_episode
  
  def set_len_ep(self, len_ep):
    assert len_ep > 0, "Episode len > 0!"
    self.len_episode = len_ep
    # self.printwithName(f"Ep. len updated to {self.len_episode } [steps].")
    self.reset()

  def step(self, action_normalized):
    action_normalized=action_normalized.reshape(self.action_shape) 
    assert not np.isnan(np.sum(action_normalized)), "Received invalid command! u contains nan"

    # self.printwithName(f"Received actionN={action}")
    action= self.am.denormalizeAction(action_normalized);

    # print ("self.action_shape= ",self.action_shape)

    # self.printwithName(f"Received action size={action.shape}")

    assert action.shape==self.action_shape, f"[Env] ERROR: action.shape={action.shape} but self.action_shape={self.action_shape}"

      
    self.printwithName(f"Timestep={self.timestep}")
    ####################################

    w_posBS, w_yawBS= self.am.f_actionAnd_w_State2wBS(action, self.w_state)

    #Update state
    self.w_state= State(w_posBS.getPosT(self.dt), w_posBS.getVelT(self.dt), w_posBS.getAccelT(self.dt), \
                        w_yawBS.getPosT(self.dt), w_yawBS.getVelT(self.dt));
    #Debugging
    # print("w_state.accel= ",self.w_state.w_accel)
    # p0=np.array([[0],[0],[0]]);
    # v0=np.array([[0],[0],[0]]);
    # a0=np.array([[0],[0],[0]]);
    # y0=np.array([[0]]);
    # y_dot0=np.array([[0]]);
    # my_state_zero=State(p0, v0, a0, y0, y_dot0)
    # f_posBS, f_yawBS= self.am.f_actionAnd_w_State2wBS(action, my_state_zero)
    # print("f_state.accel= ", f_posBS.getAccelT(self.dt))
    #####


    #Update time
    self.time = self.time + self.dt;


    ##### Construct observation
    w_obstacles=self.obsm.getFutureWPosObstacles(self.time)

    f_observationn=self.om.getNormalized_fObservationFromTime_w_stateAnd_w_gtermAnd_w_obstacles(self.time, self.w_state, self.w_gterm_pos, w_obstacles);
    ####################################

    # reward=0.0


    info = {}
    self.timestep = self.timestep + 1


    dist2goal=np.linalg.norm(self.w_state.w_pos-self.w_gterm_pos)

    self.printwithName(f"dist2goal={dist2goal}")
    # self.printwithName(f"w_state.w_pos={self.w_state.w_pos.T}")

    if ( (self.timestep >= self.len_episode) or (dist2goal<0.5) ):
      done = True
      info["constraint_violations"] = False
    # elif violation_bfr or violation_after: #or u_violation
    #   done = True
    #   info["constraint_violation"] = True
      #print(f"[Env] Terminated due to constraint violation: obs: {self.x}, act: {u}, steps: {self.timestep}")
    else:
      done = False
    
    # f_obs = self.om.getRandomNormalizedObservation()

    # reward=-np.linalg.norm(action_normalized-self.am.getDummyOptimalNormalizedAction())
    reward=0.0


    # self.printwithName(f"returning reward={reward}")
    # self.printwithName(f"returning obsN={observation}")
    # self.printwithName(f"returning obs size={observation.shape}")
    return f_observationn, reward, done, info

  def reset(self):
    self.printwithName("Resetting namespace")

    self.time=0.0
    self.timestep = 0
    self.w_state=State(np.zeros((3,1)), np.zeros((3,1)), np.zeros((3,1)), np.zeros((1,1)), np.zeros((1,1)))
    self.obsm.newRandomPos();
    # observation = self.om.getRandomNormalizedObservation()
    w_obstacles=self.obsm.getFutureWPosObstacles(self.time)
    f_observationn=self.om.getNormalized_fObservationFromTime_w_stateAnd_w_gtermAnd_w_obstacles(self.time, self.w_state, self.w_gterm_pos, w_obstacles);
    
    # assert observation.shape == self.observation_shape
    # self.printwithName(f"returning obs={observation}")
    return f_observationn

 
  def render(self, mode='human'):
    raise NotImplementedError()
    return
  
  def close (self):
    raise NotImplementedError()
    return
