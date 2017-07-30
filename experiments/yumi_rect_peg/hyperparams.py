# To get started, copy over hyperparams from another experiment.
# Visit rll.berkeley.edu/gps/hyperparams.html for documentation.
""" Hyperparameters for MJC peg insertion trajectory optimization. """
from __future__ import division

from datetime import datetime
import os.path
import numpy as np

from gps import __file__ as gps_filepath
from gps.agent.mjc.agent_mjc import AgentMuJoCo
from gps.algorithm.algorithm_traj_opt import AlgorithmTrajOpt
from gps.algorithm.cost.cost_fk import CostFK
from gps.algorithm.cost.cost_action import CostAction
from gps.algorithm.cost.cost_sum import CostSum
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython
from gps.algorithm.policy.lin_gauss_init import init_lqr
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION
from gps.gui.config import generate_experiment_info

SENSOR_DIMS = {
    JOINT_ANGLES: 7,
    JOINT_VELOCITIES: 7,
    END_EFFECTOR_POINTS: 6,
    END_EFFECTOR_POINT_VELOCITIES: 6,
    ACTION: 7,
}

yumi_GAINS = np.array([3.09, 1.08, 0.393, 0.674, 0.111, 0.152, 0.098])

BASE_DIR = '/'.join(str.split(gps_filepath, '/')[:-2])
EXP_DIR = BASE_DIR + '/../experiments/yumi_rect_peg/'


common = {
    'experiment_name': 'my_experiment' + '_' + \
            datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
    'experiment_dir': EXP_DIR,
    'data_files_dir': EXP_DIR + 'data_files/',
    'target_filename': EXP_DIR + 'target.npz',
    'log_filename': EXP_DIR + 'log.txt',
    'conditions': 1,
}

if not os.path.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])

agent = {
    'type': AgentMuJoCo,
    'filename': '/home/shahbaz/Research/Software/Spyder_ws/gps/mjc_models/yumi_right_rect_peg_mjcf.xml',
#   setting initial pos to zero pos of the robot - shahbaz
#    'x0': np.concatenate([np.array([0.7, -1.57, -0.7, 0.35, 0.7, 0., -1.]),
#                          np.zeros(7)]),
    'x0': np.concatenate([np.array([0.4, -2.2, -0.7, 0.35, 0.7, 0., -1.]),
                          np.zeros(7)]),
    #~ 'x0': np.concatenate([np.array([0.1, 0.1, -1.54, -1.7, 1.54, -0.2, 0]),
                          #~ np.zeros(7)]),
    'dt': 0.05,
    'substeps': 3, # shahbaz
    'conditions': common['conditions'],
    'pos_body_idx': np.array([1]),
#    'pos_body_offset': [[np.array([0, 0.2, 0])], [np.array([0, 0.1, 0])],
#                        [np.array([0, -0.1, 0])], [np.array([0, -0.2, 0])]],
#   changing to only 1 initial condition - shahbaz
    'pos_body_offset': [[np.array([0, 0, 0])]],
    'T': 100,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS,
                      END_EFFECTOR_POINT_VELOCITIES],
    'obs_include': [],
#   changing the camera pos for better view - shahbaz
#    'camera_pos': np.array([0., 0., 2., 0., 0.2, 0.5]),
#    'camera_pos': np.array([0., 0., 3., 0., 0., 0.]),
#    'camera_pos': np.array([5.0, 0., 3., 0., 0., 0.]),
#   'camera_pos': np.array([5.0, 0.5, 3., 0., 0., 0.]),
    'camera_pos': np.array([5.0, 0.5, 3., 0., 0., 0.]),
}

algorithm = {
    'type': AlgorithmTrajOpt,
    'conditions': common['conditions'],
    'iterations': 20,
}

algorithm['init_traj_distr'] = {
    'type': init_lqr,
    'init_gains':  1.0 / yumi_GAINS,
    'init_acc': np.zeros(SENSOR_DIMS[ACTION]),
    'init_var': 1.0,
    'stiffness': 1.0,
    'stiffness_vel': 0.5,
    'dt': agent['dt'],
    'T': agent['T'],
}

torque_cost = {
    'type': CostAction,
    'wu': 5e-5 / yumi_GAINS,
}

fk_cost = {
    'type': CostFK,
#    'target_end_effector': np.array([0.0, 0.3, -0.5, 0.0, 0.3, -0.2]), - shahbaz
#    'target_end_effector': np.array([0.3876 , -0.4418, 0.3985, 1.38, 0.1842, -0.0128]),
    #~ 'target_end_effector': np.array([0.4 , -0.4, 0.4, 0.4, -0.2, 0.4]), - worked really well
    #~ 'target_end_effector': np.array([0.4 ,-0.45, 0.25, 0.4, -0.45, 0.05]),
    'target_end_effector': np.array([0.4 ,-0.45, 0.3, 0.4, -0.45, 0.10]),
    'wp': np.array([1, 1, 1, 1, 1, 1]),
    'l1': 0.1,
    'l2': 10.0,
    'alpha': 1e-5,
}

algorithm['cost'] = {
    'type': CostSum,
    'costs': [torque_cost, fk_cost],
    'weights': [1.0, 1.0],
}

algorithm['dynamics'] = {
    'type': DynamicsLRPrior,
    'regularization': 1e-6,
    'prior': {
        'type': DynamicsPriorGMM,
        'max_clusters': 20,
        'min_samples_per_cluster': 40,
        'max_samples': 20,
    },
}

algorithm['traj_opt'] = {
    'type': TrajOptLQRPython,
}

algorithm['policy_opt'] = {}

config = {
    'iterations': algorithm['iterations'],
    'num_samples': 5,
    'verbose_trials': 1,
    'common': common,
    'agent': agent,
    'gui_on': True,
    'algorithm': algorithm,
}

common['info'] = generate_experiment_info(config)
