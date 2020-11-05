""" This file defines an agent for the PR2 ROS environment. """
import copy
import time
import numpy as np
# import sys
# rospy_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
# sys.path.append(rospy_path)
import rospy
# del sys.path[-1]
import copy
from gps.agent.agent import Agent
from gps.agent.agent_utils import generate_noise, setup
from gps.agent.config import AGENT_ROS
from gps.agent.ros.ros_utils import ServiceEmulator, msg_to_sample, \
        policy_to_msg, tf_policy_to_action_msg, tf_obs_msg_to_numpy
from gps.proto.gps_pb2 import TRIAL_ARM, AUXILIARY_ARM
from gps_agent_pkg.msg import TrialCommand, SampleResult, PositionCommand, \
        RelaxCommand, DataRequest, TfActionCommand, TfObsData
from yumikin.YumiKinematics import YumiKinematics
try:
    from gps.algorithm.policy.tf_policy import TfPolicy
except ImportError:  # user does not have tf installed.
    TfPolicy = None
from normflow_policy.rewards import process_cart_path_rwd
from garage._dtypes import EpisodeBatch

class AgentROS(Agent):
    """
    All communication between the algorithms and ROS is done through
    this class.
    """
    def __init__(self, hyperparams, policy, env_spec, init_node=True):
        """
        Initialize agent.
        Args:
            hyperparams: Dictionary of hyperparameters.
            init_node: Whether or not to initialize a new ROS node.
        """
        config = copy.deepcopy(AGENT_ROS)
        config.update(hyperparams)
        Agent.__init__(self, config)
        if init_node:
            rospy.init_node('gps_agent_ros_node')
        self._init_pubs_and_subs()
        self._seq_id = 0  # Used for setting seq in ROS commands.
        self.policy = policy
        self.env_spec = env_spec
        conditions = self._hyperparams['conditions']

        self.x0 = []
        for field in ('x0', 'ee_points_tgt', 'reset_conditions'):
            self._hyperparams[field] = setup(self._hyperparams[field],
                                             conditions)
        self.x0 = self._hyperparams['x0']
        q0 = self.x0[0][:7]
        r = rospy.Rate(1)
        r.sleep()
        self.kin = YumiKinematics(self._hyperparams['kin_params'])
        self.M_tra = np.diag(self.kin.get_cart_intertia_d(q0))[:3]
        self.K_tra = self._hyperparams['K_tra']
        self.D_tra = np.max(np.sqrt(np.multiply(self.M_tra, self.K_tra))) * np.eye(3)
        self.M_rot = np.diag(self.kin.get_cart_intertia_d(q0))[3:]
        self.K_rot = self._hyperparams['K_rot']
        self.D_rot = np.max(np.sqrt(np.multiply(self.M_rot, self.K_rot))) * np.eye(3)
        self.J_Ad_curr = None
        self.use_tf = False
        self.observations_stale = True

    def _init_pubs_and_subs(self):
        self._trial_service = ServiceEmulator(
            self._hyperparams['trial_command_topic'], TrialCommand,
            self._hyperparams['sample_result_topic'], SampleResult
        )
        self._reset_service = ServiceEmulator(
            self._hyperparams['reset_command_topic'], PositionCommand,
            self._hyperparams['sample_result_topic'], SampleResult
        )
        self._relax_service = ServiceEmulator(
            self._hyperparams['relax_command_topic'], RelaxCommand,
            self._hyperparams['sample_result_topic'], SampleResult
        )
        self._data_service = ServiceEmulator(
            self._hyperparams['data_request_topic'], DataRequest,
            self._hyperparams['sample_result_topic'], SampleResult
        )

    def _get_next_seq_id(self):
        self._seq_id = (self._seq_id + 1) % (2 ** 32)
        return self._seq_id

    def get_data(self, arm=TRIAL_ARM):
        """
        Request for the most recent value for data/sensor readings.
        Returns entire sample report (all available data) in sample.
        Args:
            arm: TRIAL_ARM or AUXILIARY_ARM.
        """
        request = DataRequest()
        request.id = self._get_next_seq_id()
        request.arm = arm
        request.stamp = rospy.get_rostime()
        result_msg = self._data_service.publish_and_wait(request)
        # TODO - Make IDs match, assert that they match elsewhere here.
        sample = msg_to_sample(result_msg, self)
        return sample

    # TODO - The following could be more general by being relax_actuator
    #        and reset_actuator.
    def relax_arm(self, arm):
        """
        Relax one of the arms of the robot.
        Args:
            arm: Either TRIAL_ARM or AUXILIARY_ARM.
        """
        relax_command = RelaxCommand()
        relax_command.id = self._get_next_seq_id()
        relax_command.stamp = rospy.get_rostime()
        relax_command.arm = arm
        self._relax_service.publish_and_wait(relax_command)

    def reset_arm(self, arm, mode, data):
        """
        Issues a position command to an arm.
        Args:
            arm: Either TRIAL_ARM or AUXILIARY_ARM.
            mode: An integer code (defined in gps_pb2).
            data: An array of floats.
        """
        reset_command = PositionCommand()
        reset_command.mode = mode
        reset_command.data = data
        # print('reset pos:', data)
        reset_command.pd_gains = self._hyperparams['pid_params']
        reset_command.arm = arm
        timeout = self._hyperparams['reset_timeout']
        reset_command.id = self._get_next_seq_id()
        self._reset_service.publish_and_wait(reset_command, timeout=timeout)
        #TODO: Maybe verify that you reset to the correct position.

    def reset(self, condition, policy=None):
        """
        Reset the agent for a particular experiment condition.
        Args:
            condition: An index into hyperparams['reset_conditions'].
        """
        condition_data = self._hyperparams['reset_conditions'][condition]
        self.reset_arm(TRIAL_ARM, condition_data[TRIAL_ARM]['mode'],
                       condition_data[TRIAL_ARM]['data'])
        # self.reset_arm(AUXILIARY_ARM, condition_data[AUXILIARY_ARM]['mode'],
        #                condition_data[AUXILIARY_ARM]['data'])
        # if policy:
        #     policy.reset()
        del self.kin
        self.kin = YumiKinematics(self._hyperparams['kin_params'])
        time.sleep(2.0)  # useful for the real robot, so it stops completely
        print('Reset done')

    def sample(self, policy, condition, verbose=True, save=True, noisy=True):
        """
        Reset and execute a policy and collect a sample.
        Args:
            policy: A Policy object.
            condition: Which condition setup to run.
            verbose: Unused for this agent.
            save: Whether or not to store the trial into the samples.
            noisy: Whether or not to use noise during sampling.
        Returns:
            sample: A Sample object.
        """
        print ('Sample initiated')
        self._init_tf(self.dU)

        self.reset(condition)
        # Generate noise.
        if noisy:
            noise = generate_noise(self.T, self.dU, self._hyperparams)
        else:
            noise = np.zeros((self.T, self.dU))

        # Execute trial.
        trial_command = TrialCommand()
        trial_command.id = self._get_next_seq_id()
        trial_command.controller = policy_to_msg(self, noise)
        trial_command.T = self.T
        trial_command.id = self._get_next_seq_id()
        trial_command.frequency = self._hyperparams['frequency']
        ee_points = self._hyperparams['end_effector_points']
        trial_command.ee_points = ee_points.reshape(ee_points.size).tolist()
        trial_command.ee_points_tgt = \
                self._hyperparams['ee_points_tgt'][condition].tolist()
        trial_command.state_datatypes = self._hyperparams['state_include']
        trial_command.obs_datatypes = self._hyperparams['state_include']

        if self.use_tf is False:
            sample_msg = self._trial_service.publish_and_wait(
                trial_command, timeout=self._hyperparams['trial_timeout']
            )
            sample = msg_to_sample(sample_msg, self)
            if save:
                self._samples[condition].append(sample)
            return sample
        else:
            self._trial_service.publish(trial_command)
            sample = self.run_trial_tf(policy, time_to_run=self._hyperparams['trial_timeout'])
            # sample = msg_to_sample(sample_msg, self)
            # if save:
            #     self._samples[condition].append(sample)
            return sample

    def run_trial_tf(self, policy, time_to_run=5):
        """ Run an async controller from a policy. The async controller receives observations from ROS subscribers
         and then uses them to publish actions."""
        should_stop = False
        consecutive_failures = 0
        start_time = time.time()
        s_time = time.time()
        X = []
        F = []
        U = []
        while should_stop is False:
            if self.observations_stale is False:
                consecutive_failures = 0
                # s_time = time.time()
                last_obs = tf_obs_msg_to_numpy(self._tf_subscriber_msg)
                # s_time = time.time()
                # print('ROS com time:',time.time() - s_time)
                X.append(last_obs)
                s_time = time.time()
                u, f = self._get_new_action(policy, last_obs)
                u[-1] = 0. # todo
                # print('Sample time:', time.time() - s_time)
                # if self.current_action_id==1:
                    # print('Initial torque', u)
                    # print('Initial force', f)
                F.append(f)
                U.append(u)
                action_msg = tf_policy_to_action_msg(self.dU, u, self.current_action_id)
                # s_time = time.time()
                # print('Sample time:',time.time() - s_time)
                self._tf_publish(action_msg)
                self.observations_stale = True
                self.current_action_id += 1
                # print('Current action id',self.current_action_id)
            else:
                # rospy.sleep(0.005)
                rospy.sleep(0.005)
                consecutive_failures += 1
                # if time.time() - start_time <= time_to_run and consecutive_failures > 5 and self.current_action_id>1 and self.current_action_id<=self.T:
                #     print('Obs lost. Filling the missing with last')
                #     print('current_action_id', self.current_action_id)
                #     F.append(copy.copy(F[-1]))
                #     X.append(copy.copy(X[-1]))
                #     U.append(copy.copy(U[-1]))
                #     should_stop = False
                if time.time() - start_time > time_to_run and consecutive_failures > 5:
                    # we only stop when we have run for the trial time and are no longer receiving obs.
                    should_stop = True
                # else:
                #     should_stop = False
        rospy.sleep(0.25)  # wait for finished trial to come in.
        path = {}
        path['observations'] = None
        path['actions'] = None
        path['agent_infos'] = {}
        path['agent_infos']['jt'] = np.array(U)
        path['agent_infos']['ef'] = np.array(F)
        path['agent_infos']['jx'] = np.array(X)
        print('Samle with size', len(X))
        # result = self._trial_service._subscriber_msg
        return path  # the trial has completed. Here is its message.

    def _get_new_action(self, policy, jx):
        # return policy.act(None, obs, None, None)
        # assert (jx.shape[0] == 7 * 2)
        x_d_e, x_dot_d_e, J_Ad = self.kin.get_cart_error_frame_terms(jx[:7], jx[7:])
        ex = np.concatenate((x_d_e, x_dot_d_e))
        f_t, _ = policy.get_action(np.concatenate((ex[:3], ex[6:9])))
        # f_t = -np.matmul(self.K_tra, ex[:3]) - np.matmul(self.D_tra, ex[6:9])
        f_r = -np.matmul(self.K_rot, ex[3:6]) - np.matmul(self.D_rot, ex[9:])
        f = np.concatenate((f_t, f_r))
        u = J_Ad.T.dot(f)
        return u, f

    def _tf_callback(self, message):
        self._tf_subscriber_msg = message
        self.observations_stale = False

    def _tf_publish(self, pub_msg):
        """ Publish a message without waiting for response. """
        self._pub.publish(pub_msg)

    def _init_tf(self, dU):
        self._tf_subscriber_msg = None
        self.observations_stale = True
        self.current_action_id = 1
        self.dU = dU
        if self.use_tf is False:  # init pub and sub if this init has not been called before.
            self._pub = rospy.Publisher('/gps_controller_sent_robot_action_tf', TfActionCommand)
            self._sub = rospy.Subscriber('/gps_obs_tf', TfObsData, self._tf_callback)
            r = rospy.Rate(0.5)  # wait for publisher/subscriber to kick on.
            r.sleep()
        self.use_tf = True
        self.observations_stale = True

    def shutdown_worker(self):
        return None

    def obtain_samples(self,
        itr, batch_size,
        agent_update=None,
        env_update=None):
        S = self._hyperparams['episode_num']
        paths = []
        for s in range(S):
            print('episode',s)
            path = self.sample(self.policy, 0, verbose=True, save=False, noisy=False)
            path = process_cart_path_rwd(path,self.kin)
            paths.append(path)

        return EpisodeBatch.from_list(self.env_spec,paths)


