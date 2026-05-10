import numpy as np
import sys
import glob
sys.path.append('python_controllers')
from mpc_node import MPCNode
from arc_bridge.lcm_msgs import unitree_a1_state_t

fname = sorted(glob.glob('logs/wbic_log_*.npz'))[-1]
d = np.load(fname)
t=d['t']
diffs=np.abs(d['foot_pos_cmd']-d['foot_pos']).sum(axis=0)
mi = np.where(diffs>0.001)[0][0]
idx = int(np.argmin(np.abs(t - (t[mi] + 1.0))))

msg = unitree_a1_state_t()
msg.rpy = d['rpy'][:, idx].tolist()
msg.position = d['pos'][:, idx].tolist()
msg.omega = d['omega'][:, idx].tolist()
msg.velocity = d['vel'][:, idx].tolist()
msg.p_gc = d['foot_pos'][:, idx].flatten().tolist()
msg.J_gc = np.zeros(12*18).tolist()
msg.qj_pos = np.zeros(12).tolist()
msg.qj_vel = np.zeros(12).tolist()

node = MPCNode()
node.is_initialized = True
node.current_cmd_pos = d['pos_cmd'][:, idx]
node.current_cmd_yaw = d['rpy_cmd'][2, idx]

# Run the single tick of the MPC logic
node.state_msg = msg
class Dummy:
    pass
node.joy_msg = Dummy()
node.joy_msg.left_stick_x = 0
node.joy_msg.left_stick_y = 0
node.joy_msg.right_stick_x = 0
node.time_since_last_mpc = 0.025
class MockLCM:
    def publish(self, topic, data):
        global captured_msg
        captured_msg = data
    def subscribe(self, *args):
        pass
node.lc = MockLCM()

node.run_mpc()

import arc_bridge.lcm_msgs.mpc_plan_t as mpc_plan_t
plan = mpc_plan_t.decode(captured_msg)
f_r = np.array(plan.reaction_force)
print("MPC node output reaction force:")
print("FR:", f_r[0:3].round(2))
print("FL:", f_r[3:6].round(2))
print("RR:", f_r[6:9].round(2))
print("RL:", f_r[9:12].round(2))
