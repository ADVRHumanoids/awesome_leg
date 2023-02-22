#!/usr/bin/env python3

from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as opt

import rospkg

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.colors import LightSource, Normalize
from mpl_toolkits.axes_grid1 import make_axes_locatable

def impact_sev_ratio(q1, q2):
  
  lambda_inv = np.zeros((6, 6))
  q_model = np.zeros(n_jnts_model)
  q_model[1] = q1
  q_model[2] = q2
  model.setJointPosition(q_model)
  model.update()
  J = model.getJacobian("tip1")
  M_inv = np.linalg.inv(model.getInertiaMatrix())
  lambda_inv = J @ M_inv @ J.T

  impact_sev_ratio = (1 + kr)/lambda_inv[2, 2]

  return impact_sev_ratio

rospackage = rospkg.RosPack() # Only for taking the path to the leg package
awesome_path = rospackage.get_path("awesome_leg")

urdf_path = awesome_path + "/" + "description/urdf/generated/awesome_leg.urdf"
srdf_path = awesome_path + "/" + "description/srdf/awesome_leg_test_rig.srdf"

urdf = open(urdf_path, "r").read() # read the URDF
srdf = open(srdf_path, "r").read() 

cfg = opt.ConfigOptions()
cfg.set_urdf(urdf)
cfg.set_srdf(srdf)
cfg.generate_jidmap()
cfg.set_bool_parameter('is_model_floating_base', False)
cfg.set_string_parameter('model_type', 'RBDL')
cfg.set_string_parameter('framework', 'ROS')

model = xbot.ModelInterface(cfg)
limits = model.getJointLimits()
ubs = limits[1][1:]
lbs = limits[0][1:]
n_discr = 100
n_jnts_model = model.getJointNum()

# range_q1 = np.linspace(lbs[0], ubs[0], n_discr)
# range_q2 = np.linspace(lbs[1], ubs[1], n_discr)

# range_q1 = np.linspace(0.58, 1.2, n_discr)
# range_q2 = np.linspace(0.58, 1.2, n_discr)

range_q1 = np.linspace(0, ubs[0], n_discr)
range_q2 = np.linspace(0, ubs[1], n_discr)

# range_q1 = np.array([0.0, 1.5708])
# range_q2 = np.array([0.0, 0.0])

X, Y = np.meshgrid(range_q1, range_q2)

kr = 0.0
impact_severity_ratio = np.zeros((n_discr, n_discr))

for i in range(range_q1.size):

  for j in range(range_q2.size):

    impact_severity_ratio[i, j] = impact_sev_ratio(range_q1[i], range_q2[j])

extent = (range_q1[0], range_q1[range_q1.size-1], range_q1[0], range_q1[range_q1.size-1])
fig, ax = plt.subplots()
ls = LightSource(315, 45)
ax.set_title("Impact severity ratio")
p = ax.imshow(impact_severity_ratio, cmap = 'viridis', extent=extent, vmin=np.min(impact_severity_ratio), vmax=np.max(impact_severity_ratio))
divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="5%", pad=0.05)
cbar = fig.colorbar(p, cax=cax, shrink = 0.5, aspect = 20, label='impact strenght')
ax.set_xlabel('q1') 
ax.set_ylabel('q2') 

q_home_refq1 = np.array([1.5708, 1.58, 2.10, 1.73, 1.25, 1.01, 0.58, 2.16, 2.3562, 0.0])
q_home_refq2 = np.array([0.0, 0.58, 1.3, 2.4, 0.58, 1.54, 0.96, 2.02, 2.3562, 0.0])
impact_meas_ref = np.zeros(q_home_refq1.size)
for i in range(q_home_refq1.size):
  impact_meas_ref[i] = impact_sev_ratio(q_home_refq1[i], q_home_refq2[i])
  group_label = ["homing_min", "homing0", "homing1", "homing2", "homing3", "homing4", "homing5", "homing6", "homing7", "homing_max"]
cdict = {1: 'red', 2: 'blue', 3: 'green'}
cmap = plt.cm.get_cmap('hsv', len(group_label))
i = 0
for i in range(q_home_refq1.size):
  
    ax.scatter(q_home_refq1[i], q_home_refq2[i], c = [cmap(i)], label = group_label[i], s = 50, marker = 'x')

ax.legend()


fig3d, ax3d = plt.subplots(subplot_kw={"projection": "3d"})
ax3d.plot_surface(X, Y, impact_severity_ratio)
ax3d.scatter(q_home_refq1, q_home_refq2, impact_meas_ref, s = 50, marker = 'x', c='white')
# plt.title('Impact severity ratio')
# p = plt.imshow(impact_severity_ratio)
# plt.colorbar(p)

# fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
# surf = ax.plot_surface(q1, q2, impact_severity_ratio, cmap=cm.coolwarm,
#                        linewidth=0, antialiased=False)

fig2d, ax2d = plt.subplots()
ax2d.scatter(range(len(impact_meas_ref)), impact_meas_ref, marker = 'x')
ax2d.set_xlabel('configuration n.')
ax2d.set_ylabel('impact severity ratio')
ax2d.grid()

plt.show()
