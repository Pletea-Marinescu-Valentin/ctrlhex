import numpy as np
from pyulog import ULog


def load(path):
    u = ULog(path)
    return u, {d.name: d.data for d in u.data_list if d.multi_id == 0}


def euler(q0, q1, q2, q3):
    roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))))
    return roll, pitch, yaw


def attitude(d, topic='vehicle_attitude', field='q'):
    a = d[topic]
    return a['timestamp'] / 1e6, euler(*(a['%s[%d]' % (field, i)] for i in range(4)))
