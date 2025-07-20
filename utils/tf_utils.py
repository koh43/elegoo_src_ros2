import numpy as np
np.seterr(all="ignore")
from scipy.spatial.transform import Rotation as R

def R2quat(rot_mtx):
    r = R.from_matrix(rot_mtx)
    return r.as_quat()

