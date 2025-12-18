import numpy as np

from stonesoup.models.transition import TransitionModel

def quat_mul(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_from_omega(omega, dt): 
    ang = np.linalg.norm(omega) * dt
    if ang < 1e-12: 
        return np.array([1, 0, 0, 0], dtype=float)

    axis = omega/np.linalg.norm(omega)
    return np.array([np.cos(0.5*ang), *(np.sin(0.5*ang)*axis)], dtype=float)

class QuatBiasTransition(TransitionModel): 
    """
        State x = [qw, qx, qy, qz, bgx, bgy, bgz]
    """

    ndim = 7

    def __init__(self, q_process=1e-6, b_process=1e-6): 
        super().__init__()
        self.q_proc = q_process
        self.b_proc = b_process

    @property
    def ndim_state(self): 
        return self.ndim

    def function(self, state_vector, noise=False, **kwargs):
        dt =  kwargs.get('time_interval', 0.01)
        u = kwargs.get('u', np.zeros(3))
        x = state_vector.reshape(-1)
        q = x[0:4]; q = q/np.linalg.norm(q)
        b = x[4:7]
        omega = u - b
        dq = quat_from_omega(omega, dt)
        q_new = quat_mul(q, dq)
        q_new = q_new/np.linalg.norm(q_new)
        x_new = np.hstack([q_new, b])

        return x_new.reshape(-1, 1)

    def jacobian(self, state_vector, **kwargs): 
        f0 = self.function(state_vector, **kwargs).reshape(-1)
        eps = 1e-6
        J = np.zeros((7, 7))

        for i in range(7): 
            dx = np.zeros(7); dx[i] = eps
            f1 = self.function((state_vector.reshape(-1)+dx).reshape(-1,1), **kwargs).reshape(-1)
            J[:, i] = (f1 - f0)/eps

        return J

    @property
    def covar(self): 
        # Process noise for [quat, bias]: small on quat, small random walk on bias
        Qq = self.q_proc*np.eye(4)
        Qb = self.b_proc*np.eye(3)
        return np.block([
            [Qq, np.zeros((4, 3))], 
            [np.zeros((3, 4)), Qb]
        ])

    
