import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        return np.eye(3)

    def control_input_matrix_B(mu, delta_t):
        theta=mu[2]
        matriz = np.array([[np.cos(theta)*delta_t, 0],[np.sin(theta)*delta_t, 0], [0, delta_t]])
        return matriz


    return state_transition_matrix_A, control_input_matrix_B

def velocity_motion_model_2():
    def A(delta_t):
        matriz = np.array([
            [1, 0, 0, delta_t, 0, 0],
            [0, 1, 0, 0, delta_t, 0], 
            [0, 0, 1, 0, 0, delta_t],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]])
        return matriz

    def B(mu, dt):
        b=np.zeros((6, 2))
        return b

    return A, B
