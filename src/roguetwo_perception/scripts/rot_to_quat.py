from tf.transformations import quaternion_from_matrix

import numpy as np

rot = np.array([[-0.0261938, -0.99948997, -0.0182671, 0.00431358],
                [0.04802614, 0.01699406, -0.9987015, 0.04431263],
                [0.99850257, -0.02703709,  0.0475565, 0.00519636],
                [0, 0, 0, 1]])

quaternion = quaternion_from_matrix(rot)
print (quaternion)


