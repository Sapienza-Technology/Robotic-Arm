The Jacobian matrix is given by

.. math::
    J = \begin{bmatrix}
    \frac{\partial x}{\partial q_1} & \frac{\partial x}{\partial q_2} & \frac{\partial x}{\partial q_3} & \frac{\partial x}{\partial q_4} & \frac{\partial x}{\partial q_5} & \frac{\partial x}{\partial q_6} \\
    \frac{\partial y}{\partial q_1} & \frac{\partial y}{\partial q_2} & \frac{\partial y}{\partial q_3} & \frac{\partial y}{\partial q_4} & \frac{\partial y}{\partial q_5} & \frac{\partial y}{\partial q_6} \\
    \frac{\partial z}{\partial q_1} & \frac{\partial z}{\partial q_2} & \frac{\partial z}{\partial q_3} & \frac{\partial z}{\partial q_4} & \frac{\partial z}{\partial q_5} & \frac{\partial z}{\partial q_6} \\
    \end{bmatrix}

The Jacobian matrix is implemented in the function :func:`jacobian_bis`.

The pseudo-inverse of the Jacobian matrix is given by

.. math::
    J^+ = (J^TJ)^{-1}J^T

The pseudo-inverse of the Jacobian matrix is implemented in the function :func:`jacobian_pinv_bis`.

