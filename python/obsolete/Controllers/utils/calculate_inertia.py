def compute_J(m_plate, r_plate, m_x, l_x, m_y, l_y):
    """
    Calculates total moment of inertia for gimbal with rotating plate.
    """
    J_plate = 0.5 * m_plate * r_plate**2
    J_x = (1/3) * m_x * l_x**2
    J_y = (1/3) * m_y * l_y**2

    return J_plate + J_x + J_y