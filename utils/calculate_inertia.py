def compute_J(m_kubek, r_kubek, m_x, l_x, m_y, l_y):
    """
    Oblicza całkowity moment bezwładności systemu 2D gimbala.
    
    Parametry:
        m_kubek: masa kubka [kg]
        r_kubek: promień kubka [m]
        m_x: masa ramienia X [kg]
        l_x: długość ramienia X [m]
        m_y: masa ramienia Y [kg]
        l_y: długość ramienia Y [m]
        
    Zwraca:
        J_total: całkowity moment bezwładności [kg·m^2]
    """
    J_kubek = 0.5 * m_kubek * r_kubek**2
    J_x = (1/3) * m_x * l_x**2
    J_y = (1/3) * m_y * l_y**2

    return J_kubek + J_x + J_y