import numpy as np

g = 9.81 # m/s^2
d_water = 1000 # density water

def calculate_buoyancy(V, density_fluid):
    '''
    Problem #1
    buoyancy force = density fluid * volume * gravitational accel constant
    Given volume and density of fluid, will return the buoyancy force in Newtons.
    '''
    if V <= 0 or density_fluid <= 0:
        raise ValueError("Cannot calculate buoyancy of an object with no volume or density.")
    return g * V * density_fluid

def will_it_float(V, mass):
    '''
    Problem #2
    Given volume and mass, will return true if the object can float or false if it sinks.
    '''
    if V <= 0 or mass <= 0:
        raise ValueError("Cannot calculate buoyancy of an object with no volume or mass.")
    den = mass / V
    if den == d_water:
        return None
    return den < d_water

def calculate_pressure(depth):
    '''
    Problem #3
    pressure = density fluid * gravitational accel constant * depth
    Given depth (negative or positive, it runs through abs), calculates the pressure in pascals and returns it.
    '''
    pressure = 101.3
    if depth == 0:
        return pressure
    elif depth < 0:
        depth = abs(depth)
    return pressure + d_water * g * depth

def calculate_acceleration(F, m):
    '''
    Problem #4
    Provided a force in newtons and a mass in kg, this function will compute acceleration.
    It raises a value error in the case that mass is less than or equal to 0.
    '''
    if m <= 0:
        raise ValueError("Cannot calculate acceleration of an object with no mass.")
    return F / m

def calculate_angular_acceleration(tau, I):
    '''
    Problem #5
    Provided a torque in newton-meters and a moment of inertia in m^2kg, this function will compute angular acceleration.
    It raises a value error in the case that the moment of inertia is less than or equal to 0.
    ''' 
    if I <= 0:
        raise ValueError("Cannot calculate angular acceleration of an object with no inertia.")
    return tau / I

def calculate_torque(F_magnitude, F_direction, r):
    '''
    Problem #6
    Given the force magnitude in newtons, the force direction in radians, and the radius in meters,
    this function will return torque in newton-meters.
    ''' 
    F_magnitude = np.abs(F_magnitude)
    return r * F_magnitude * np.sin(F_direction)

def calculate_moment_of_inertia(m, r):
    '''
    Problem #7
    Given the mass in kg and the radius in meters,
    this function will return I, the moment of inertia in m^2kg.
    '''
    return m * r * r

'''
(needed for problems 8 and 9)
supplemental methods that calculate rotation matrix and rov rotation matrix given respective angles
'''
def get_rotation_matrix(theta):
    rot_mat = np.array(
        [
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ]
    )
    return rot_mat

def get_rov_rotation_matrix(a):
    rov_rot_mat = np.array(
        [
            [np.cos(a), np.cos(a), -np.cos(a), -np.cos(a)],
            [np.sin(a), -np.sin(a), -np.sin(a), np.sin(a)]
        ]
    )
    return rov_rot_mat

def calculate_auv_acceleration(F_magnitude, F_angle, mass = 100):
    '''
    Problem #8

    a)
    Given:
    F_magnitude: the magnitude of force applied by the thruster in Newtons.
    F_angle: the angle of the force applied by the thruster in radians. The angle is measured from the x axis. Positive angles are measured in the counter-clockwise direction.
    mass (optional): the mass of the AUV in kilograms. The default value is 100kg. 
    volume (optional): the volume of the AUV in cubic meters. The default value is 0.1m^3
    thruster_distance(optional): the distance from the center of mass of the AUV to the thruster in meters. The default value is 0.5m.
    The function returns the acceleration of the AUV in meters per second squared.
    '''
    if mass <= 0:
        raise ValueError("Mass cannot be negative or zero.")
    rov_rot_mat = get_rov_rotation_matrix(F_angle) # alpha
    thrusters = np.array([[F_magnitude], [F_magnitude], [-F_magnitude], [-F_magnitude]])
    force_matrix = np.matmul(rov_rot_mat, thrusters)
    a = force_matrix / mass
    return a

def calculate_auv_angular_acceleration(F_magnitude, F_angle, inertia = 1, thruster_distance = 0.5):
    '''
    Problem #8

    b)
    Given:
    F_magnitude: the magnitude of force applied by the thruster in Newtons.
    F_angle: the angle of the force applied by the thruster in radians. The angle is measured from the x axis. Positive angles are measured in the counter-clockwise direction.
    inertia (optional): the moment of inertia of the AUV in kgm^2. The default value is 1 kgm^2.
    thruster_distance(optional): the distance from the center of mass of the AUV to the thruster in meters. The default value is 0.5m.
    The function returns the angular acceleration of the AUV in radians per second squared.
    '''
    if inertia <= 0:
        raise ValueError("Inertia cannot be negative or zero.")
    torque = calculate_torque(F_magnitude, F_angle, thruster_distance)
    a = torque / inertia
    return a

def calculate_auv2_acceleration(T, alpha, theta, mass = 100):
    '''
    Problem #9
    a)
    Given: 
    T = np.ndarray of magnitudes of force vectors.
    alpha = angle of thrusters in radians.
    theta = angle AUV in radians.
    mass (optional) = mass of AUV in kg. default 100 kg.
    '''
    if mass <= 0:
        raise ValueError("Mass cannot be negative or zero.")
    rov_rot_mat = get_rov_rotation_matrix(alpha)
    rot_mat = get_rov_rotation_matrix(theta)
    force_matrix = np.matmul(rot_mat, rov_rot_mat, T)
    a = force_matrix / mass
    return a

def calculate_auv2_angular_acceleration(T, alpha, L, l, inertia = 100):
    '''
    Problem #9
    b)
    Given: 
    T = np.ndarray of magnitudes of force vectors
    alpha = angle of thrusters in radians
    L = the distance from the center of mass of the AUV to the thrusters in meters.
    l = the distance from the center of mass of the AUV to the thrusters in meters.
    inertia (optional) = moment of inertia of AUV in kgm^2. default 100 kgm^2.
    ''' 
    forces = np.matmul(get_rov_rotation_matrix(alpha), T)
    thruster_distance = np.sqrt(L * L + l * l) # pythag
    a = np.array([forces[0] / inertia, forces[1]/ inertia])
    F_magnitude = np.sqrt(a[0] * 1[0] + a[1] * a[1])
    torque = calculate_torque(F_magnitude, alpha, thruster_distance)
    if inertia <= 0:
        raise ValueError("Inertia cannot be negative or zero.")
    a = torque / inertia
    return a