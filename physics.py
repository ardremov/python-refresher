g = 9.81 # m/s^2
d_water = 1000 # density water

'''
buoyancy force = density fluid * volume * gravitational accel constant
Given volume and density of fluid, will return the buoyancy force in Newtons.
'''
def calculate_buoyancy(V, density_fluid):
    if V <= 0 or density_fluid <= 0:
        raise ValueError("Cannot calculate buoyancy of an object with no volume or density.")
    return g * V * density_fluid

'''Given volume and mass, will return true if the object can float or false if it sinks.'''
def will_it_float(V, mass):
    if V <= 0 or mass <= 0:
        raise ValueError("Cannot calculate buoyancy of an object with no volume or mass.")
    den = mass / V
    if den == d_water:
        return None
    return den < d_water

'''
pressure = density fluid * gravitational accel constant * depth
Given depth (negative or positive, it runs through abs), calculates the pressure in pascals and returns it.
'''
def calculate_pressure(depth):
    pressure = 101.3
    if depth == 0:
        return pressure
    elif depth < 0:
        depth = abs(depth)
    return pressure + d_water * g * depth