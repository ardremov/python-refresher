# buoyancy force = density fluid * volume * gravitational accel constant

g = 9.81 # m/s^2
d_water = 1000 # density water

def calculate_buoyancy(V, density_fluid):
    if V <= 0 or density_fluid <= 0:
        raise ValueError("Cannot calculate buoyancy of an object with no volume or density.")
    return g * V * density_fluid

def will_it_float(V, mass):
    # return true if float, false if sink
    if V <= 0 or mass <= 0:
        raise ValueError("Cannot calculate buoyancy of an object with no volume or mass.")
    elif (den == d_water):
        return None
    den = mass / V
    return den < d_water

# pressure = density fluid * gravitational accel constant * depth
def calculate_pressure(depth):
    if depth == 0:
        return 101.3
    elif depth < 0:
        depth = abs(depth)
    return d_water * g * depth
print(will_it_float(1, 10000))