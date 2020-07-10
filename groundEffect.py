def groundEffect(z, radius):
    effect = 1-(8.6*(radius/(4*z))**2)
    return effect