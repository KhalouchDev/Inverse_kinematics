import numpy as np

topLength = 80  # a
bottomLength = 80  # b
height = 120  # c


def z_kinematics(topLen, bottomLen, z):
    # Calculating the leg height is simple using the cosine rule
    # Two sides of the triangle are already there and constant (topLen, bottomLen)
    # The only variable is the height (z), according to the input the angles of the shoulder and knee will be calculated
    B_rad = (z ** 2 + topLen ** 2 - bottomLen ** 2) / (2 * z * topLen)
    B_rad = np.arccos(B_rad)
    B_angle = np.round(np.rad2deg(B_rad), 0)

    C_angle = 180 - (B_angle * 2)

    return B_angle, C_angle


def x_kinematics(z, x):
    # Calculating the x kinematics will be accomplished by using pythagorean's theorem
    # z will be the first side of the right angle triangle, x will be the base of the right angle triangle
    # The hypotenuse will be the new leg length (z)
    # Then we can calculate the angle of the shoulder
    shoulder_rad = np.arctan(x / z)
    shoulder_angle = np.rad2deg(np.round(shoulder_rad), 0)

    z2 = z / np.cos(shoulder_angle)

    return shoulder_angle, z2


def inverse_kinematics(topLen, bottomLen, x, z):
    # Calculate angle to add to shoulder
    shoulder_rad = np.arctan(x / z)
    shoulder_angle = np.round(np.rad2deg(shoulder_rad), 0)

    # Calculate new z (hypotenuse)
    z2 = z / np.cos(shoulder_angle)

    # Calculate the angle of shoulder and knee
    B_rad = (z2 ** 2 + topLen ** 2 - bottomLen ** 2) / (2 * z2 * topLen)
    B_rad = np.arccos(B_rad)
    B_angle = np.round(np.rad2deg(B_rad), 0) + shoulder_angle # shoulder

    C_rad = (topLen ** 2 + bottomLen ** 2 - z2 ** 2) / (2 * bottomLen * topLen)
    C_rad = np.arccos(C_rad)
    C_angle = np.round(np.rad2deg(C_rad), 0)  # knee

    return B_angle, C_angle


shoulderAng, kneeAng = inverse_kinematics(topLength, bottomLength, 20, height)
print(f"Shoulder angle: {shoulderAng}, Knee angle: {kneeAng}")
