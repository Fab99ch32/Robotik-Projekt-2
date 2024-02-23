import cv2
import numpy as np
import math

camera_matrix = [
    [2.43442033e+03, 0.00000000e+00, 1.32928359e+03],
    [0.00000000e+00, 2.42670731e+03, 9.60008894e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
]

distortion_coefficients = [
    [0.09842787, -0.34590361, -0.00035382, 0.00109482, 0.295309]
]

def calculate_marker_position(corners, marker_size_in_meters):
    marker_points_3d = np.array([
        [-marker_size_in_meters / 2, marker_size_in_meters / 2, 0],
        [marker_size_in_meters / 2, marker_size_in_meters / 2, 0],
        [marker_size_in_meters / 2, -marker_size_in_meters / 2, 0],
        [-marker_size_in_meters / 2, -marker_size_in_meters / 2, 0]
    ], dtype=np.float32)

    # Single corner
    c = np.array(corners[0], dtype=np.float32)

    # Perform pose estimation
    _, rotation_vector, translation_vector = cv2.solvePnP(
        marker_points_3d, c[0], np.array(camera_matrix), np.array(distortion_coefficients), flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    
    x_position, y_position, z_position,= translation_vector
    
    # Convert rotation vector to rotation matrix 
    rmat, _ = cv2.Rodrigues(rotation_vector)
    
    quaternion = cv2.RQDecomp3x3(rmat)[0]

    yaw = np.arctan2(rmat[1, 0], rmat[0, 0])
    
    return float(x_position), float(y_position), float(z_position), float(yaw), rotation_vector, translation_vector


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def rotation_matrix_to_euler(R):
    """
    Convert a 3x3 rotation matrix to Euler angles.

    Parameters:
    - R (numpy.ndarray): 3x3 rotation matrix.

    Returns:
    - numpy.ndarray: Yaw, pitch, and roll angles.
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    
    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    
    return np.array([yaw, pitch, roll])

def rotation_matrix_to_quaternion(R):
    """
    Convert a 3x3 rotation matrix to a unit quaternion.

    Parameters:
    - R (numpy.ndarray): 3x3 rotation matrix.

    Returns:
    - numpy.ndarray: Unit quaternion [x, y, z, w].
    """
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return np.array([x, y, z, w])


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


def detect_aruco_markers(frame):
    """
    Detects Aruco markers in an image frame.

    Args:
        frame (numpy.ndarray): The input image frame.

    Returns:
        Tuple: Lists of corner points and marker IDs.
    """
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

    return corners, ids

def calculate_distance(corners, i, marker_size):
    """
    Calculates the distance from the camera to an Aruco marker.

    Args:
        corners (list): List of corner points of the detected marker.
        i (int): Index of the marker in the corners list.
        marker_size (float): The real-world size of the Aruco marker.

    Returns:
        float: Calculated distance from the camera to the marker.
    """
    # Convert the camera_matrix to a numpy array
    camera_matrix_np = np.array(camera_matrix)

    marker_size_in_pixels = np.linalg.norm(corners[i][0][0] - corners[i][0][1])
    distance = marker_size * camera_matrix_np[0, 0] / marker_size_in_pixels
    return distance

def calculate_rotation_matrix(theta):
    """
    Calculates a 3x3 rotation matrix for a given rotation angle (Theta).

    Args:
        theta (float): Rotation angle in radians.

    Returns:
        numpy.ndarray: 3x3 rotation matrix.
    """
    try:
        rotation_matrix = np.array([
            [np.cos(theta), 0, -np.sin(theta)],
            [0, 1, 0],
            [np.sin(theta), 0, np.cos(theta)]
        ])

        return rotation_matrix

    except Exception as e:
        print(f"Error calculating rotation matrix: {e}")
        return np.array([])

def calculate_camera_position(corners, ids, camera_relative_to_marker):
    """
    Calculates the position of the camera relative to the Aruco marker.

    Args:
        corners (list): List of corner points of the detected marker.
        ids (list): List of marker IDs.
        camera_relative_to_marker (numpy.ndarray): Relative position of the camera to the marker.

    Returns:
        Tuple: X, Y, and Theta (rotation around the Y-axis) of the camera.
    """
    if ids is not None and len(ids) > 0:
        x, y, z, theta, r, t = calculate_marker_position(corners, 0.1)

        if x is not None and y is not None and z is not None and theta is not None:
            marker_position = np.array([x, y, z])
            rotation_matrix = calculate_rotation_matrix(theta)

            if rotation_matrix is not None:
                camera_position = marker_position + rotation_matrix.dot(camera_relative_to_marker)

                # Extracting X, Y, and Theta from the camera position
                x_camera, y_camera, _ = camera_position
                theta_camera = theta

                return float(x_camera), float(y_camera), float(theta_camera)

    return None, None, None