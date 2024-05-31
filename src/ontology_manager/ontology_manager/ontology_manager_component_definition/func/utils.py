import os
import numpy as np
from scipy.spatial.transform import Rotation

def rename_file(file_path):
  directory, filename = os.path.split(file_path)
  name, ext = os.path.splitext(filename)
  
  # Initialize a counter
  counter = 0

  # Construct the new filename with a counter
  new_filename = f"{name}_{counter}{ext}"
  
  # Check if the new filename already exists
  while os.path.exists(os.path.join(directory, new_filename)):
    counter += 1
    new_filename = f"{name}_{counter}{ext}"
  
  # Rename the file
  new_file_path = os.path.join(directory, new_filename)
  os.rename(file_path, new_file_path)
  
  return new_file_path

def create_tmp_file(self, file_path):
  # Copy the file to a new file name the same but with _tmp
  directory, filename = os.path.split(file_path)
  name, ext = os.path.splitext(filename)
  new_filename = f"{name}_tmp_0{ext}"
  
  if self.get_parameter('overwriteOntology').get_parameter_value().bool_value == True:
    counter = 0
    while os.path.exists(os.path.join(directory, new_filename)):
      counter += 1
      new_filename = f"{name}_tmp_{counter}{ext}"

  new_file_path = os.path.join(directory, new_filename)
  os.system(f'cp {file_path} {new_file_path}')

  return new_file_path

def ros_pose_to_euclidean(ros_pose_str):
    # ros_pose_str = "[[x, y, z], [w, y, z, w]]"
    # Parse the input string to extract position and orientation
    pose_str = ros_pose_str.replace("[", "").replace("]", "").split(",")
    position_str = pose_str[0:3]
    orientation_str = pose_str[3:7]

    # Convert position and orientation to numpy arrays
    position = np.array([float(coord) for coord in position_str])
    orientation = np.array([float(coord) for coord in orientation_str])
    
    # Converting Euler angles to rotation matrix
    rotation = Rotation.from_euler('xyz', orientation[:-1], degrees=True)
    
    # Transforming ROS pose to Euclidean pose
    euclidean_position = position
    euclidean_orientation = rotation.as_euler('xyz', degrees=True)
    
    eucli_pose = np.array([euclidean_position, euclidean_orientation])
    return eucli_pose
