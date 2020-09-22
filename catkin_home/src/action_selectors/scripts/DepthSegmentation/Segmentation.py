"""
Contains algorithms that compute information of the framed objects by the 
object detection nn using the depth frame by the Intel RealSense Camera.

In general, these give okay results that are extra useful information to
use as reference. But the algorithms are kind of flaky as the input's quality
changes a lot. Also, these were originally developed to analyze  objects 
like cups, bottles, glasses; continous, solid objects (not like chairs,
tables).

The algorithms are `depth_segment_to_get_center_and_angle_of_object` and
`depth_trace_limits_of_four_sides_of_object`.

Things to remember:
+ The RS camera, when a depth pixel is unkown (a dead place),
  sets the pixel value to 0.
+ As get_objects_and_coordinates_in_terminal.py outputs,
  these assumes coordinates are in pixels but in float.
+ The RS ROS node depth frame's values are in millimeters.

Note: All the algorithms here assume that the depth and color
frames are the same size; will fail somewhere if is not true.
Remember that filters like "depth decimation" shrinks the depth
image.

TODO: As this comes from from a nn, maybe ensure the coord are inside the img
size, and adjust the algorithms to ensure they are using the 
inclusive/exclusive agreements.
"""
import pyrealsense2 as rs


def depth_segment_to_get_center_and_angle_of_object(
  image, depth_image, obj, paint_image=False):
  """
  Calculate center of mass and inclination angles in 3dims by segmenting
  the object using the depth.

  This outputs the center of mass and the inclination trying to
  do it in "3D" using the x,y and z,y planes, this by expanding the
  image moments formulas to use the depth values. For this, a 
  breath-first traverse is done inside the object's square using the
  depth values to guide it.

  TODO: Maybe if thinking the formulas more they can be improved,
  as the concept seems to give useful results.
  """
  # From the center of the square, get a starting point that
  # is the median of the nearby values.
  center_x = int((obj.x_max - obj.x_min) / 2 + obj.x_min)
  center_y = int((obj.y_max - obj.y_min) / 2 + obj.y_min)

  sub_section = depth_image[center_y - 2:center_y + 2, center_x - 3: center_x + 3]
  arg_median = np.argsort(sub_section, axis=None)[sub_section.size//2]
  initial_coord = (
    arg_median/sub_section.shape[1] + center_y, 
    arg_median%sub_section.shape[1] + center_x,
  )
  # print(sub_section, depth_image[initial_coord[0], initial_coord[1]])


  # Init the values for the segmentation and image moments. 
  m00 = 0
  m10 = 0
  m01 = 0
  m001 = 0
  m011 = 0
  m11 = 0
  m20 = 0
  m02 = 0
  m002 = 0

  # Traverse in depth-first order the object from the center, stoping
  # when the depth diff is too big.
  pixel_done = np.full((depth_image.shape), False)
  queue_next = [initial_coord]
  while len(queue_next) > 0:
    actual_coord = queue_next.pop()
    actual_value = depth_image[actual_coord[0], actual_coord[1]]

    pixel_done[actual_coord[0], actual_coord[1]] = True

    if paint_image:
      # TODO: Maybe change `paint_image` to be None or a tuple of
      # pixel values to set this.
      image[actual_coord[0], actual_coord[1], 0] = 255

    m00 += 1
    m10 += actual_coord[1]
    m01 += actual_coord[0]
    m001 += int(depth_image[actual_coord[0], actual_coord[1]])
    m11 += actual_coord[0] * actual_coord[1]
    m011 += actual_coord[0] * int(depth_image[actual_coord[0], actual_coord[1]])
    m20 += actual_coord[1] ** 2
    m02 += actual_coord[0] ** 2
    m002 += int(depth_image[actual_coord[0], actual_coord[1]]) ** 2

    # Add the surrounding coords to be traversed, only if they are inside
    # the square, not done yet, not unkown, and not a big change of depth.
    next_coords = (
      (actual_coord[0] - 1, actual_coord[1]),
      (actual_coord[0], actual_coord[1] + 1),
      (actual_coord[0] + 1, actual_coord[1]),
      (actual_coord[0], actual_coord[1] - 1),
    )
    for next_coord in next_coords:
      if (next_coord[0] >= obj.y_max or next_coord[0] <= obj.y_min or
        next_coord[1] >= obj.x_max or next_coord[1] <= obj.x_min or
        pixel_done[next_coord[0], next_coord[1]] or
        depth_image[next_coord[0], next_coord[1]] == 0 or
        abs(int(actual_value) - int(depth_image[next_coord[0], next_coord[1]])) > 5):
        continue
      queue_next.append(next_coord)


  # Calculate the final useful values to get the center of mass and
  # the degrees of inclination in x,y and z,y planes.
  x_median = m10 / m00
  y_median = m01 / m00
  miu11 = m11 - x_median * y_median * m00
  miu20 = m20 - x_median * m10
  miu02 = m02 - y_median * m01

  z_median = m001 / m00
  miu011 = m011 - z_median * y_median * m00
  miu002 = m002 - z_median * m001

  angle_x_y = 0.5 * math.atan2(2 * miu11, miu20 - miu02)
  angle_x_y = math.degrees(angle_x_y)
  angle_z_y = 0.5 * math.atan2(2 * miu011, miu002 - miu02)
  angle_z_y = math.degrees(angle_z_y)
  print("ANGLE=", angle_x_y, angle_z_y)

  return (angle_x_y, angle_z_y), (x_median, y_median, z_median)


def depth_trace_limits_of_four_sides_of_object(
  image, depth_image, obj, intrinsics, center_x_y=None, angle_x_y=None,
  paint_image=False):
  """
  Find limits of the four sides (front plane: up, right, down, left) of the
  possibly inclined object by following the depth values.

  This returns the horizontal and vertical limit values in millimeters.
  These are, the x,y,z position translated to real measurements relative to the
  camera of the four sides of the object. This moves from the center of the
  square to the extreme of each side trying to change a little the path 
  when a hole or big diff in depth is found. Also, tries to account for
  the inclination of the obj by "logically rotating" the drawn cross.

  TODO: In general this ~usually~ worked. The idea is nice, but maybe the
  calculations and conditions can be improved.
  TODO: Join the four movements into one (method?).
  """
  # The "total" pixels that each step the traverse will move. This can
  # be seen as the hypotenus of the movement, as is later used to calculate
  # the actual K_x,K_y. If greater the faster, but easier to make mistakes.
  K = 3

  # Calculate `K_y` and `K_x` that will be the reference when moving 
  # in each step. K_y is pointing to the current direction, K_x to the next 
  # 90' clockwise direction, i.e. if going north: K_y is north and K_x is
  # right.
  # `angle_x_y` is used to calculate this, as this is to align the traverse
  # with the inclination of the obj to correctly arrive to the four sides.
  if angle_x_y != None:
    # TODO: This needs review and could be improved.
    # This tries to make the angle given by the calculated moments that
    # [i believe] are only between 90,-90 to something useful here. Also,
    # includes some params to modulate the actual impact of the angle in
    # the ratio between K_x and K_y.
    angle_x_y = angle_x_y - (90 - angle_x_y) * 1.0 if angle_x_y > 0 else angle_x_y + (90 + angle_x_y) * 1.0
    K_y, K_x = K * math.sin(math.radians(angle_x_y)), K * math.cos(math.radians(angle_x_y))
    K_x *= -1 if angle_x_y > 0 else 1
    K_y, K_x = int(round(abs(K_y))), int(round(K_x))
  else:
    K_y, K_x = K, 0
  print("Ks = ", K_y, K_x)

  if center_x_y == None:
    # TODO: Check if makes sense to also use here the median center like
    # the one at other method.
    center_x = int((obj.x_max - obj.x_min) / 2 + obj.x_min)
    center_y = int((obj.y_max - obj.y_min) / 2 + obj.y_min)
  else:
    (center_x, center_y) = center_x_y


  # Left (west). When blocked: left-north.
  y, x = center_y, center_x
  continuous_holes = 0
  continuous_changes = 0
  past_y, past_x = y, x
  while (x > obj.x_min and y > obj.y_min + (obj.y_max - obj.y_min) * 0.30 and
    continuous_holes < 10 and continuous_changes < 10):
    # The new coord is inside of the square by the left, hasn't move too
    # much to the north, and the continuous holes found and changes required
    # haven't been too much.
    if paint_image:
      image[y, x, :] = (0, 255, 0)

    if depth_image[y, x] == 0:
      # A hole found, let's try in another row to the north.
      # TODO: Maybe here and below, the twist in the route, should be
      # related to the values/ratio of/between K_y, K_x.
      y -= 2
      x -= 0
      continuous_holes += 1
      continue
    continuous_holes = 0

    # if abs(int(depth_image[y, x]) - int(depth_image[center_y, center_x])) > 50:
    #   # The current depth is too big in compar with the depth of the center.
    #   break

    if (abs(int(depth_image[y, x]) - int(depth_image[past_y, past_x])) > 15):
      # The current depth is too big in compar with the last one found.
      # TODO: Make the threshold be related to K.
      y -= 2
      x -= 1
      continuous_changes += 1
      continue
    continuous_changes = 0

    past_y, past_x = y, x
    # Take a step in the calculated real direction to left.
    x -= K_y
    y -= K_x
  left_y, left_x = past_y, past_x
  print("Stats left = ", continuous_holes, continuous_changes)

  # Right (east). When blocked: right-south
  y, x = center_y, center_x
  continuous_holes = 0
  continuous_changes = 0
  past_y, past_x = y, x
  while (x < obj.x_max and y < obj.y_max - (obj.y_max - obj.y_min) * 0.30 and
    continuous_holes < 10 and continuous_changes < 10):
    if paint_image:
      image[y, x, :] = (0, 255, 0)

    if depth_image[y, x] == 0:
      y += 2
      x += 0
      continuous_holes += 1
      continue
    continuous_holes = 0

    # if abs(int(depth_image[y, x]) - int(depth_image[center_y, center_x])) > 50:
    #   break

    if (abs(int(depth_image[y, x]) - int(depth_image[past_y, past_x])) > 15):
      y += 2
      x += 1
      continuous_changes += 1
      continue
    continuous_changes = 0

    past_y, past_x = y, x
    x += K_y
    y += K_x
  right_y, right_x = past_y, past_x
  print("Stats right = ", continuous_holes, continuous_changes)

  # North. When blocked: north-right
  y, x = center_y, center_x
  continuous_holes = 0
  continuous_changes = 0
  past_y, past_x = y, x
  while (y > obj.y_min and x < obj.x_max - (obj.x_max - obj.x_min) * 0.30 and
    continuous_holes < 10 and continuous_changes < 10):
    if paint_image:
      image[y, x, :] = (0, 255, 0)

    if depth_image[y, x] == 0:
      y -= 0
      x += 2
      continuous_holes += 1
      continue
    continuous_holes = 0

    # if abs(int(depth_image[y, x]) - int(depth_image[center_y, center_x])) > 50:
    #   break

    if (abs(int(depth_image[y, x]) - int(depth_image[past_y, past_x])) > 15):
      y -= 1
      x += 2
      continuous_changes += 1
      continue
    continuous_changes = 0

    past_y, past_x = y, x
    x += K_x
    y -= K_y
  north_y, north_x = past_y, past_x
  print("Stats north = ", continuous_holes, continuous_changes)

  # South. When blocked: south-left
  y, x = center_y, center_x
  continuous_holes = 0
  continuous_changes = 0
  past_y, past_x = y, x
  while (y < obj.y_max and x > obj.x_min + (obj.x_max - obj.x_min) * 0.30 and
    continuous_holes < 10 and continuous_changes < 10):
    if paint_image:
      image[y, x, :] = (0, 255, 0)

    if depth_image[y, x] == 0:
      y += 0
      x -= 2
      continuous_holes += 1
      continue
    continuous_holes = 0

    # if abs(int(depth_image[y, x]) - int(depth_image[center_y, center_x])) > 50:
    #   break

    if (abs(int(depth_image[y, x]) - int(depth_image[past_y, past_x])) > 15):
      y += 1
      x -= 2
      continuous_changes += 1
      continue
    continuous_changes = 0

    past_y, past_x = y, x
    x -= K_x
    y += K_y
  south_y, south_x = past_y, past_x
  print("Stats south = ", continuous_holes, continuous_changes)

  
  # Using the intrinsics and the deprojection of the RS, translate to real
  # measurements the four points. For this, convert the pixels to the
  # original img size and the depth (ROS) millis to the cm used in the `rs2`.
  orig_h, orig_w = intrinsics.height, intrinsics.width
  left = rs.rs2_deproject_pixel_to_point(
    intrinsics, 
    [float(left_x) / image.shape[1] * orig_w, float(left_y) / image.shape[0] * orig_h],
    depth_image[left_y, left_x] * 0.1)
  right = rs.rs2_deproject_pixel_to_point(
    intrinsics,
    [float(right_x) / image.shape[1] * orig_w, float(right_y) / image.shape[0] * orig_h], 
    depth_image[right_y, right_x] * 0.1)
  north = rs.rs2_deproject_pixel_to_point(
    intrinsics, 
    [float(north_x) / image.shape[1] * orig_w, float(north_y) / image.shape[0] * orig_h],
    depth_image[north_y, north_x] * 0.1)
  south = rs.rs2_deproject_pixel_to_point(
    intrinsics,
    [float(south_x) / image.shape[1] * orig_w, float(south_y) / image.shape[0] * orig_h], 
    depth_image[south_y, south_x] * 0.1)
  
  print("Horiz dist = ",
    math.sqrt(math.pow(left[0]-right[0], 2) + math.pow(left[1]-right[1], 2) + math.pow(left[2]-right[2], 2)))
  print("Vert dist = ",
    math.sqrt(math.pow(north[0]-south[0], 2) + math.pow(north[1]-south[1], 2) + math.pow(north[2]-south[2], 2)))

  return north, right, south, left
