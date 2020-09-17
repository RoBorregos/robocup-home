'''
Things to remember:
+ The RS camera, when a depth pixel is unkown (a dead place),
  sets the pixel value to 0.
+ As get_objects_and_coordinates_in_terminal.py outputs,
  these assumes coordinates are in pixels but in float.

Note: All the algorithms here assume that the depth and color
frames are the same size; will fail somewhere if is not true.
Remember that filters like "depth decimation" shrinks the depth
image.

TODO: As this comes from from a nn, maybe
ensure the coord are inside the img size, and do agjustment to
the algorithms to ensure they are using the inclusive/exclusive treatments.
'''

def depth_segment_to_get_center_and_angle_of_object(
  image, depth_image, obj, paint_image=False):
  '''
  Calculate center of mass and inclination angles in 3dims by segmenting
  the object using the depth.

  This outputs the center of mass and the inclination trying to
  do it in "3D" using the x,y and z,y planes, this by expanding the
  formulas to use the depth values. Uses the image moments. For this, 
  a breath-first traverse is done inside the object's square using
  the depth values to guide it.

  TODO: Maybe if thinking the formulas more they can be improved,
  as the concept seems to give useful results.
  '''
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

    # Add the sorrounding coords to be traverse, only if they are inside
    # the square, not yet done, not unkown, and not a big change of depth.
    next_coords = (
      (actual_coord[0] - 1, actual_coord[1]),
      (actual_coord[0], actual_coord[1] + 1),
      (actual_coord[0] + 1, actual_coord[1]),
      (actual_coord[0], actual_coord[1] - 1),
    )
    for next_coord in next_coords:
      if (next_coord[0] >= obj.y_max or next_coord[0] < obj.y_min or
        next_coord[1] >= obj.x_max or next_coord[1] < obj.x_min or
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
