# This is msg type not intented to be published by itself alone,
# but with objects_detected msg, that's why doesn't include a Header
# field.
# TODO: Include z to express avg distance to object.
string name
float32 score
float32 y_min
float32 x_min
float32 y_max
float32 x_max
