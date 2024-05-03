# Custom messge to bundle ZED data together in a single message
std_msgs/Header header
# TODO: consider using compressed images instead of raw images to reduce bandwidth
sensor_msgs/Image left_n
sensor_msgs/Image right_n
sensor_msgs/Image left_n_1
sensor_msgs/Image right_n_1
builtin_interfaces/Time capture_time_n
builtin_interfaces/Time capture_time_n_1
sensor_msgs/Image depth_n