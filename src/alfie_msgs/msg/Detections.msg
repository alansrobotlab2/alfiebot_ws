# All open-vocabulary detections for a single camera frame.
std_msgs/Header header   # stamp + frame_id copied from the source image
string source_topic      # image topic the frame came from
string prompt            # comma-separated queries used for this inference
uint32 image_width       # source image width, pixels
uint32 image_height      # source image height, pixels
Detection[] detections
