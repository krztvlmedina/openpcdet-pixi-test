# Performance metrics message for the detection node
# Published on /perf/detection topic

std_msgs/Header header              # Contains timestamp and frame_id
uint32 sequence_id                  # Sequential frame number
uint32 input_point_count            # Points received
uint32 detection_count              # Number of detections
float64 receive_timestamp           # When pointcloud was received (epoch seconds)
float64 preprocess_end_timestamp    # When preprocessing finished (epoch seconds)
float64 inference_end_timestamp     # When model inference finished (epoch seconds)
float64 postprocess_end_timestamp   # When postprocessing finished (epoch seconds)
string model_name                   # Which model is running
