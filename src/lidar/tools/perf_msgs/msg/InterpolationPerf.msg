# Performance metrics message for the interpolation node
# Published on /perf/interpolation topic

std_msgs/Header header          # Contains timestamp and frame_id
uint32 sequence_id              # Sequential frame number
uint32 input_point_count        # Points before interpolation
uint32 output_point_count       # Points after interpolation
float64 receive_timestamp       # When input was received (epoch seconds)
float64 process_start_timestamp # When interpolation started (epoch seconds)
float64 process_end_timestamp   # When interpolation finished (epoch seconds)
float64 publish_timestamp       # When output was published (epoch seconds)
string config_name              # Which interpolation config is active
