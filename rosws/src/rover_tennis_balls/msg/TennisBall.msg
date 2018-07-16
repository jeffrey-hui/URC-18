Header header

uint8 tennis_ball_type
geometry_msgs/Vector3 position_or_ray_dir # if pos, 3d pos in frame of this message, if ray ray originating from origin of this frame in this direction
float32 confidence # between 0-1

uint8 TENNIS_BALL_RAY=1
uint8 TENNIS_BALL_POS=2

