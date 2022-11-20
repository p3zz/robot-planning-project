# An array of poses that represents a Path for a robot to follow.

# Indicates the frame_id of the path.
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# Array of poses to follow.
geometry_msgs/PoseStamped[] poses
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	Pose pose
		Point position
			float64 x=0
			float64 y=0
			float64 z=0
		Quaternion orientation
			float64 x=0
			float64 y=0
			float64 z=0
			float64 w=1

    std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	Pose pose
		Point position
			float64 x=3
			float64 y=0
			float64 z=0
		Quaternion orientation
			float64 x=0
			float64 y=0
			float64 z=0
			float64 w=1
        
