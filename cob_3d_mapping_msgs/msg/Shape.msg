Header header

byte POLYGON=0
byte LINE=1
byte CURVED=2
byte MESH=3
byte OTHER=4
# potential extensions: SPHERE, CYLINDER, BOX

byte type
int32 id

# define shape parameters
# for plane
# normal vector = params[0],params[1],params[2]
#	d = params[3]
# for line
# direction vector = params[0],params[1],params[2]
float64[] params

sensor_msgs/PointCloud2[] points

#### define mesh ####
# each three entries form a triangle; indices of points are stored
int32[] vertices

geometry_msgs/Point32 centroid
std_msgs/ColorRGBA color
bool[] holes
