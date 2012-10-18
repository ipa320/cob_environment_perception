Header header

uint8 POLYGON=0
uint8 LINE=1
uint8 CURVED=2
uint8 MESH=3
uint8 OTHER=4
uint8 CYLINDER=5
# potential extensions: SPHERE, CYLINDER, BOX

uint8 type
int32 id
# define shape parameters
# for plane
# normal vector = params[0],params[1],params[2]
#	d = params[3]
# for line
# direction vector = params[0],params[1],params[2]
#
#for cylinder
# symmetry axis = params[0],params[1],params[2]
# z axis 	= params[3], params[4], params[5]
# origin	= params[6], params[7], params[8]
# radius	= params[9]
# 
float64[] params

sensor_msgs/PointCloud2[] points

#### define mesh ####
# each three entries form a triangle; indices of points are stored
int32[] vertices

geometry_msgs/Point32 centroid
std_msgs/ColorRGBA color
bool[] holes
