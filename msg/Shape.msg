# A client-assigned value, used to refer to unique shapes.
int32 id

# If true, deletes the shape with the same id.
bool delete

# The type of the shape, as a SHP_* constant
int32 type

# If true, fill the shape. Not all shapes may use this parameter.
bool fill

# The thickness of the line. Not all shapes may use this parameter.
float32 thickness

# The color of the shape.
uint8[] color

# Arbitrary data storage, used differently
Point[] poly
int32[] param

# Constants:
int32 SHP_NONE = 0
int32 SHP_CIRCLE = 1