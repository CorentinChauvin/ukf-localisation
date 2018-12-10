#
# Description of a cone
#

# 2D-position of the cone
float64 x
float64 y

# Color of the cone
uint8 UNDEFINED = 0
uint8 YELLOW = 1
uint8 BLUE = 2
uint8 ORANGE = 3

uint8 color

# Covariance on the position [m^2] (2x2 matrix in row-major order)
float64[4] covariance

# Confidence in the detection
float64 probability
