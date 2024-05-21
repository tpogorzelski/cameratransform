import cameratransform as ct
import cv2
import numpy as np
from math import *

WINDOW_NAME = "output"
cv2.namedWindow(WINDOW_NAME,cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME,1024,1024)

def nothing(x):
    pass

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in first place) to euler roll, pitch, yaw
    quaternion = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    w = quaternion[0]
    x = quaternion[1]
    y = quaternion[2]
    z = quaternion[3]
    
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

def quaternionMult(quaternionOne, quaternionTwo):
    w1, x1, y1, z1 = quaternionOne
    w2, x2, y2, z2 = quaternionTwo
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return round(w, 6), round(x, 6), round(y, 6), round(z, 6)

def quaternionConjugate(quaternion):
    w, x, y, z = quaternion
    return (w, -x, -y, -z)

def quaternionvectorProduct(quaternion, vector): #https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    if quaternion[0] == 1:
        return 0, 1, 0, 0
    else:
        quaternion2 = (0.0,) + vector
        # print('test:', quaternionMult(quaternionMult(quaternion, quaternion2), quaternionConjugate(quaternion)))
        return quaternionMult(quaternionMult(quaternion, quaternion2), quaternionConjugate(quaternion))

def numTiles(z):
    return(pow(2,z))

def sec(x):
    return(1/cos(x))

def latlon2relativeXY(lat, lon):
    x = (lon + 180) / 360
    y = (1 - log(tan(radians(lat)) + sec(radians(lat))) / pi) / 2
    return(x,y)

def latlon2xy_float(lat, lon, z):
    n = numTiles(z) 
    x,y = latlon2relativeXY(lat,lon)
    return [n*x, n*y]

def latlon2xy(lat, lon, z):
    x, y = latlon2xy_float(lat, lon, z)
    return [round(x), round(y)]

def xy2latlon(x, y, z):
    n = numTiles(z)
    relY = y / n
    lat = mercatorToLat(pi * (1 - 2 * relY))
    lon = -180.0 + 360.0 * x / n
    return[lat, lon]

def mercatorToLat(mercatorY):
    return(degrees(atan(sinh(mercatorY))))

def overlay_image(background, image, transparency=50):
    
    no_black = np.any(image != [0, 1, 0], axis=-1)
    mask = np.zeros_like(image)
    mask[no_black] = [255, 255, 255]
    inv_mask = cv2.bitwise_not(mask)

    background_image = cv2.subtract(background, inv_mask)
    background = cv2.subtract(background, mask)
    
    output = cv2.addWeighted(background_image, 1-transparency/100, image, transparency/100, 0)

    return cv2.add(background, output)

gnss_location_geo = [50.1169801,22.3066840,913.4144,0,0,0,0.715877,0.051987,0.068798,0.692882,333.53000] #frame from AHRS

map_image = cv2.imread("259.7_map.png")
altitude_gl = 182.0
altitude_agl = round(gnss_location_geo[2] - altitude_gl)
map_zoom = 15
center_tile = latlon2xy(gnss_location_geo[0], gnss_location_geo[1], map_zoom)

center_map_geo = xy2latlon(center_tile[0]+0.5, center_tile[1]+0.5, map_zoom) #LU corner + half tile to get center

C_Earth = 40075016.686
tile_width_m = C_Earth*cos(radians(gnss_location_geo[0]))/2**map_zoom
map_width_m = round(tile_width_m*7)

# intrinsic camera parameters
camera_image = cv2.imread("259.7.png")
focallength_mm = 6   
sensorsize_mm = (6.17, 4.55)    

#extrinsic camera parameters
q0_init = gnss_location_geo[6]
q1_init = gnss_location_geo[7]
q2_init = gnss_location_geo[8]
q3_init = gnss_location_geo[9] 

q_init = [q0_init,q1_init,q2_init,q3_init] #wxyz
euler = euler_from_quaternion(q_init)
roll_init = round(euler[0])
pitch_init = round(euler[1])
yaw_init = round(euler[2])

pos_x_m = 0
pos_y_m = 0



#sliders
cv2.createTrackbar("focallength_mm",WINDOW_NAME,focallength_mm,20,nothing)
cv2.createTrackbar("elevation_m",WINDOW_NAME,altitude_agl,1500,nothing)
cv2.createTrackbar("roll_deg", WINDOW_NAME, roll_init, 180,nothing)
cv2.createTrackbar("tilt_deg",WINDOW_NAME,pitch_init,180,nothing)
cv2.createTrackbar("heading_deg", WINDOW_NAME, yaw_init, 180,nothing)
cv2.createTrackbar("pos_x_m", WINDOW_NAME, pos_x_m, 1000,nothing)
cv2.createTrackbar("pos_y_m", WINDOW_NAME, pos_y_m, 1000,nothing)
cv2.createTrackbar("transparency", WINDOW_NAME, 90, 100,nothing)

cv2.setTrackbarMin("focallength_mm",WINDOW_NAME, 2)
cv2.setTrackbarMin("elevation_m",WINDOW_NAME, 0)
cv2.setTrackbarMin("roll_deg",WINDOW_NAME, -180)
cv2.setTrackbarMin("tilt_deg",WINDOW_NAME, -180)
cv2.setTrackbarMin("heading_deg",WINDOW_NAME, -180)
cv2.setTrackbarMin("pos_x_m",WINDOW_NAME, -300)
cv2.setTrackbarMin("pos_y_m",WINDOW_NAME, -300)
cv2.setTrackbarMin("transparency",WINDOW_NAME, 0)

while True:
    
    # initialize the camera
    cam = ct.Camera(ct.RectilinearProjection(focallength_mm=focallength_mm,
                                        sensor=sensorsize_mm,
                                        image=(camera_image.shape[1], camera_image.shape[0])))

    cam.elevafocallength_mmtion_m = cv2.getTrackbarPos("focallength_mm",WINDOW_NAME)
    cam.elevation_m = cv2.getTrackbarPos("elevation_m",WINDOW_NAME)
    cam.roll_deg = cv2.getTrackbarPos("roll_deg",WINDOW_NAME)
    cam.tilt_deg = cv2.getTrackbarPos("tilt_deg",WINDOW_NAME)
    cam.heading_deg = cv2.getTrackbarPos("heading_deg",WINDOW_NAME)
    cam.pos_x_m = cv2.getTrackbarPos("pos_x_m",WINDOW_NAME)
    cam.pos_y_m = cv2.getTrackbarPos("pos_y_m",WINDOW_NAME)
    transparency = cv2.getTrackbarPos("transparency",WINDOW_NAME)
    
    top_camera_image = cam.getTopViewOfImage(camera_image, [round(-1792/2), round(1792/2), round(-1792/2), round(1792/2)], scaling=1, do_plot=False)

    top_camera_image = cv2.cvtColor(top_camera_image, cv2.COLOR_BGRA2BGR)

    output = overlay_image(map_image, top_camera_image, transparency)

    cv2.imshow(WINDOW_NAME, output)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()
        
cv2.destroyAllWindows()



# focallength_mm = 4
# altitude_agl = 297
# roll_init = -58
# pitch_init = 34
# yaw_init = 26
# pos_x_m = -113
# pos_y_m = 75