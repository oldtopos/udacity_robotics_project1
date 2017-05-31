import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh_low=(160, 160, 160), rgb_thresh_high=(256, 256, 256)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh =  np.logical_and(img[:,:,0] > rgb_thresh_low[0], img[:,:,0] < rgb_thresh_high[0] ) \
                 &   np.logical_and(img[:,:,1] > rgb_thresh_low[1], img[:,:,1] < rgb_thresh_high[1] ) \
                 &   np.logical_and(img[:,:,2] > rgb_thresh_low[2], img[:,:,2] < rgb_thresh_high[2] )
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select, above_thresh.size

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    yaw_rad = yaw * np.pi / 180
    x_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    y_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)  
    return x_rotated, y_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    xpix_translated = np.int_(xpos + (xpix_rot / scale))
    ypix_translated = np.int_(ypos + (ypix_rot / scale)) 
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    image = Rover.img

    scale = 10.0
    worldsize =  Rover.worldmap.shape[0]
    xpos = Rover.pos[ 0 ]
    ypos = Rover.pos[ 1 ] 
    yaw = Rover.yaw
    roll = Rover.roll
    pitch = Rover.pitch
    
    if roll > 180.0:
        roll -= 360.0
        
    if pitch > 180.0:
        pitch -= 360.0

    # Define calibration box in source (actual) and destination (desired) coordinates
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    # The destination box will be 2*dst_size on each side
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                      [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      ])
    
    warped = perspect_transform(image, source, destination)
    navigable, nav_size = color_thresh(warped)
    obstacle_threshed = 1 - navigable

    # Calculate pixel values in rover-centric coords for obstacles
    xpix, ypix = rover_coords(obstacle_threshed)
    xworld, yworld = pix_to_world(xpix, ypix, xpos, ypos, yaw, worldsize, scale )
    
    #
    #  Update modified to more aggressively indicate sensing and clip stored values
    #
    if roll > -0.75 and roll < 0.75 and pitch > -0.75 and pitch < 0.75:
        Rover.worldmap[yworld, xworld, 0] += 1.0
        np.clip(Rover.worldmap[:,:,0], 0, 255, out=Rover.worldmap[:,:,0])

    # Calculate pixel values in rover-centric coords and distance/angle to all pixels
    xpix, ypix = rover_coords(navigable)
    xworld, yworld = pix_to_world(xpix, ypix,xpos, ypos, yaw, worldsize, scale )
     
    Rover.nav_dist, Rover.nav_angles = to_polar_coords(xpix, ypix)
    Rover.nav_mean_dir  = np.mean(Rover.nav_angles)
   
    #
    #  Update modified to more aggressively indicate sensing and clip stored values
    #
    if roll > -0.75 and roll < 0.75 and pitch > -0.75 and pitch < 0.75:
         Rover.worldmap[yworld, xworld, 2] += (255.0 - Rover.worldmap[yworld, xworld, 2]) / 3.5
         Rover.worldmap[yworld, xworld, 0] = 1.0
         np.clip(Rover.worldmap[:,:,2], 0, 255, out=Rover.worldmap[:,:,2])
    
    #
    #  Look for rocks, set values to None if no target in camera view
    #
    rock_threshed, rock_size = color_thresh( warped, ( 40, 110, 0 ), ( 255, 240, 90 ) )

    if rock_size > 0:
        # Calculate pixel values in rover-centric coords and distance/angle to all pixels
        xpix, ypix = rover_coords(rock_threshed)
        xworld, yworld = pix_to_world(xpix, ypix, xpos, ypos, yaw, worldsize, scale )
        
        Rover.nav_rock_dist, Rover.nav_rock_angles = to_polar_coords(xpix, ypix)
        Rover.nav_mean_rock_dir = np.mean(Rover.nav_rock_angles)
        #
        #  Update modified to more aggressively indicate sensing and clip stored values
        #
        if roll > -0.75 and roll < 0.75 and pitch > -0.75 and pitch < 0.75:
            Rover.worldmap[yworld, xworld, 1] += (255.0 - Rover.worldmap[yworld, xworld, 1]) / 2.0
            np.clip(Rover.worldmap[:,:,1], 0, 255, out=Rover.worldmap[:,:,1])
    else:
        Rover.nav_rock_dist = None
        Rover.nav_rock_angles = None
        Rover.nav_mean_rock_dir = None

    
    
    return Rover
