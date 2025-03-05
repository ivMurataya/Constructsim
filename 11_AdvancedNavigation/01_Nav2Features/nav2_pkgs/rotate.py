import cv2

# Load the PGM file
image = cv2.imread("map_keepout.pgm", cv2.IMREAD_UNCHANGED)

# Rotate the image 90 degrees counterclockwise
rotated_image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)

# Save the rotated image
cv2.imwrite("map_keepout_rotated.pgm", rotated_image)
