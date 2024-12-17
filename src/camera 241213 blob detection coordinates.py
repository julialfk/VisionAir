import cv2
import numpy as np
  
# initialize the camera 
# If you have multiple camera connected with current device, assign a value in cam_port variable according to that 
cam_port = 0
print("init cam") 
cam = cv2.VideoCapture(cam_port) 

print("capture") 
# reading the input using the camera 

#Background subtractor
# fgbg = cv2.createBackgroundSubtractorMOG2() 
fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()

n=20
for i in range(n):
    restult, pic = cam.read()
    print("...")

result, first = cam.read() 
if not result:
    print("No image detected. Please! try again") 
# fgmask = fgbg.apply(first)
# first_gray = cv2.cvtColor(first, cv2.COLOR_BGR2GRAY)
# first_gray = cv2.GaussianBlur(first_gray, (21, 21), 0)
  
# If image will detected without any error, show result 
while True:
    # showing result, it takes frame name and image output as arguments
    result, image = cam.read() 

    if not result:
        print("No image detected. Please! try again") 
        break
    
    #original picture
    #cv2.imshow("GeeksForGeeks", image) 
    difference = cv2.absdiff(image, first)
    cv2.imshow("diff", difference)

    #convert original picture to grayscale
    gray = cv2.cvtColor(difference, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    cv2.imshow("grayscale", gray) 

    #convert grayscale to black and white with thresholding
    # ret,thresh1 = cv2.threshold(gray_image,125,255,cv2.THRESH_BINARY)
    thresh = cv2.threshold(gray, 17, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=15)
    cv2.imshow("thresholded", thresh) 

    #apply background subtraction
    # fgmask = fgbg.apply(image)
    # cv2.imshow("background subtracted", fgmask)

    # setting up blob detection
    # detector = cv2.SimpleBlobDetector() -- doesn't work "unexpected C++ error"
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 30000
    params.filterByColor = True
    params.blobColor = 255
    params.filterByConvexity = False
    params.filterByInertia = False
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(thresh)
    im_with_keypoints = cv2.drawKeypoints(thresh, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow("Keypoints", im_with_keypoints)

    #get keypoints parameters
    #list of blobs keypoints
    for kp in keypoints:
        x = kp.pt[0]
        y = kp.pt[1]
        s = kp.size
        print(f"x", x, "y", y, "size", s)
    
    # saving image in local storage 
    cv2.imwrite("GeeksForGeeks.png", image) 
  
    # If keyboard interrupt occurs, destroy image window 
    key = cv2.waitKey(1)
    print(key)
    if key == 81:
        break
cam.release()
cv2.destroyAllWindows()