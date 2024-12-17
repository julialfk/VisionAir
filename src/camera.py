import cv2
  
# initialize the camera 
# If you have multiple camera connected with  
# current device, assign a value in cam_port  
# variable according to that 
cam_port = 0
print("init cam") 
cam = cv2.VideoCapture(cam_port) 

print("capture") 
# reading the input using the camera 

fgbg = cv2.createBackgroundSubtractorMOG2() 
  
# If image will detected without any error,  
# show result 
while True:
    result, image = cam.read() 
    print("show")   
    # showing result, it take frame name and image  
    # output 
    if not result:
        print("No image detected. Please! try again") 
        break
    cv2.imshow("GeeksForGeeks", image) 
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("gray", gray_image) 

    ret,thresh1 = cv2.threshold(gray_image,127,255,cv2.THRESH_BINARY)
    cv2.imshow("thresholded", thresh1) 

    fgmask = fgbg.apply(image) 
    cv2.imshow("thresholded", fgmask)

    print("write") 
    # saving image in local storage 
    cv2.imwrite("GeeksForGeeks.png", image) 
  
    # If keyboard interrupt occurs, destroy image  
    # window 
    key = cv2.waitKey(500)
    print(key)
    if key == 81:
        break
cv2.destroyWindow("GeeksForGeeks") 
  
# If captured image is corrupted, moving to else part 

    