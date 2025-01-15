import cv2
import sys

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

img = cv2.imread('OXO200Front.jpg')

if __name__ == '__main__' :

# Set up tracker.
# KCF works best with fast objects but does bad if the object is lost and the ROI box is a fixed size.
# TLD is slower but it scales to the object and can recover lost objects.
# Medianflow scales randomly and cannot recover lost object.
# For the GOTURN you need to download the goturn.prototxt file and goturn.caffemodel from the net and put it in same folder as your current working directory to run it.
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
    tracker_type = tracker_types[3]

if int(minor_ver) < 3:
    tracker = cv2.Tracker_create(tracker_type)
else:
    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.legacy.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.legacy.TrackerMedianFlow_create()
    if tracker_type == 'GOTURN':
        tracker = cv2.TrackerGOTURN_create()

cap = cv2.VideoCapture(0)
ok, frame = cap.read()

n=20
for i in range(n):
    ok, frame = cap.read()
    cv2.imshow("Tracking", frame)

#Initializing a bounding box 
bbox = img

bbox = cv2.selectROI(frame, False)
ok = tracker.init(frame, bbox)

while(1):
    ok, frame = cap.read()
    timer = cv2.getTickCount()
    ok, bbox = tracker.update(frame)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
    if ok:
        (sp_x, sp_y, w, h) = [int(v) for v in bbox]
        cv2.rectangle(frame, (sp_x, sp_y), (sp_x+w, sp_y+h), (255,0,0), 2, 1)
        x = sp_x + ((w-sp_x)/2)
        y = sp_y + ((h-sp_y)/2)
        # size = diagonal of the bounding box rectangle
        s = (w**2 + h**2)**(0.5)
        print(f'x, y, size: {x, y, s}')
    else :
        # Tracking failure
        cv2.putText(frame, "Tracking failure detected", (100,80), 
        cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
        cv2.putText(frame, tracker_type + " Tracker", (100,20), 
        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);

    # Display FPS on frame
    cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), 
    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);

    # Display result
    cv2.imshow("Tracking", frame)

    K = cv2.waitKey(1)
    if (K==27):
        break
cap.release()
cv2.destroyAllWindows()