import cv2

img = cv2.imread('/home/bcsh/upros_class_code/src/upros_navigation/maps/my_lab.pgm')  # 读取图像

# 获取某个像素点的像素值（以BGR通道顺序为例，即Blue、Green、Red）
# 假设要获取像素点(100, 200)的像素值



def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        b, g, r = img[y, x]
        print("x,y:",x, y,"BGR:", b, g, r)
       # img[y, x] = (0, 0, 0)    #black
        # img[y, x] = (205, 205, 205)
        img[y, x] = (254, 254, 254) # white
        cv2.imshow("image", img)
        
cv2.namedWindow("image")
cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
while(1):
    cv2.imshow("image", img)
    key = cv2.waitKey(5) & 0xFF
    if key == ord(' '):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imwrite('/home/bcsh/upros_class_code/src/upros_navigation/maps/my_lab1.pgm', gray) 
        break
cv2.destroyAllWindows()
