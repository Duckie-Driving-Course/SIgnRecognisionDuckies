import cv2


class EdgeDetector:
    target = 0
    img = 0
    img_gray = 0
    img_blur = 0

    def __init__(self, img):
        self.img = cv2.imread(img)

    def initialize_image(self):
        self.img_gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.img_blur = cv2.GaussianBlur(self.img_gray, (3, 3), 0)

    def sobel_edge_detection(self):
        sobelx = cv2.Sobel(src=self.img_blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5)
        sobely = cv2.Sobel(src=self.img_blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5)
        sobelxy = cv2.Sobel(src=self.img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5)

        return sobelx, sobely, sobelxy

    def canny_edge_detection(self,img):
        edges = cv2.Canny(image=img, threshold1=100, threshold2=200)

        return edges

    def display(self,mode):
        cv2.imshow('Original', cv2.resize(self.img, (982,800)))
        cv2.waitKey(0)
        cv2.imshow('Gray', cv2.resize(self.img_gray, (982,800)))
        cv2.waitKey(0)
        cv2.imshow('Blur', cv2.resize(self.img_blur, (982,800)))
        cv2.waitKey(0)
        if mode == "sobel":
            sobelx, sobely, sobelxy = self.sobel_edge_detection()
            cv2.imshow('Sobel X', cv2.resize(sobelx,(982,800)))
            cv2.waitKey(0)
            cv2.imshow('Sobel Y', cv2.resize(sobely,(982,800)))
            cv2.waitKey(0)
            cv2.imshow('Sobel X Y using Sobel() function', cv2.resize(sobelxy,(982,800)))
            cv2.waitKey(0)
        elif mode == "canny":
            edges = self.canny_edge_detection(self.img_gray)
            resized_edges = cv2.resize(edges, (982,800))
            cv2.imshow('Canny Edge Detection', resized_edges)
            cv2.waitKey(0)

    def match(self, trgt):
        temp_img = self.canny_edge_detection(self.img_blur)
        target_img = self.canny_edge_detection(cv2.imread(trgt,cv2.IMREAD_GRAYSCALE))
        w, h = target_img.shape[::-1]
        res = cv2.matchTemplate(temp_img, target_img, cv2.TM_CCOEFF)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
        top_left = max_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv2.rectangle(temp_img, top_left, bottom_right, 200, 2)

        cv2.imshow('match', cv2.resize(temp_img,(982,800)))
        cv2.waitKey(0)


image = "/home/alsanon/PycharmProjects/SIgnRecognisionDuckies/assets/signs/field_test1.jpg"
target = "/home/alsanon/PycharmProjects/SIgnRecognisionDuckies/assets/signs/stop.jpg"
target2 = "/home/alsanon/PycharmProjects/SIgnRecognisionDuckies/assets/canny_signs/canny_sign_6_cropped_bottom.jpg"
edgo = EdgeDetector(image)
edgo.initialize_image()
edgo.display("canny")



