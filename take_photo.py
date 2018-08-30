import cv2

if __name__ == "__main__":
    pattern_size = (10, 9)
    total = 11
    capture: cv2.VideoCapture = cv2.VideoCapture(1)
    cv2.namedWindow("chessboard", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("chessboard", 1280, 720)
    while True:
        _, frame = capture.read()
        cv2.imshow("chessboard", frame)
        key = cv2.waitKey(10)
        if key == ord('y'):
            # save
            total += 1
            image_name = "chessboards/image_" + str(total) + ".jpg"
            cv2.imwrite(image_name, frame)