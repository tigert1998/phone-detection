import numpy as np
import cv2
import glob


def create_window(name, size=(1280, 720)):
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, size[0], size[1])


def calc_mean_error(object_points, image_points, rotation_vectors,
                    translation_vectors, camera_matrix, distortion_coefficients):
    mean_error = 0
    for i in range(len(object_points)):
        _img_points, _ = cv2.projectPoints(
            object_points[i],
            rotation_vectors[i],
            translation_vectors[i],
            camera_matrix,
            distortion_coefficients
        )
        error = cv2.norm(image_points[i], _img_points, cv2.NORM_L2) / len(_img_points)
        mean_error += error
    mean_error /= len(object_points)
    return mean_error


def save_calibration_coefficients(camera_matrix, distortion_coefficients):
    file_storage: cv2.FileStorage = cv2.FileStorage("calibration_coefficients.yml", cv2.FILE_STORAGE_WRITE)
    file_storage.write("camera_matrix", camera_matrix)
    file_storage.write("distortion_coefficients", distortion_coefficients)


if __name__ == "__main__":
    termination_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    pattern_size = (9, 10)
    object_points_single_group = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    object_points_single_group[:, :2] = np.mgrid[0: pattern_size[0], 0: pattern_size[1]].T.reshape(-1, 2)

    object_points = []
    image_points = []

    images = list(glob.glob('chessboards/*.jpg'))

    # create_window("Image")

    for file_name in images:
        image = cv2.imread(file_name)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        result, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if result:
            object_points.append(object_points_single_group)
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), termination_criteria)
            image_points.append(corners)

            # Draw and display the corners
            # cv2.drawChessboardCorners(img, patternSize, corners, ret)
            # cv2.imshow('Image', img)
            # cv2.waitKey(0)

    cv2.destroyAllWindows()

    result, camera_matrix, distortion_coefficients, rotation_vectors, translation_vectors = cv2.calibrateCamera(
        object_points,
        image_points,
        gray.shape[::-1],
        None,
        None
    )

    mean_error = calc_mean_error(
        object_points,
        image_points,
        rotation_vectors,
        translation_vectors,
        camera_matrix,
        distortion_coefficients
    )

    print("mean error: ", mean_error)
    save_calibration_coefficients(camera_matrix, distortion_coefficients)

    # create_window("Undistorted Image")
    # create_window("Original Image")
    #
    # for fileName in images:
    #     img = cv2.imread(fileName)
    #     h, w = img.shape[:2]
    #     newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    #     dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    #     x, y, w, h = roi
    #     dst = dst[y: y + h, x: x + w]
    #     cv2.imshow("Undistorted Image", dst)
    #     cv2.imshow("Original Image", img)
    #     cv2.waitKey(0)
