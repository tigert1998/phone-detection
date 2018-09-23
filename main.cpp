#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <functional>
#include <algorithm>

using cv::Mat;
using cv::Point;
using std::vector;
using std::string;
using boost::optional;
using boost::format;

optional<string> calibration_configuration_file_name = boost::none;
std::function<bool()> is_calibrated = [] () -> bool { return calibration_configuration_file_name != boost::none; };
Mat camera_matrix, distortion_coefficients;
bool contour_detected = false;
cv::VideoCapture capture;

Mat DealFrame(const Mat &);

void ShowFrame(string title, const Mat &frame, cv::Size size = cv::Size(1280, 720)) {
    Mat resized;
    cv::resize(frame, resized, size);
    imshow(title, resized);
}

void Init(int argc, char **argv) {
    namespace po = boost::program_options;
    namespace fs = boost::filesystem;
    po::options_description desc;
    desc.add_options()
        ("help", "produce help message")
        ("index", po::value<int>(), "set camera index")
        ("config", po::value<string>(), "set calibration configuration file path")
        ("pic", po::value<string>(), "set static picture as input");
    try {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
        if (vm.count("help")) {
            std::cout << desc << std::endl;
            exit(1);
        }
        if (vm.count("pic")) {
            string pic_path = vm["pic"].as<string>();
            Mat image = cv::imread(pic_path);
            if (image.empty()) {
                std::cout << "image path isn't setted properly" << std::endl;
                exit(1);
            }
            ShowFrame("Detection", DealFrame(image));
            cv::waitKey(0);
            exit(0);
        }
        if (vm.count("index")) {
            int camera_index = vm["index"].as<int>();
            capture.open(camera_index);
        } else {
            capture.open(0);
        }
        if (!capture.grab()) {
            std::cout << "camera index isn't setted properly" << std::endl;
            exit(1);
        }
        if (vm.count("config")) {
            calibration_configuration_file_name = vm["config"].as<string>();
            if (!fs::exists(calibration_configuration_file_name.value())) {
                std::cout << "calibration configuration file path isn't setted properly" << std::endl;
                exit(1);
            }
        }
    } catch (...) {
        std::cout << desc << std::endl;
        exit(1);
    }
    if (!is_calibrated()) return;
    cv::FileStorage file_storage(calibration_configuration_file_name.value(), cv::FileStorage::Mode::READ);
    file_storage["camera_matrix"] >> camera_matrix;
    file_storage["distortion_coefficients"] >> distortion_coefficients;
    file_storage.release();
}

double ConvexArea(const vector<Point> &convex) {
    int area = 0;
    for (int i = 0; i < convex.size(); i++) {
        int j = (i + 1) % convex.size();
        area += convex[i].cross(convex[j]);
    }
    return abs(static_cast<double>(area) / 2);
}

Mat Undistort(const Mat &frame) {
    if (!is_calibrated()) return frame;
    Mat result(frame.size(), CV_8UC3, cv::Scalar::all(255)), temp;
    cv::Rect roi;
    auto new_camera_matrix = cv::getOptimalNewCameraMatrix(
        camera_matrix,
        distortion_coefficients,
        frame.size(),
        1,
        frame.size(),
        &roi
    );
    cv::undistort(frame, temp, camera_matrix, distortion_coefficients, new_camera_matrix);
    Mat mask(frame.size(), CV_8UC1, cv::Scalar::all(0));
    mask(roi).setTo(cv::Scalar::all(255));
    temp.copyTo(result, mask);
    return result;
}

Mat DealFrame(const Mat &frame) {
    using std::max;
    using std::min;
    Mat ans = Undistort(frame);
    ShowFrame("Original Frame", ans);
    vector<vector<Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cvtColor(ans, ans, cv::COLOR_BGR2GRAY);
    auto kernal = getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    int frame_area = frame.size().width * frame.size().height;
    dilate(ans, ans, kernal);

    threshold(ans, ans, 120, 255, cv::THRESH_BINARY);
    findContours(ans, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    contour_detected = contours.size() >= 1;
    cvtColor(ans, ans, cv::COLOR_GRAY2BGR);
    if (!contour_detected) {
        std::cout << "Contour not detected" << std::endl;
        return ans;
    }
    ans.setTo(cv::Scalar::all(255));
    {
        vector<vector<Point>> new_contours;
        std::transform(contours.begin(), contours.end(), std::back_inserter(new_contours), [] (const std::vector<Point> &contour) {
            static vector<Point> hull;
            convexHull(contour, hull);
            return hull;
        });
        contours = new_contours;
        new_contours.clear();
        std::copy_if(contours.begin(), contours.end(), std::back_inserter(new_contours), [&] (const std::vector<Point> &contour) {
            static constexpr double minimum = 0.1, maximum = 0.5;
            double area = ConvexArea(contour);
            bool res = minimum * frame_area <= area && area <= maximum * frame_area;
            return res;
        });
        contours = new_contours;
    }
    for (const vector<Point> &contour: contours) {
        int min_x = INT_MAX, min_y = INT_MAX, max_x = 0, max_y = 0;
        polylines(ans, contour, true, cv::Scalar(255, 0, 0));
        for (const Point &point: contour) {
            min_x = min(min_x, point.x);
            max_x = max(max_x, point.x);
            min_y = min(min_y, point.y);
            max_y = max(max_y, point.y);
        }
        cv::rectangle(ans, Point(min_x, min_y), Point(max_x, max_y), cv::Scalar(0, 255, 0));
        cv::drawMarker(ans, Point((min_x + max_x) / 2, (min_y + max_y) / 2), cv::Scalar(0, 0, 255));
    }
    std::cerr << format("[contours.size() = %d]") % contours.size() << std::endl;
    return ans;
}

int main(int argc, char **argv) {
    Init(argc, argv);
    while (true) {
        Mat frame;
        capture.read(frame);
        ShowFrame("Detection", DealFrame(frame));
        auto key = cv::waitKey(10);
    }
    return 0;
}
