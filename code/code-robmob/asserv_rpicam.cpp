// asserv_rpicam.cpp — Suivi de couleur par camera avec OpenCV
//
// Compilation :
//   g++ -std=c++17 -O2 -Wall -Wextra asserv_rpicam.cpp -o asserv_rpicam $(pkg-config --cflags --libs opencv4)
//
// Usage :
//   ./asserv_rpicam
//   LIBGL_ALWAYS_SOFTWARE=1 ./asserv_rpicam 
//   Clic gauche dans la fenetre camera pour choisir la couleur cible.
//   Appuyer sur q ou Echap pour quitter.

#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <mutex>
#include <optional>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

static constexpr const char* WINDOW_FRAME = "camera";
static constexpr const char* WINDOW_MASK  = "mask";
static constexpr int    H_TOL    = 10;
static constexpr int    S_TOL    = 60;
static constexpr int    V_TOL    = 60;
static constexpr double MIN_AREA = 150.0;

struct TrackerParams {
    cv::Vec3b hsv_target;
};

struct UiState {
    std::optional<cv::Point> click;
    std::optional<TrackerParams> tracker;
};

static std::mutex g_state_mutex;
static UiState    g_state;

static uint8_t clamp_u8(int value) {
    return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

static cv::Mat make_mask_from_hsv(const cv::Mat& hsv, cv::Vec3b target) {
    int h = target[0];
    int s = target[1];
    int v = target[2];

    double low_s  = clamp_u8(s - S_TOL);
    double high_s = clamp_u8(s + S_TOL);
    double low_v  = clamp_u8(v - V_TOL);
    double high_v = clamp_u8(v + V_TOL);

    cv::Mat mask;

    // Teinte OpenCV en HSV: [0, 179]. On gere le wrap-around pres de 0/179.
    if (h - H_TOL < 0) {
        cv::Mat mask1, mask2;
        cv::inRange(hsv,
                    cv::Scalar(0, low_s, low_v),
                    cv::Scalar(h + H_TOL, high_s, high_v),
                    mask1);
        cv::inRange(hsv,
                    cv::Scalar(180 + (h - H_TOL), low_s, low_v),
                    cv::Scalar(179, high_s, high_v),
                    mask2);
        cv::bitwise_or(mask1, mask2, mask);
    } else if (h + H_TOL > 179) {
        cv::Mat mask1, mask2;
        cv::inRange(hsv,
                    cv::Scalar(h - H_TOL, low_s, low_v),
                    cv::Scalar(179, high_s, high_v),
                    mask1);
        cv::inRange(hsv,
                    cv::Scalar(0, low_s, low_v),
                    cv::Scalar((h + H_TOL) - 180, high_s, high_v),
                    mask2);
        cv::bitwise_or(mask1, mask2, mask);
    } else {
        cv::inRange(hsv,
                    cv::Scalar(h - H_TOL, low_s, low_v),
                    cv::Scalar(h + H_TOL, high_s, high_v),
                    mask);
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::Mat opened;
    cv::morphologyEx(mask, opened, cv::MORPH_OPEN, kernel);

    return opened;
}

static void on_mouse(int event, int x, int y, int /*flags*/, void* /*userdata*/) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        std::lock_guard<std::mutex> lock(g_state_mutex);
        g_state.click = cv::Point(x, y);
    }
}

int main() {
    // Forcer le rendu logiciel pour eviter les erreurs GLX via ssh -Y.
    setenv("LIBGL_ALWAYS_SOFTWARE", "1", 0);

    cv::VideoCapture cam("/dev/video0", cv::CAP_V4L2);
    if (!cam.isOpened()) {
        std::fprintf(stderr, "Impossible d'ouvrir la camera (index 0).\n");
        return 1;
    }

    cv::namedWindow(WINDOW_FRAME, cv::WINDOW_NORMAL);
    cv::namedWindow(WINDOW_MASK, cv::WINDOW_NORMAL);
    cv::setMouseCallback(WINDOW_FRAME, on_mouse);

    std::printf("Clic gauche dans la fenetre camera pour choisir la couleur cible.\n");
    std::printf("Appuyer sur q ou Echap pour quitter.\n");

    cv::Mat frame, hsv;

    for (;;) {
        cam.read(frame);
        if (frame.empty()) {
            continue;
        }

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Recuperer un eventuel clic en attente.
        std::optional<cv::Point> pending_click;
        {
            std::lock_guard<std::mutex> lock(g_state_mutex);
            pending_click = g_state.click;
            g_state.click.reset();
        }

        if (pending_click) {
            cv::Point p = *pending_click;
            if (p.x >= 0 && p.y >= 0 && p.x < hsv.cols && p.y < hsv.rows) {
                cv::Vec3b pixel = hsv.at<cv::Vec3b>(p.y, p.x);
                {
                    std::lock_guard<std::mutex> lock(g_state_mutex);
                    g_state.tracker = TrackerParams{pixel};
                }
                std::printf("Couleur calibree en (%d, %d): HSV=(%d, %d, %d)\n",
                            p.x, p.y, pixel[0], pixel[1], pixel[2]);
            }
        }

        // Lire le tracker courant.
        std::optional<TrackerParams> tracker;
        {
            std::lock_guard<std::mutex> lock(g_state_mutex);
            tracker = g_state.tracker;
        }

        if (tracker) {
            cv::Mat mask = make_mask_from_hsv(hsv, tracker->hsv_target);

            cv::Moments moments = cv::moments(mask, true);
            if (moments.m00 > MIN_AREA) {
                int cx = static_cast<int>(moments.m10 / moments.m00);
                int cy = static_cast<int>(moments.m01 / moments.m00);

                cv::circle(frame, cv::Point(cx, cy), 8,
                           cv::Scalar(0, 255, 0), 2, cv::LINE_8);

                char buf[64];
                std::snprintf(buf, sizeof(buf), "Barycentre: (%d, %d)", cx, cy);
                cv::putText(frame, buf, cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            }

            cv::imshow(WINDOW_MASK, mask);
        } else {
            cv::putText(frame, "Cliquez sur la couleur cible", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

            cv::Mat empty = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
            cv::imshow(WINDOW_MASK, empty);
        }

        cv::imshow(WINDOW_FRAME, frame);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q') {
            break;
        }
    }

    return 0;
}
