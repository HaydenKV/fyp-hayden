#include <iostream>
#include <string>
#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui.hpp>
#include "camera.h"

static std::string parentDir(const std::string& p) {
    std::size_t pos = p.find_last_of("/\\");
    if (pos == std::string::npos) return std::string(".");
    if (pos == 0) return std::string("/");
    return p.substr(0, pos);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: calib_tool <path/to/chessboard_config.yaml> [--draw]\n";
        return 1;
    }
    const std::string configPath = argv[1];
    const bool draw = (argc >= 3 && std::string(argv[2]) == std::string("--draw"));

    try {
        ChessboardData data(configPath);

        Camera cam;
        cam.calibrate(data);

        const std::string outPath = parentDir(configPath) + "/camera.xml";
        cv::FileStorage fs(outPath, cv::FileStorage::WRITE);
        fs << "camera" << cam;
        fs.release();
        std::cout << "Wrote calibration to: " << outPath << "\n";

        if (draw) {
            data.drawBoxes(cam);
            for (const auto& ci : data.chessboardImages) {
                cv::imshow("calib preview (ESC to quit)", ci.image);
                int c = cv::waitKey(0);
                if (c == 27) break;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "calib_tool error: " << e.what() << "\n";
        return 2;
    }
    return 0;
}
