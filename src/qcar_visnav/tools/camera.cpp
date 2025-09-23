#include <cassert>
#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <vector>
#include <regex>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <bitset>
#include <cstdio>

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

#include "to_string.hpp"
#include "camera.h"

// ---------------- Small path helpers (C++14, no filesystem) ----------------
static std::string parentDir(const std::string& p) {
    std::size_t pos = p.find_last_of("/\\");
    if (pos == std::string::npos) return std::string(".");
    if (pos == 0) return std::string("/");
    return p.substr(0, pos);
}
static std::string filenameOnly(const std::string& p) {
    std::size_t pos = p.find_last_of("/\\");
    return (pos == std::string::npos) ? p : p.substr(pos+1);
}
static std::string stemOf(const std::string& fname) {
    std::size_t pos = fname.find_last_of('.');
    return (pos == std::string::npos) ? fname : fname.substr(0, pos);
}
static inline double clampd(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// ==================== Pose ====================
Pose::Pose()
    : rotationMatrix(cv::Matx33d::eye())
    , translationVector(cv::Vec3d(0,0,0))   // OpenCV 3.x friendly
{}

Pose::Pose(const cv::Mat & rvec, const cv::Mat & tvec)
{
    cv::Rodrigues(rvec, rotationMatrix);
    translationVector = cv::Vec3d(tvec.at<double>(0,0),
                                  tvec.at<double>(1,0),
                                  tvec.at<double>(2,0));
}

Pose Pose::operator*(const Pose & other) const
{
    Pose result;
    result.rotationMatrix = rotationMatrix * other.rotationMatrix;
    result.translationVector = rotationMatrix * other.translationVector + translationVector;
    return result;
}

cv::Vec3d Pose::operator*(const cv::Vec3d & r) const
{
    return rotationMatrix * r + translationVector;
}

Pose Pose::inverse() const
{
    Pose result;
    result.rotationMatrix = rotationMatrix.t();
    result.translationVector = -result.rotationMatrix * translationVector;
    return result;
}

// ==================== Chessboard ====================
void Chessboard::write(cv::FileStorage & fs) const
{
    fs << "{"
       << "grid_width"  << boardSize.width
       << "grid_height" << boardSize.height
       << "square_size" << squareSize
       << "}";
}

void Chessboard::read(const cv::FileNode & node)
{
    node["grid_width"]  >> boardSize.width;
    node["grid_height"] >> boardSize.height;
    node["square_size"] >> squareSize;
}

std::vector<cv::Point3f> Chessboard::gridPoints() const
{
    std::vector<cv::Point3f> rPNn_all;
    rPNn_all.reserve(boardSize.height*boardSize.width);
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            rPNn_all.push_back(cv::Point3f(j*squareSize, i*squareSize, 0));
    return rPNn_all;
}

std::ostream & operator<<(std::ostream & os, const Chessboard & cb)
{
    return os << "boardSize: " << cb.boardSize << ", squareSize: " << cb.squareSize;
}

// ==================== ChessboardImage ====================
ChessboardImage::ChessboardImage(const cv::Mat & image_, const Chessboard & chessboard, const std::string & filename_)
    : image(image_), filename(filename_), isFound(false)
{
    if (image.empty()) { corners.clear(); return; }

    cv::Mat gray;
    if (image.channels() == 3 || image.channels() == 4) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else gray = image.clone();

    std::vector<cv::Point2f> detected;
    const int findFlags = cv::CALIB_CB_ADAPTIVE_THRESH |
                          cv::CALIB_CB_NORMALIZE_IMAGE |
                          cv::CALIB_CB_FAST_CHECK;

    isFound = cv::findChessboardCorners(gray, chessboard.boardSize, detected, findFlags);

    if (isFound) {
        const cv::Size  winSize(11, 11);
        const cv::Size  zeroZone(-1, -1);
        const cv::TermCriteria term(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 1e-3);

        cv::Mat gray8;
        if (gray.type() != CV_8U) gray.convertTo(gray8, CV_8U);
        else gray8 = gray;

        cv::cornerSubPix(gray8, detected, winSize, zeroZone, term);
        corners = std::move(detected);
    } else {
        corners.clear();
    }
}

void ChessboardImage::drawCorners(const Chessboard & chessboard)
{
    cv::drawChessboardCorners(image, chessboard.boardSize, corners, isFound);
}

void ChessboardImage::drawBox(const Chessboard & chessboard, const Camera & camera)
{
    if (!isFound) return;

    const Pose Tnb = camera.cameraToBody(Tnc);
    const Pose Tcn = Tnc.inverse();

    const double W = (chessboard.boardSize.width  - 1) * chessboard.squareSize;
    const double H = (chessboard.boardSize.height - 1) * chessboard.squareSize;

    const cv::Vec3d p0(0.0, 0.0, 0.0);
    const cv::Vec3d p1(W  , 0.0, 0.0);
    const cv::Vec3d p2(W  , H  , 0.0);
    const cv::Vec3d p3(0.0, H  , 0.0);

    const double Zmag = 0.23;
    const cv::Vec3d c = 0.5 * (p0 + p2);
    const double z_up   = (Tcn * (c + cv::Vec3d(0,0, Zmag)))[2];
    const double z_down = (Tcn * (c + cv::Vec3d(0,0,-Zmag)))[2];
    const double Z = (z_up < z_down) ? Zmag : -Zmag;

    const cv::Vec3d q0 = p0 + cv::Vec3d(0,0,Z);
    const cv::Vec3d q1 = p1 + cv::Vec3d(0,0,Z);
    const cv::Vec3d q2 = p2 + cv::Vec3d(0,0,Z);
    const cv::Vec3d q3 = p3 + cv::Vec3d(0,0,Z);

    const cv::Scalar BLUE (255, 0, 0);
    const cv::Scalar RED  (0, 0, 255);
    const cv::Scalar GREEN(0, 255, 0);

    auto draw_edge = [&](const cv::Vec3d& a, const cv::Vec3d& b, const cv::Scalar& color, int thickness)
    {
        const int    kSegs   = 64;
        const double maxJump = 60.0;
        cv::Point prev; bool havePrev = false;

        for (int i = 0; i <= kSegs; ++i) {
            const double t = static_cast<double>(i) / kSegs;
            const cv::Vec3d P = a*(1.0 - t) + b*t;
            if (!camera.isWorldWithinFOV(P, Tnb)) { havePrev = false; continue; }
            const cv::Vec2d pix = camera.worldToPixel(P, Tnb);
            const cv::Point cur(cvRound(pix[0]), cvRound(pix[1]));
            if (!havePrev) { prev = cur; havePrev = true; continue; }
            if (cv::norm(cur - prev) > maxJump) { prev = cur; continue; }
            cv::line(image, prev, cur, color, thickness, cv::LINE_AA);
            prev = cur;
        }
    };

    // Base
    draw_edge(p0, p1, BLUE, 2); draw_edge(p1, p2, BLUE, 2);
    draw_edge(p2, p3, BLUE, 2); draw_edge(p3, p0, BLUE, 2);
    // Sides
    draw_edge(p0, q0, RED, 2);  draw_edge(p1, q1, RED, 2);
    draw_edge(p2, q2, RED, 2);  draw_edge(p3, q3, RED, 2);
    // Lid
    draw_edge(q0, q1, GREEN, 2); draw_edge(q1, q2, GREEN, 2);
    draw_edge(q2, q3, GREEN, 2); draw_edge(q3, q0, GREEN, 2);
}

void ChessboardImage::recoverPose(const Chessboard & chessboard, const Camera & camera)
{
    std::vector<cv::Point3f> rPNn_all = chessboard.gridPoints();
    cv::Mat rvec, tvec;
    cv::solvePnP(rPNn_all, corners, camera.cameraMatrix, camera.distCoeffs, rvec, tvec);
    Pose Tcn(rvec, tvec);
    Tnc = Tcn.inverse();
}

// ==================== ChessboardData ====================
ChessboardData::ChessboardData(const std::string & configPath)
{
    // config exists?
    cv::FileStorage fs(configPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open config file: " + configPath);
    }

    // Read chessboard + regex
    cv::FileNode node = fs["chessboard_data"];
    node["chessboard"] >> chessboard;

    std::string pattern;
    node["file_regex"] >> pattern;
    fs.release();

    std::regex re(pattern, std::regex_constants::basic | std::regex_constants::icase);

    const std::string root = parentDir(configPath);
    std::cout << "Scanning directory " << root << " for file pattern \"" << pattern << "\"\n";

    // Gather candidate files recursively with cv::glob (C++14-friendly)
    std::vector<cv::String> files;
    auto add_glob = [&](const std::string& pat){
        std::vector<cv::String> tmp; cv::glob(pat, tmp, true);
        files.insert(files.end(), tmp.begin(), tmp.end());
    };
    add_glob(root + "/*.jpg");  add_glob(root + "/*.jpeg"); add_glob(root + "/*.png");
    add_glob(root + "/*.bmp");  add_glob(root + "/*.tif");  add_glob(root + "/*.tiff");
    add_glob(root + "/*.mp4");  add_glob(root + "/*.avi");  add_glob(root + "/*.mov"); add_glob(root + "/*.mkv");

    chessboardImages.clear();

    for (const auto& p_cv : files)
    {
        const std::string p = static_cast<std::string>(p_cv);
        const std::string fname = filenameOnly(p);
        if (!std::regex_match(fname, re)) continue;

        std::cout << "Loading " << fname << "...";
        cv::Mat image = cv::imread(p, cv::IMREAD_COLOR);

        if (!image.empty()) {
            std::cout << " image, detecting...";
            ChessboardImage ci(image, chessboard, fname);
            std::cout << (ci.isFound ? " found\n" : " not found\n");
            if (ci.isFound) chessboardImages.push_back(ci);
            continue;
        }

        // Try as video
        cv::VideoCapture cap(p);
        if (!cap.isOpened()) { std::cout << " not image/video, skipping\n"; continue; }

        int nFrames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
        if (nFrames <= 0) {
            nFrames = 0; cv::Mat tmp; while (cap.read(tmp)) ++nFrames; cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        }
        std::cout << " video, " << nFrames << " frames\n";
        if (nFrames <= 0) { std::cout << " no frames, skip\n"; continue; }

        const int target = 50;
        const int stride = std::max(1, nFrames / target);
        int kept = 0;

        for (int idxFrame = 0; idxFrame < nFrames; idxFrame += stride)
        {
            std::cout << "Reading " << fname << " frame " << idxFrame << "...";
            cap.set(cv::CAP_PROP_POS_FRAMES, idxFrame);
            cv::Mat frame;
            if (!cap.read(frame)) { std::cout << " EOF\n"; break; }

            std::cout << " detecting...";
            char buf[256];
            std::snprintf(buf, sizeof(buf), "%s_%05d.jpg", stemOf(fname).c_str(), idxFrame);
            ChessboardImage ci(frame, chessboard, std::string(buf));
            std::cout << (ci.isFound ? " found\n" : " not found\n");
            if (ci.isFound) {
                chessboardImages.push_back(ci);
                if (++kept >= target) { std::cout << " reached target of " << target << ", stopping\n"; break; }
            }
        }
    }
}

// ==================== ChessboardData helpers ====================
void ChessboardData::drawCorners()
{
    for (auto & ci : chessboardImages) ci.drawCorners(chessboard);
}
void ChessboardData::drawBoxes(const Camera & camera)
{
    for (auto & ci : chessboardImages) ci.drawBox(chessboard, camera);
}
void ChessboardData::recoverPoses(const Camera & camera)
{
    for (auto & ci : chessboardImages) ci.recoverPose(chessboard, camera);
}

// ==================== Camera ====================
void Camera::calibrate(ChessboardData & chessboardData)
{
    std::vector<cv::Point3f> rPNn_all = chessboardData.chessboard.gridPoints();

    std::vector<std::vector<cv::Point2f>> rQOi_all;
    rQOi_all.reserve(chessboardData.chessboardImages.size());
    for (const auto & ci : chessboardData.chessboardImages) rQOi_all.push_back(ci.corners);
    assert(!rQOi_all.empty());

    imageSize = chessboardData.chessboardImages[0].image.size();

    // OpenCV 3.x-safe flags
    flags = cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL | cv::CALIB_TILTED_MODEL;


    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs   = cv::Mat::zeros(12, 1, CV_64F);

    std::vector<std::vector<cv::Point3f>> objectPoints(rQOi_all.size(), rPNn_all);
    std::vector<cv::Mat> rvecs, tvecs;

    std::cout << "Calibrating camera...";
    double rms = cv::calibrateCamera(objectPoints, rQOi_all, imageSize,
                                     cameraMatrix, distCoeffs, rvecs, tvecs, flags);
    std::cout << " done\n";

    calcFieldOfView();

    // Save per-image extrinsics (camera pose in world)
    for (std::size_t k = 0; k < chessboardData.chessboardImages.size(); ++k)
    {
        Pose Tcn(rvecs[k], tvecs[k]);
        chessboardData.chessboardImages[k].Tnc = Tcn.inverse();
    }

    printCalibration();
    std::cout << std::setw(30) << std::right << "RMS reprojection error: " << rms << "\n";

    assert(cv::checkRange(cameraMatrix));
    assert(cv::checkRange(distCoeffs));
}

void Camera::printCalibration() const
{
    std::bitset<8*sizeof(flags)> bitflag(flags);
    std::cout << "\nCalibration data:\n";
    std::cout << std::setw(30) << std::right << "Bit flags: " << bitflag.to_string() << "\n";
    std::cout << std::setw(30) << std::right << "cameraMatrix:\n" << cameraMatrix << "\n";
    std::cout << std::setw(30) << std::right << "distCoeffs:\n"    << distCoeffs.t() << "\n";
    std::cout << std::setw(30) << std::right << "Focal lengths: "
              << "(" << cameraMatrix.at<double>(0,0) << ", " << cameraMatrix.at<double>(1,1) << ")\n";
    std::cout << std::setw(30) << std::right << "Principal point: "
              << "(" << cameraMatrix.at<double>(0,2) << ", " << cameraMatrix.at<double>(1,2) << ")\n";
    std::cout << std::setw(30) << std::right << "Field of view (horizontal): "
              << 180.0/CV_PI*hFOV << " deg\n";
    std::cout << std::setw(30) << std::right << "Field of view (vertical): "
              << 180.0/CV_PI*vFOV << " deg\n";
    std::cout << std::setw(30) << std::right << "Field of view (diagonal): "
              << 180.0/CV_PI*dFOV << " deg\n";
}

void Camera::calcFieldOfView()
{
    assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 && cameraMatrix.type() == CV_64F);

    const double W = static_cast<double>(imageSize.width);
    const double H = static_cast<double>(imageSize.height);
    const double midx = (W - 1.0) * 0.5;
    const double midy = (H - 1.0) * 0.5;

    auto angle_between = [](const cv::Vec3d& a, const cv::Vec3d& b) {
        double c = a.dot(b) / (cv::norm(a) * cv::norm(b));
        c = clampd(c, -1.0, 1.0);
        return std::acos(c);
    };

    cv::Vec3d uLeft  = pixelToVector(cv::Vec2d(0.0,   midy));
    cv::Vec3d uRight = pixelToVector(cv::Vec2d(W-1.0, midy));
    hFOV = angle_between(uLeft, uRight);

    cv::Vec3d uTop    = pixelToVector(cv::Vec2d(midx, 0.0));
    cv::Vec3d uBottom = pixelToVector(cv::Vec2d(midx, H-1.0));
    vFOV = angle_between(uTop, uBottom);

    cv::Vec3d uTL = pixelToVector(cv::Vec2d(0.0,   0.0));
    cv::Vec3d uBR = pixelToVector(cv::Vec2d(W-1.0, H-1.0));
    dFOV = angle_between(uTL, uBR);

    // Build per-azimuth cosine limits (with margin)
    cosThetaLimit_.assign(360, -1.0);
    for (int deg = 0; deg < 360; ++deg) {
        double maxTheta = 0.0;
        double radAz = deg * CV_PI / 180.0;

        for (int rstep = 0; rstep < 2; ++rstep) {
            double r = 0.499 - 0.002 * rstep;
            double u = (W - 1) * (0.5 + r * std::cos(radAz));
            double v = (H - 1) * (0.5 + r * std::sin(radAz));
            cv::Vec3d uPCc = pixelToVector(cv::Vec2d(u, v));
            double theta = std::acos(clampd(uPCc[2] / cv::norm(uPCc), -1.0, 1.0));
            if (theta > maxTheta) maxTheta = theta;
        }
        cosThetaLimit_[deg] = std::cos(maxTheta + (10.0 * CV_PI / 180.0));
    }
}

Pose Camera::cameraToBody(const Pose & Tnc) const { return Tnc * Tbc.inverse(); }
Pose Camera::bodyToCamera(const Pose & Tnb) const { return Tnb * Tbc; }

cv::Vec3d Camera::worldToVector(const cv::Vec3d & rPNn, const Pose & Tnb) const
{
    Pose Tnc = bodyToCamera(Tnb);
    Pose Tcn = Tnc.inverse();
    cv::Vec3d rPCc = Tcn * rPNn;
    return rPCc / cv::norm(rPCc);
}

cv::Vec2d Camera::worldToPixel(const cv::Vec3d & rPNn, const Pose & Tnb) const
{
    return vectorToPixel(worldToVector(rPNn, Tnb));
}

cv::Vec2d Camera::vectorToPixel(const cv::Vec3d & rPCc) const
{
    cv::Point3f P(static_cast<float>(rPCc[0]/rPCc[2]),
                  static_cast<float>(rPCc[1]/rPCc[2]),
                  1.0f);
    std::vector<cv::Point3f> obj(1, P);
    std::vector<cv::Point2f> img;

    cv::Mat rvec = cv::Mat::zeros(3,1,CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3,1,CV_64F);

    cv::projectPoints(obj, rvec, tvec, cameraMatrix, distCoeffs, img);
    return cv::Vec2d(img[0].x, img[0].y);
}

cv::Vec3d Camera::pixelToVector(const cv::Vec2d & rQOi) const
{
    std::vector<cv::Point2f> distorted; distorted.emplace_back((float)rQOi[0], (float)rQOi[1]);
    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(distorted, undistorted, cameraMatrix, distCoeffs);
    cv::Vec3d rPCc(undistorted[0].x, undistorted[0].y, 1.0);
    return rPCc / cv::norm(rPCc);
}

bool Camera::isVectorWithinFOV(const cv::Vec3d & rPCc) const
{
    if (!std::isfinite(rPCc[0]) || !std::isfinite(rPCc[1]) || !std::isfinite(rPCc[2])) return false;
    if (rPCc[2] <= 1e-9) return false;

    cv::Vec3d dir = rPCc / cv::norm(rPCc);
    const cv::Vec3d z(0,0,1);

    double az = std::atan2(dir[1], dir[0]) * 180.0 / CV_PI;
    int bin = static_cast<int>(std::lround(az));
    bin = (bin % 360 + 360) % 360;

    double cosang = dir.dot(z);
    if (cosang < cosThetaLimit_[bin]) return false;

    cv::Vec3d P(rPCc[0]/rPCc[2], rPCc[1]/rPCc[2], 1.0);
    cv::Vec2d px = vectorToPixel(P);
    if (!std::isfinite(px[0]) || !std::isfinite(px[1])) return false;

    return (px[0] >= 0.0 && px[0] < imageSize.width &&
            px[1] >= 0.0 && px[1] < imageSize.height);
}

bool Camera::isWorldWithinFOV(const cv::Vec3d & rPNn, const Pose & Tnb) const
{
    return isVectorWithinFOV(worldToVector(rPNn, Tnb));
}

void Camera::write(cv::FileStorage & fs) const
{
    fs << "{"
       << "camera_matrix"           << cameraMatrix
       << "distortion_coefficients" << distCoeffs
       << "flags"                   << flags
       << "imageSize"               << imageSize
       << "}";
}

void Camera::read(const cv::FileNode & node)
{
    node["camera_matrix"]           >> cameraMatrix;
    node["distortion_coefficients"] >> distCoeffs;
    node["flags"]                   >> flags;
    node["imageSize"]               >> imageSize;

    calcFieldOfView();

    assert(cameraMatrix.cols == 3);
    assert(cameraMatrix.rows == 3);
    assert(cameraMatrix.type() == CV_64F);
    assert(distCoeffs.cols == 1);
    assert(distCoeffs.type() == CV_64F);
}
