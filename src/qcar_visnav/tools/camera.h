#ifndef CAMERA_H
#define CAMERA_H

#include <vector>
#include <string>
#include <ostream>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include "serialisation.hpp"

// ==================== Pose ====================
struct Pose
{
    cv::Matx33d rotationMatrix;   // R in SO(3)
    cv::Vec3d   translationVector;// r in R^3

    Pose();                       // R=I, r=0
    Pose(const cv::Mat & rvec, const cv::Mat & tvec); // from OpenCV rvec/tvec

    Pose operator*(const Pose & other) const;   // SE(3) group op
    cv::Vec3d operator*(const cv::Vec3d & r) const; // act on points

    Pose inverse() const;          // inverse in SE(3)
};

// ==================== Chessboard ====================
struct Chessboard
{
    cv::Size boardSize;
    float    squareSize;

    void write(cv::FileStorage & fs) const;
    void read (const cv::FileNode & node);

    std::vector<cv::Point3f> gridPoints() const;
    friend std::ostream & operator<<(std::ostream &, const Chessboard &);
};

struct Camera;

// ==================== ChessboardImage ====================
struct ChessboardImage
{
    ChessboardImage(const cv::Mat &, const Chessboard &, const std::string & filename = std::string());
    cv::Mat image;
    std::string filename;
    Pose Tnc;                                   // camera pose in world
    std::vector<cv::Point2f> corners;           // detected corners
    bool isFound{false};

    void drawCorners(const Chessboard &);
    void drawBox(const Chessboard &, const Camera &);
    void recoverPose(const Chessboard &, const Camera &);
};

// ==================== ChessboardData ====================
struct ChessboardData
{
    explicit ChessboardData(const std::string &configPath); // load & scan

    Chessboard chessboard;
    std::vector<ChessboardImage> chessboardImages;

    void drawCorners();
    void drawBoxes(const Camera &);
    void recoverPoses(const Camera &);
};

// ==================== Camera ====================
struct Camera
{
    void calibrate(ChessboardData &);
    void printCalibration() const;

    Pose cameraToBody(const Pose & Tnc) const;
    Pose bodyToCamera(const Pose & Tnb) const;

    cv::Vec3d worldToVector(const cv::Vec3d & rPNn, const Pose & Tnb) const;
    cv::Vec2d worldToPixel (const cv::Vec3d &, const Pose &) const;
    cv::Vec2d vectorToPixel(const cv::Vec3d &) const;
    cv::Vec3d pixelToVector(const cv::Vec2d &) const;

    bool isWorldWithinFOV (const cv::Vec3d & rPNn, const Pose & Tnb) const;
    bool isVectorWithinFOV(const cv::Vec3d & rPCc) const;

    void write(cv::FileStorage &) const;
    void read (const cv::FileNode &);

    void calcFieldOfView();

    // Intrinsics
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    int     flags = 0;
    cv::Size imageSize;

    // Camera-in-body pose (unused in calib; set externally if needed)
    Pose Tbc;

    // FOV + per-azimuth limits
    double hFOV = 0.0, vFOV = 0.0, dFOV = 0.0;
    std::vector<double> cosThetaLimit_;
};

#endif
