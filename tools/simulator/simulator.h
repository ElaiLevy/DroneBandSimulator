#ifndef ORB_SLAM2_SIMULATOR_H
#define ORB_SLAM2_SIMULATOR_H

#include <memory>
#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/gl/glsl.h>
#include <pangolin/gl/glvbo.h>
#include <functional>
#include <pangolin/display/display.h>
#include <pangolin/display/pangolin_gl.h>
#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/glgeometry.h>
#include "ORBextractor.h"
#include "System.h"
#include <Eigen/SVD>
#include <filesystem>
#include "include/run_model/TextureShader.h"

/**
 *  @class Simulator
 *  @brief This class provides a simulation environment for virtual robotic navigation and mapping.
 *
 *  The Simulator class integrates ORBSLAM2 and a 3D blender model to create a comprehensive testing bed
 *  for SLAM (Simultaneous Localization and Mapping) and navigation algorithms. It negates the need for
 *  a physical robot interface.
 *
 *  With this simulator and a 3D blender model, a robot's movement can be simulated in a variety of virtual environments.
 *  It uses the Pangolin library to capture the current viewer image from the display window, which is then
 *  sent to ORBSLAM2. This approach allows users to observe, analyze and improve the efficiency of their implemented
 *  algorithms in real-time with expansive and flexible datasets.
 *
 *  Features include:
 *  - Virtual robotic navigation in 3D environments using A,S,D,W,E,Q,R,F to move in the model
 *  - On-the-fly ORBSLAM2 map generation and navigation from the 3D model, and extraction of current location and full map.
 *  - Real-time visualization using Pangolin
 */
class Simulator
{
public:
    Simulator(std::string ORBSLAMConfigFile, std::string model_path, bool alignModelToTexture, std::string modelTextureNameToAlignTo,
              bool trackImages = true, bool saveMap = false, std::string simulatorOutputDirPath = "../slamMaps/", bool loadMap = false,
              std::string mapLoadPath = "../slamMaps/example.bin",
              double movementFactor = 0.01,
              double speedFactor = 1.0,
              std::string vocPath = "../Vocabulary/ORBvoc.txt");

    std::thread run();

    bool isReady() { return ready; }

    bool startScanning() { return start; }

    cv::Mat getCurrentLocation();
    cv::Mat getCurrentLocationWithOffset();

    std::vector<ORB_SLAM2::MapPoint *> getCurrentMap() { return SLAM->GetMap()->GetAllMapPoints(); }

    void command(const std::string &command, int intervalUsleep = 50000,
                 double fps = 30.0,
                 int totalCommandTimeInSeconds = 1);

    void stop() { stopFlag = true; }

    void setTrack(bool value) { track = value; }

    void setSpeed(double speed);

    double getSpeed() const;

    void simulatorRunThread();

    std::shared_ptr<ORB_SLAM2::System> GetSLAM() { return SLAM; }

private:
    std::unordered_map<std::string, bool> commandMap = {
            {"cw", true},
            {"ccw", true},
            {"forward", true},
            {"back", true},
            {"right", true},
            {"up", true},
            {"down", true},
            {"left", true},
            {"flip", false},
            {"rc", false}};
    std::shared_ptr<ORB_SLAM2::System> SLAM;
    pangolin::OpenGlRenderState s_cam;
    pangolin::OpenGlRenderState s_cam2;
    Eigen::Matrix3d K;
    std::shared_ptr<ORB_SLAM2::ORBextractor> orbExtractor;
    std::string simulatorOutputDir;
    bool stopFlag;
    bool stopFlagSLAM;
    bool ready;
    bool start;

    bool saveMapSignal;
    bool track;
    double movementFactor{};
    std::string modelPath;
    bool alignModelToTexture;
    std::string modelTextureNameToAlignTo;
    std::vector<Eigen::Vector3d> Picks_w;
    bool isSaveMap;
    bool isInitalized;
    bool isLocalized;
    bool trackImages;
    bool cull_backfaces;
    pangolin::GlSlProgram program;
    pangolin::GlGeometry geomToRender;
    Eigen::Vector2i viewportDesiredSize;
    cv::Mat Tcw;
    cv::Mat Tcw2;
    cv::Mat currentImg;
    std::mutex locationLock;
    std::mutex imgLock;
    int numberOfFeatures;
    int trackingNumberOfFeatures;
    double speedFactor;

    void SLAMThread();

    bool feedSLAM(cv::Mat &img);

    void extractSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo,
                        Eigen::MatrixXf &surface);

    void alignModelViewPointToSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo);

    void saveMap(std::string prefix = "");

    void intervalOverCommand(const std::function<void(pangolin::OpenGlRenderState &, double &)> &func,
                             double value, int intervalUsleep,
                             double fps,
                             int totalCommandTimeInSeconds);

    void applyCommand(std::string &command, double value,
                      int intervalUsleep,
                      double fps,
                      int totalCommandTimeInSeconds);

    void applyCommands();

    void static applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyUpModelCam(pangolin::OpenGlRenderState &cam, double value);

    void static applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);

    void slower();

    void faster();

    void RenderScene(pangolin::OpenGlRenderState &cam, pangolin::GlGeometry &geom);

    // New functions added for navigation
    void navigateToExit(const Eigen::Vector3d &exitPoint, double speed, int intervalUsleep, double fps);
    bool isAtExit(const Eigen::Vector3d &exitPoint);
};

#endif // ORB_SLAM2_SIMULATOR_H








































































































































































































































