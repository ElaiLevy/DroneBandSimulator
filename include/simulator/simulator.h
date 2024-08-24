#ifndef ORB_SLAM2_SIMULATOR_H
#define ORB_SLAM2_SIMULATOR_H

#include <memory>
#include <Eigen/SVD>
#include <filesystem>
#include <functional>
#include <pangolin/gl/glsl.h>
#include <pangolin/pangolin.h>
#include <pangolin/gl/glvbo.h>
#include <pangolin/display/display.h>
#include <pangolin/utils/file_utils.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/display/pangolin_gl.h>
#include <pangolin/geometry/glgeometry.h>

#include "ORBextractor.h"
#include "System.h"

#include "RunModel/TextureShader.h"
#include "Auxiliary.h"
#include "RoomExit/RoomExit.h" // Include RoomExit header

class Simulator
{
public:
    Simulator(std::string ORBSLAMConfigFile, std::string model_path, bool alignModelToTexture, std::string modelTextureNameToAlignTo,
              Eigen::Vector3f startingPoint, bool saveMap = false, std::string simulatorOutputDirPath = "../slamMaps/", bool loadMap = false,
              std::string mapLoadPath = "../slamMaps/example.bin", double movementFactor = 0.01, double speedFactor = 1.0,
              std::string vocPath = "../Vocabulary/ORBvoc.txt");

    std::thread run();
    bool isReady() { return ready; }
    bool startScanning() { return start; }
    cv::Mat getCurrentLocationSlam();
    Eigen::Matrix4d getCurrentLocation();
    std::vector<ORB_SLAM2::MapPoint *> getCurrentMap() { return SLAM->GetMap()->GetAllMapPoints(); };
    void command(const std::string &command, int intervalUsleep = 50000, double fps = 30.0, int totalCommandTimeInSeconds = 1);
    void stop() { stopFlag = true; }
    void setTrack(bool value);
    void setSpeed(double speed);
    double getSpeed() const;
    void simulatorRunThread();
    void drawPoint(cv::Point3d point, float size, Eigen::Vector3d color);
    void cleanPoints();

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
    std::string vocPath;
    std::string ORBSLAMConfigFile;
    std::string mapLoadPath;
    bool loadMap;
    pangolin::OpenGlRenderState s_cam;
    pangolin::OpenGlRenderState s_cam2; // Second camera for the second drone
    Eigen::Matrix3d K;
    std::shared_ptr<ORB_SLAM2::ORBextractor> orbExtractor;
    std::string simulatorOutputDir;
    bool stopFlag;
    bool stopFlagSLAM;
    bool ready;
    bool start;
    bool initSlam;

    std::vector<std::pair<cv::Point3d, std::pair<float, Eigen::Vector3d>>> points;

    Eigen::Vector3f startingPoint;
    bool saveMapSignal;
    bool track;
    double movementFactor{};
    std::string modelPath;
    bool alignModelToTexture;
    std::string modelTextureNameToAlignTo;
    bool isSaveMap;
    bool isInitalized;
    bool isLocalized;
    bool cull_backfaces;
    pangolin::GlSlProgram program;
    pangolin::GlGeometry geomToRender;
    Eigen::Vector2i viewportDesiredSize;
    cv::Mat Tcw;
    cv::Mat currentImg;
    std::mutex locationLock;
    std::mutex imgLock;
    int numberOfFeatures;
    int trackingNumberOfFeatures;
    double speedFactor;

    std::shared_ptr<RoomExit> roomExit; // RoomExit object

    void SLAMThread();
    bool feedSLAM(cv::Mat &img);
    void extractSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo,
                        Eigen::MatrixXf &surface);
    void alignModelViewPointToSurface(const pangolin::Geometry &modelGeometry, std::string modelTextureNameToAlignTo);
    void saveMap(std::string prefix = "");
    void intervalOverCommand(const std::function<void(pangolin::OpenGlRenderState &, double &)> &func,
                             double value, int intervalUsleep, double fps, int totalCommandTimeInSeconds);
    void applyCommand(std::string &command, double value, int intervalUsleep, double fps, int totalCommandTimeInSeconds);
    void static applyForwardToModelCam(pangolin::OpenGlRenderState &cam, double value);
    void static applyRightToModelCam(pangolin::OpenGlRenderState &cam, double value);
    void static applyYawRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);
    void static applyUpModelCam(pangolin::OpenGlRenderState &cam, double value);
    void static applyPitchRotationToModelCam(pangolin::OpenGlRenderState &cam, double value);
    void slower();
    void faster();

    void navigateToExit();
    void commandDroneTowardsPoint(pangolin::OpenGlRenderState &cam, const Eigen::Vector3d &point, double movementFactor);
};

#endif // ORB_SLAM2_SIMULATOR_H
