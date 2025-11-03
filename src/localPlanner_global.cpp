#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std;

const double PI = 3.1415926;
#define PLOTPATHSET 1   // keep free path visualization

// ---------------- Parameters (kept) ----------------
string pathFolder;
double vehicleLength = 0.4;     // 40cm quad
double vehicleWidth  = 0.4;     // 40cm quad
double sensorOffsetX = 0.0;
double sensorOffsetY = 0.0;
bool   twoWayDrive   = false;

double laserVoxelSize = 0.05;
bool   checkObstacle  = true;
bool   checkRotObstacle = false;

double adjacentRange = 3.5;     // planning horizon (m)
int    pointPerPathThre = 2;

double maxSpeed = 1.0;          // not used for joystick anymore, kept for compatibility
double dirWeight = 0.02;        // direction preference weight
double dirThre   = 90.0;        // allowable dir window
bool   dirToVehicle = false;    // same semantics as original

double pathScale    = 1.0;      // scale lattice
double minPathScale = 0.75;
double pathScaleStep = 0.25;

double minPathRange  = 1.0;
double pathRangeStep = 0.5;
bool   pathCropByGoal = true;

double goalCloseDis   = 0.2;
double goalClearRange = 0.2;
double goalX = 0.0, goalY = 0.0;

// ---------------- Constants from original ----------------
const int pathNum = 343;
const int groupNum = 7;
float gridVoxelSize = 0.02f;
float searchRadius  = 0.55f;
float gridVoxelOffsetX = 3.2f;
float gridVoxelOffsetY = 4.5f;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

// ---------------- Clouds & data ----------------
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[1];

pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
#endif

int   pathList[pathNum] = {0};
float endDirPathList[pathNum] = {0};
int   clearPathList[36 * pathNum] = {0};
float pathPenaltyList[36 * pathNum] = {0};   // kept to minimize code changes (unused terrain cost = 0)
float clearPathPerGroupScore[36 * groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;

double odomTime = 0.0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter;
rclcpp::Node::SharedPtr nh;

//laser_geometry::LaserProjector projector;

// ---------------- Handlers ----------------
void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  odomTime = rclcpp::Time(odom->header.stamp).seconds();
  double roll, pitch, yaw;
  const auto& q = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;

  // offset lidar vs base if needed
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;
}

void goalHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
}

void laserScanHandler(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
  laserCloud->clear();
  laserCloudCrop->clear();

  double angle = scan->angle_min;
  for (size_t i = 0; i < scan->ranges.size(); i++, angle += scan->angle_increment) {
    float r = scan->ranges[i];
    if (!std::isfinite(r) || r < 0.05f || r > adjacentRange) continue;

    pcl::PointXYZI pt;
    pt.x = r * cos(angle);
    pt.y = r * sin(angle);
    pt.z = 0.0;
    pt.intensity = 1.0;
    laserCloudCrop->push_back(pt);
  }

  laserCloudDwz->clear();
  laserDwzFilter.setInputCloud(laserCloudCrop);
  laserDwzFilter.filter(*laserCloudDwz);

  newLaserCloud = true;
}

// ---------------- File Readers (unchanged logic) ----------------
int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum = 0;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) { RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit."); exit(1); }
    strLast = strCur;
    strCur = string(str);
    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) { RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit."); exit(1); }
    }
  }
  return pointNum;
}

void readStartPaths()
{
  string fileName = pathFolder + "/startPaths.ply";
  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) { RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit."); exit(1); }

  int pointNum = readPlyHeader(filePtr);
  pcl::PointXYZ point; int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);
    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) { RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit."); exit(1); }
    if (groupID >= 0 && groupID < groupNum) startPaths[groupID]->push_back(point);
  }
  fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths()
{
  string fileName = pathFolder + "/paths.ply";
  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) { RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit."); exit(1); }
  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 30, pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);
    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) { RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit."); exit(1); }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }
  fclose(filePtr);
}
#endif

void readPathList()
{
  string fileName = pathFolder + "/pathList.ply";
  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) { RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit."); exit(1); }

  if (pathNum != readPlyHeader(filePtr)) { RCLCPP_INFO(nh->get_logger(), "Incorrect path number, exit."); exit(1); }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);
    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) { RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit."); exit(1); }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0f * atan2(endY, endX) * 180.0f / PI;
    }
  }
  fclose(filePtr);
}

void readCorrespondences()
{
  string fileName = pathFolder + "/correspondences.txt";
  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) { RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit."); exit(1); }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) { RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit."); exit(1); }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) { RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit."); exit(1); }
      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else break;
    }
  }
  fclose(filePtr);
}

// ---------------- Main ----------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("localPlanner");

  // Parameters (trimmed: no joystick/terrain)
  nh->declare_parameter<std::string>("pathFolder", pathFolder);
  nh->declare_parameter<double>("vehicleLength", vehicleLength);
  nh->declare_parameter<double>("vehicleWidth", vehicleWidth);
  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("laserVoxelSize", laserVoxelSize);
  nh->declare_parameter<bool>("checkObstacle", checkObstacle);
  nh->declare_parameter<bool>("checkRotObstacle", checkRotObstacle);
  nh->declare_parameter<double>("adjacentRange", adjacentRange);
  nh->declare_parameter<int>("pointPerPathThre", pointPerPathThre);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("dirWeight", dirWeight);
  nh->declare_parameter<double>("dirThre", dirThre);
  nh->declare_parameter<bool>("dirToVehicle", dirToVehicle);
  nh->declare_parameter<double>("pathScale", pathScale);
  nh->declare_parameter<double>("minPathScale", minPathScale);
  nh->declare_parameter<double>("pathScaleStep", pathScaleStep);
  nh->declare_parameter<double>("minPathRange", minPathRange);
  nh->declare_parameter<double>("pathRangeStep", pathRangeStep);
  nh->declare_parameter<bool>("pathCropByGoal", pathCropByGoal);
  nh->declare_parameter<double>("goalCloseDis", goalCloseDis);
  nh->declare_parameter<double>("goalClearRange", goalClearRange);
  nh->declare_parameter<double>("goalX", goalX);
  nh->declare_parameter<double>("goalY", goalY);

  nh->get_parameter("pathFolder", pathFolder);
  nh->get_parameter("vehicleLength", vehicleLength);
  nh->get_parameter("vehicleWidth", vehicleWidth);
  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("laserVoxelSize", laserVoxelSize);
  nh->get_parameter("checkObstacle", checkObstacle);
  nh->get_parameter("checkRotObstacle", checkRotObstacle);
  nh->get_parameter("adjacentRange", adjacentRange);
  nh->get_parameter("pointPerPathThre", pointPerPathThre);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("dirWeight", dirWeight);
  nh->get_parameter("dirThre", dirThre);
  nh->get_parameter("dirToVehicle", dirToVehicle);
  nh->get_parameter("pathScale", pathScale);
  nh->get_parameter("minPathScale", minPathScale);
  nh->get_parameter("pathScaleStep", pathScaleStep);
  nh->get_parameter("minPathRange", minPathRange);
  nh->get_parameter("pathRangeStep", pathRangeStep);
  nh->get_parameter("pathCropByGoal", pathCropByGoal);
  nh->get_parameter("goalCloseDis", goalCloseDis);
  nh->get_parameter("goalClearRange", goalClearRange);
  nh->get_parameter("goalX", goalX);
  nh->get_parameter("goalY", goalY);

  // Subs & pubs
  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("/odom", 5, odometryHandler);
  auto subLaser    = nh->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), laserScanHandler);
  auto subGoal     = nh->create_subscription<geometry_msgs::msg::PointStamped>("/way_point", 5, goalHandler);

  auto pubPath = nh->create_publisher<nav_msgs::msg::Path>("/path", 5);
  nav_msgs::msg::Path path;

#if PLOTPATHSET == 1
  auto pubFreePaths = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", 2);
#endif

  RCLCPP_INFO(nh->get_logger(), "Reading path files…");

  // init containers
  laserCloudStack[0].reset(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < groupNum; i++) startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
#if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++) paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
#endif
  for (int i = 0; i < gridVoxelNum; i++) correspondences[i].resize(0);

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);

  // read path assets
  readStartPaths();
#if PLOTPATHSET == 1
  readPaths();
#endif
  readPathList();
  readCorrespondences();

  RCLCPP_INFO(nh->get_logger(), "Initialization complete.");

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (newLaserCloud) {
      newLaserCloud = false;

      // stack (single scan kept for compatibility)
      laserCloudStack[0]->clear();
      *laserCloudStack[0] = *laserCloudDwz;

      // planner cloud is the stacked laser
      plannerCloud->clear();
      *plannerCloud += *laserCloudStack[0];

      // Points are already in base_link; just crop by range and z=0
      plannerCloudCrop->clear();
      int plannerCloudSize = plannerCloud->points.size();
      for (int i = 0; i < plannerCloudSize; i++) {
        const auto &pt = plannerCloud->points[i];
        float dis = sqrt(pt.x * pt.x + pt.y * pt.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(pt);
        }
      }

      // ---- Goal direction (deg) in base_link ----
      float relativeGoalDis = adjacentRange;
      float joyDir = 0.0f;  // use goal-based preferred direction; if no goal given, 0° (forward)
      {
        float relX = ((goalX - vehicleX) * cos(vehicleYaw) + (goalY - vehicleY) * sin(vehicleYaw));
        float relY = (-(goalX - vehicleX) * sin(vehicleYaw) + (goalY - vehicleY) * cos(vehicleYaw));
        relativeGoalDis = sqrt(relX * relX + relY * relY);
        if (relativeGoalDis > 1e-3) joyDir = atan2(relY, relX) * 180.0 / PI;
      }

      bool pathFound = false;
      float defPathScale = static_cast<float>(pathScale);
      float curPathScale = defPathScale;
      float pathRange = adjacentRange;

      while (curPathScale >= minPathScale && pathRange >= minPathRange) {
        // reset accumulators
        for (int i = 0; i < 36 * pathNum; i++) {
          clearPathList[i] = 0;
          pathPenaltyList[i] = 0.0f;
        }
        for (int i = 0; i < 36 * groupNum; i++) {
          clearPathPerGroupScore[i] = 0.0f;
        }

        float minObsAngCW = -180.0f;
        float minObsAngCCW = 180.0f;

        float diameter = sqrtf((vehicleLength/2.0f) * (vehicleLength/2.0f) + (vehicleWidth/2.0f)*(vehicleWidth/2.0f));
        float angOffset = atan2f(vehicleWidth, vehicleLength) * 180.0f / PI;

        // Accumulate voxel correspondences against paths for each rotation bin
        int pcN = plannerCloudCrop->points.size();
        for (int i = 0; i < pcN; i++) {
          float x = plannerCloudCrop->points[i].x / curPathScale;
          float y = plannerCloudCrop->points[i].y / curPathScale;
          float h = 1.0f; // intensity dummy (no terrain)

          float dis = sqrtf(x * x + y * y);
          if (dis < pathRange / curPathScale && (dis <= (relativeGoalDis + goalClearRange) / curPathScale || !pathCropByGoal) && checkObstacle) {
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              float rotAng = (10.0f * rotDir - 180.0f) * PI / 180.0f;
              float angDiff = fabsf(joyDir - (10.0f * rotDir - 180.0f));
              if (angDiff > 180.0f) angDiff = 360.0f - angDiff;

              if ((angDiff > dirThre && !dirToVehicle) ||
                  (fabsf(10.0f * rotDir - 180.0f) > dirThre && fabsf(joyDir) <= 90.0f && dirToVehicle) ||
                  ((10.0f * rotDir > dirThre && 360.0f - 10.0f * rotDir > dirThre) && fabsf(joyDir) > 90.0f && dirToVehicle)) {
                continue;
              }

              float x2 = cosf(rotAng) * x + sinf(rotAng) * y;
              float y2 = -sinf(rotAng) * x + cosf(rotAng) * y;

              float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;
              int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
              int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
              if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                int ind = gridVoxelNumY * indX + indY;
                int blockedPathByVoxelNum = correspondences[ind].size();
                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                  // without terrain, treat as obstacle hits
                  clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                }
              }
            }
          }

          // rotation blocking cone (optional)
          if (dis < diameter / curPathScale && checkRotObstacle) {
            float angObs = atan2f(y, x) * 180.0f / PI;
            if (angObs > 0) {
              if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
              if (minObsAngCW < angObs + angOffset - 180.0f) minObsAngCW = angObs + angOffset - 180.0f;
            } else {
              if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
              if (minObsAngCCW > 180.0f + angObs - angOffset) minObsAngCCW = 180.0f + angObs - angOffset;
            }
          }
        }

        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;

        // Score per rotation & group
        for (int i = 0; i < 36 * pathNum; i++) {
          int rotDir = int(i / pathNum);
          float angDiff = fabsf(joyDir - (10.0f * rotDir - 180.0f));
          if (angDiff > 180.0f) angDiff = 360.0f - angDiff;

          if ((angDiff > dirThre && !dirToVehicle) ||
              (fabsf(10.0f * rotDir - 180.0f) > dirThre && fabsf(joyDir) <= 90.0f && dirToVehicle) ||
              ((10.0f * rotDir > dirThre && 360.0f - 10.0f * rotDir > dirThre) && fabsf(joyDir) > 90.0f && dirToVehicle)) {
            continue;
          }

          if (clearPathList[i] < pointPerPathThre) {
            // no terrain penalty; keep interface
            float penaltyScore = 1.0f;
            float dirDiff = fabsf(joyDir - endDirPathList[i % pathNum] - (10.0f * rotDir - 180.0f));
            if (dirDiff > 360.0f) dirDiff -= 360.0f;
            if (dirDiff > 180.0f) dirDiff = 360.0f - dirDiff;

            float rotDirW = (rotDir < 18) ? fabsf(fabsf(rotDir - 9) + 1) : fabsf(fabsf(rotDir - 27) + 1);
            float groupDirW = 4 - fabsf(pathList[i % pathNum] - 3);
            float score = (1 - sqrtf(sqrtf(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;
            if (relativeGoalDis < goalCloseDis) score = (1 - sqrtf(sqrtf(dirWeight * dirDiff))) * groupDirW * groupDirW * penaltyScore;
            if (score > 0) {
              clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
            }
          }
        }

        // Pick best rotation+group within rotation constraints
        float maxScore = 0.0f;
        int selectedGroupID = -1;
        for (int i = 0; i < 36 * groupNum; i++) {
          int rotDir = int(i / groupNum);
          float rotAng = (10.0f * rotDir - 180.0f) * PI / 180.0f;
          float rotDeg = 10.0f * rotDir; if (rotDeg > 180.0f) rotDeg -= 360.0f;

          if (maxScore < clearPathPerGroupScore[i] &&
              ((rotAng * 180.0f / PI > minObsAngCW && rotAng * 180.0f / PI < minObsAngCCW) ||
               (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) ||
               !checkRotObstacle)) {
            maxScore = clearPathPerGroupScore[i];
            selectedGroupID = i;
          }
        }

        if (selectedGroupID >= 0) {
          int rotDir = int(selectedGroupID / groupNum);
          float rotAng = (10.0f * rotDir - 180.0f) * PI / 180.0f;

          selectedGroupID = selectedGroupID % groupNum;
          int selectedPathLength = startPaths[selectedGroupID]->points.size();
          path.poses.resize(selectedPathLength);
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrtf(x * x + y * y);

            if (dis <= pathRange / curPathScale && dis <= relativeGoalDis / curPathScale) {
              path.poses[i].pose.position.x = curPathScale * (cosf(rotAng) * x - sinf(rotAng) * y);
              path.poses[i].pose.position.y = curPathScale * (sinf(rotAng) * x + cosf(rotAng) * y);
              path.poses[i].pose.position.z = curPathScale * z;
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
          path.header.frame_id = "base_link";
          pubPath->publish(path);

#if PLOTPATHSET == 1
          // visualize all free paths for the chosen rotation
          freePaths->clear();
          for (int i = 0; i < 36 * pathNum; i++) {
            int rdir = int(i / pathNum);
            if (rdir != rotDir) continue; // only the winning rotation for clarity
            float rotAngV = (10.0f * rdir - 180.0f) * PI / 180.0f;
            float rotDeg = 10.0f * rdir; if (rotDeg > 180.0f) rotDeg -= 360.0f;

            float angDiff = fabsf(joyDir - (10.0f * rdir - 180.0f));
            if (angDiff > 180.0f) angDiff = 360.0f - angDiff;
            if ((angDiff > dirThre && !dirToVehicle) ||
                (fabsf(10.0f * rdir - 180.0f) > dirThre && fabsf(joyDir) <= 90.0f && dirToVehicle) ||
                ((10.0f * rdir > dirThre && 360.0f - 10.0f * rdir > dirThre) && fabsf(joyDir) > 90.0f && dirToVehicle)) {
              continue;
            }
            if (clearPathList[i] < pointPerPathThre) {
              int freePathLength = paths[i % pathNum]->points.size();
              for (int j = 0; j < freePathLength; j++) {
                pcl::PointXYZI point = paths[i % pathNum]->points[j];
                float x = point.x, y = point.y, z = point.z;
                float dis = sqrtf(x * x + y * y);
                if (dis <= pathRange / curPathScale && (dis <= (relativeGoalDis + goalClearRange) / curPathScale || !pathCropByGoal)) {
                  point.x = curPathScale * (cosf(rotAngV) * x - sinf(rotAngV) * y);
                  point.y = curPathScale * (sinf(rotAngV) * x + cosf(rotAngV) * y);
                  point.z = curPathScale * z;
                  point.intensity = 1.0f;
                  freePaths->push_back(point);
                }
              }
            }
          }
          sensor_msgs::msg::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths, freePaths2);
          freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
          freePaths2.header.frame_id = "base_link";
          pubFreePaths->publish(freePaths2);
#endif
          pathFound = true;
        }

        if (!pathFound) {
          if (curPathScale >= minPathScale + pathScaleStep) {
            curPathScale -= pathScaleStep;
            pathRange = adjacentRange * curPathScale / defPathScale;
          } else {
            pathRange -= pathRangeStep;
          }
        } else {
          break;
        }
      } // while search scales

      // If nothing found, publish a zero-length path at base_link
      if (!pathFound) {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;
        path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        path.header.frame_id = "base_link";
        pubPath->publish(path);

#if PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::msg::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        freePaths2.header.frame_id = "base_link";
        pubFreePaths->publish(freePaths2);
#endif
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}

