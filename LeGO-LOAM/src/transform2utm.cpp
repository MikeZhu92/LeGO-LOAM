//
// Created by mikezhu on 6/24/19.
//
#include "utility.h"

#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


class Transform2UTM {
private:
    ros::NodeHandle nh;

    ros::Subscriber novatelSub;
    ros::Subscriber legoSub;

    ros::Publisher local2utmPub;

    bool isTransformed;
    double gpsTime;
    double legoTime;

    Eigen::Affine3d geoPose;
    Eigen::Affine3d legoPoseUTM;

    Eigen::Affine3d trans1;
    Eigen::Affine3d local2UTM;

public:
    Transform2UTM()
    : nh("~"),
      isTransformed(false),
      gpsTime(.0)
    {
        trans1 = Eigen::Translation3d(0,0,0) *
                 Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068);
        novatelSub = nh.subscribe<nav_msgs::Odometry>("/navsat/odom", 1,
            &Transform2UTM::utmInfoHandler, this);
        legoSub = nh.subscribe<nav_msgs::Odometry>("/integrated_to_init", 1,
            &Transform2UTM::legoInfoHandler, this);
        local2utmPub = nh.advertise<geometry_msgs::PoseStamped>("/lego_utm", 1);
    }

    ~Transform2UTM(){};

private:
    void utmInfoHandler(const nav_msgs::Odometry::ConstPtr &utmMsg)
    {
        tf::poseMsgToEigen(utmMsg->pose.pose, geoPose);
        gpsTime = utmMsg->header.stamp.toSec();
    }

    void legoInfoHandler(const nav_msgs::Odometry::ConstPtr &legoMsg)
    {
        legoTime = legoMsg->header.stamp.toSec();
        geometry_msgs::Vector3 lidarVelo = legoMsg->twist.twist.linear;
        Eigen::Affine3d legoPose;
        tf::poseMsgToEigen(legoMsg->pose.pose, legoPose);
        if (std::fabs(lidarVelo.x) < 0.01 &&
            std::fabs(lidarVelo.y) < 0.01 &&
            std::fabs(gpsTime - legoTime) < 0.2)
        {
            computeTransform(legoPose);
        }
        if (isTransformed) {
            publishLegoOdom(legoMsg, legoPose);
        }
    }

    inline void computeTransform(Eigen::Affine3d &inputPose)
    {
        legoPoseUTM = trans1 * inputPose;
        local2UTM = geoPose * legoPoseUTM.inverse();
        isTransformed = true;
    }

    void publishLegoOdom(const nav_msgs::Odometry::ConstPtr &legoMsg,
                         Eigen::Affine3d &legoPose)
    {
        legoPoseUTM = local2UTM * trans1 * legoPose;
        geometry_msgs::PoseStamped newPose;
        newPose.header = legoMsg->header;
        tf::poseEigenToMsg(legoPoseUTM, newPose.pose);
        local2utmPub.publish(newPose);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");
    Transform2UTM Lego2UTM;
    ROS_INFO("\033[1;32m---->\033[0m Coordinate Transformer Started.");
    ros::spin();
    return 0;
}