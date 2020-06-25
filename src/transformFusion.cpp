// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "utility.h"

class TransformFusion{

private:

    ros::NodeHandle nh;

    ros::Publisher pubLaserOdometry2;
    ros::Publisher pubReOdometry;
    ros::Publisher pubOdometryPlane;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subOdomAftMapped;
  

    nav_msgs::Odometry laserOdometry2;
    nav_msgs::Odometry loamOdometry;
    nav_msgs::Odometry PlaneOdometry;
    tf::StampedTransform laserOdometryTrans2;
    tf::TransformBroadcaster tfBroadcaster2;

    tf::StampedTransform laserOdometryTrans3;
    tf::TransformBroadcaster tfBroadcaster3;

    tf::StampedTransform AftMappedTrans;
    tf::TransformBroadcaster tfBroadcasterAftMapped;
    
    float transformSum[6];
    float transformIncre[6];
    float transformMapped[6];
    float transformBefMapped[6];
    float transformAftMapped[6];

    double OdometryPrevPose_x;
    double OdometryPrevPose_y;
    double PlaneOdometryYaw;
    double PlaneOdometryX;
    double PlaneOdometryY;
    double PosePrevX, PosePrevY, PosePrevZ, PosePrevPrevX, PosePrevPrevY, PosePrevPrevZ;
    int CountForPlaneOdom;

    std_msgs::Header currentHeader;

public:

    TransformFusion(): PlaneOdometryX(0), PlaneOdometryY(0), PlaneOdometryYaw(0), 
                       OdometryPrevPose_x(0), OdometryPrevPose_y(0), CountForPlaneOdom(0),
                       PosePrevX(0), PosePrevY(0), PosePrevZ(0), PosePrevPrevX(0), PosePrevPrevY(0), PosePrevPrevZ(0)
    {

        pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/integrated_to_init", 5);
        pubReOdometry = nh.advertise<nav_msgs::Odometry> ("/Odometry/loam", 5);
        pubOdometryPlane = nh.advertise<nav_msgs::Odometry> ("/Odometry/plane_loam", 5);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &TransformFusion::laserOdometryHandler, this);
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 5, &TransformFusion::odomAftMappedHandler, this);

        laserOdometry2.header.frame_id = "/odom_loam";
        laserOdometry2.child_frame_id = "/camera";

        laserOdometryTrans2.frame_id_ = "/odom_loam";
        laserOdometryTrans2.child_frame_id_ = "/camera";

        AftMappedTrans.frame_id_ = "/odom_loam";
        AftMappedTrans.child_frame_id_ = "/camera";

        laserOdometryTrans3.frame_id_ = "/odom_loam_";
        laserOdometryTrans3.child_frame_id_ = "/base_loam_";
        

        for (int i = 0; i < 6; ++i)
        {
            transformSum[i] = 0;
            transformIncre[i] = 0;
            transformMapped[i] = 0;
            transformBefMapped[i] = 0;
            transformAftMapped[i] = 0;
        }
        
    }

    void transformAssociateToMap()
    {
        float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
        float y1 = transformBefMapped[4] - transformSum[4];
        float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

        float x2 = x1;
        float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
        float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

        transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
        transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
        transformIncre[5] = z2;

        float sbcx = sin(transformSum[0]);
        float cbcx = cos(transformSum[0]);
        float sbcy = sin(transformSum[1]);
        float cbcy = cos(transformSum[1]);
        float sbcz = sin(transformSum[2]);
        float cbcz = cos(transformSum[2]);

        float sblx = sin(transformBefMapped[0]);
        float cblx = cos(transformBefMapped[0]);
        float sbly = sin(transformBefMapped[1]);
        float cbly = cos(transformBefMapped[1]);
        float sblz = sin(transformBefMapped[2]);
        float cblz = cos(transformBefMapped[2]);

        float salx = sin(transformAftMapped[0]);
        float calx = cos(transformAftMapped[0]);
        float saly = sin(transformAftMapped[1]);
        float caly = cos(transformAftMapped[1]);
        float salz = sin(transformAftMapped[2]);
        float calz = cos(transformAftMapped[2]);

        float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                  - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                  - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                  - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
                  - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
        transformMapped[0] = -asin(srx);

        float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                     - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                     - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                     + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                     + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
        float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                     - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                     + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                     + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                     - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                     + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
        transformMapped[1] = atan2(srycrx / cos(transformMapped[0]), 
                                   crycrx / cos(transformMapped[0]));
        
        float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]), 
                                   crzcrx / cos(transformMapped[0]));

        x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
        y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
        z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

        transformMapped[3] = transformAftMapped[3] 
                           - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
        transformMapped[4] = transformAftMapped[4] - y2;
        transformMapped[5] = transformAftMapped[5] 
                           - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
    {
        currentHeader = laserOdometry->header;

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        transformSum[0] = -pitch;
        transformSum[1] = -yaw;
        transformSum[2] = roll;

        transformSum[3] = laserOdometry->pose.pose.position.x;
        transformSum[4] = laserOdometry->pose.pose.position.y;
        transformSum[5] = laserOdometry->pose.pose.position.z;

        transformAssociateToMap();

        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                  (transformMapped[2], -transformMapped[0], -transformMapped[1]);

        laserOdometry2.header.stamp = laserOdometry->header.stamp;
        laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
        laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
        laserOdometry2.pose.pose.orientation.z = geoQuat.x;
        laserOdometry2.pose.pose.orientation.w = geoQuat.w;
        laserOdometry2.pose.pose.position.x = transformMapped[3];
        laserOdometry2.pose.pose.position.y = transformMapped[4];
        laserOdometry2.pose.pose.position.z = transformMapped[5];

        pubLaserOdometry2.publish(laserOdometry2);

        laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
        laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
        tfBroadcaster2.sendTransform(laserOdometryTrans2);

    }
    
    void PlaneOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
    {
        // geometry_msgs::PoseStamped converted_pose;
        // tf::Transform transform_camera_to_base;
        // tf::Transform transform_map_to_odom;

        // transform_camera_to_base.setOrigin(tf::Vector3(0, 0, 0));
        // transform_camera_to_base.setRotation(tf::Quaternion(-0.5, -0.5, -0.5, 0.5)); //camera to base_loam
        // transform_map_to_odom.setOrigin(tf::Vector3(0, 0, 0));
        // transform_map_to_odom.setRotation(tf::Quaternion(0.500, 0.500, 0.500, 0.500)); // map to odom_loam

        // tf::poseTFToMsg(transform_map_to_odom *AftMappedTrans*transform_camera_to_base , converted_pose.pose); //laserOdometryTrans2 --> AftMappedTrans : laserOdometry loop rate is too fast. 
        // std::cout << converted_pose.pose << std::endl;
        // if (!std::isnan(converted_pose.pose.position.x) && !std::isinf(converted_pose.pose.position.x))
        // {
        //     loamOdometry.header  = laserOdometry2.header;
        //     loamOdometry.header.frame_id = "odom_loam_";
        //     loamOdometry.child_frame_id = "base_odom_";
        //     loamOdometry.pose.pose = converted_pose.pose;

        //     pubReOdometry.publish(loamOdometry);

        //     laserOdometryTrans3.stamp_ = odomAftMapped->header.stamp;
        //     laserOdometryTrans3.setRotation(tf::Quaternion(converted_pose.pose.orientation.x, converted_pose.pose.orientation.y, converted_pose.pose.orientation.z, converted_pose.pose.orientation.w));
        //     laserOdometryTrans3.setOrigin(tf::Vector3(converted_pose.pose.position.x, converted_pose.pose.position.y, converted_pose.pose.position.z));
        //     tfBroadcaster3.sendTransform(laserOdometryTrans3);
            
        //     //Inner Product
        //     double UnitCurDist, UnitPrevDist;
        //     double ProdInner, angle, DistDelta, angle_sign;
        //     tf::Vector3 CrossProd;
        //     geometry_msgs::Vector3 geoVect;
            

        //     auto UnitCur = tf::Vector3(converted_pose.pose.position.x - PosePrevPrevX, 
        //                                converted_pose.pose.position.y - PosePrevPrevY, 
        //                                converted_pose.pose.position.z - PosePrevPrevZ);
        //     auto UnitPrev = tf::Vector3(PosePrevX - PosePrevPrevX, 
        //                                 PosePrevY - PosePrevPrevY,
        //                                 PosePrevZ - PosePrevPrevZ);
            
        //     UnitCurDist = tf::tfDistance(tf::Vector3(converted_pose.pose.position.x,converted_pose.pose.position.y,converted_pose.pose.position.z), 
        //                                 tf::Vector3(PosePrevPrevX,PosePrevPrevY,PosePrevPrevZ));
        //     UnitPrevDist = tf::tfDistance(tf::Vector3(PosePrevX,PosePrevY,PosePrevZ), 
        //                                 tf::Vector3(PosePrevPrevX,PosePrevPrevY,PosePrevPrevZ));

        //     DistDelta = sqrt(pow(converted_pose.pose.position.x - PosePrevX, 2) + 
        //                     pow(converted_pose.pose.position.y - PosePrevY, 2) +  
        //                     pow(converted_pose.pose.position.z - PosePrevZ, 2));

        //     if (CountForPlaneOdom > 3 )
        //     {
        //         ProdInner = tf::tfDot(UnitCur, UnitPrev);
        //         if (UnitCurDist != 0 && UnitPrevDist != 0)
        //         {
        //             angle = acos(ProdInner/ (UnitCurDist * UnitPrevDist));    
        //         }
        //         CrossProd = tf::tfCross(UnitCur, UnitPrev);
        //         vector3TFToMsg(CrossProd, geoVect);
        //         if (geoVect.z > 0)
        //         { angle_sign = 1 ;}  
        //         else
        //         { angle_sign = -1;}
                
        //     }                         

        //     PosePrevPrevX = PosePrevX;
        //     PosePrevPrevY = PosePrevY;
        //     PosePrevPrevZ = PosePrevZ;
        //     PosePrevX = converted_pose.pose.position.x;
        //     PosePrevY = converted_pose.pose.position.y;
        //     PosePrevZ = converted_pose.pose.position.z;

        //     std::cout << "Prod: " << ProdInner << ", UnitCur: "<< UnitCurDist << ", UnitPrev: " <<  UnitPrevDist<< std::endl;
        //     std::cout << "cross product : \n" << geoVect <<  "\n Count: "<< CountForPlaneOdom << std::endl;
        //     //Pub Plane Odometry
        //     PlaneOdometryYaw = PlaneOdometryYaw + angle * angle_sign;
        //     if (PlaneOdometryYaw > M_PI * 2)
        //     {
        //         PlaneOdometryYaw = PlaneOdometryYaw - 2 * M_PI;
        //     }
        //     else if (PlaneOdometryYaw < -M_PI * 2)
        //     {
        //         PlaneOdometryYaw = PlaneOdometryYaw + 2 * M_PI;
        //     }        
        //     PlaneOdometryX = PlaneOdometryX +  DistDelta * cos(PlaneOdometryYaw);
        //     PlaneOdometryY = PlaneOdometryY +  DistDelta * sin(PlaneOdometryYaw);
        //     std::cout << "X: " << PlaneOdometryX << ", Y: " << PlaneOdometryY << ", Yaw: " <<  PlaneOdometryYaw  * 57.3<< std::endl;
        //     std::cout << "DistDelta: " << DistDelta << ", angle: " <<  angle <<  ", sin: "<< sin(angle) << std::endl;
            
        //     PlaneOdometry.header  = odomAftMapped->header;
        //     PlaneOdometry.header.frame_id = "odom_loam_";
        //     PlaneOdometry.child_frame_id = "base_odom_";
        //     PlaneOdometry.pose.pose.position.x = PlaneOdometryX;
        //     PlaneOdometry.pose.pose.position.y = PlaneOdometryY;
        //     geometry_msgs::Quaternion PlaneQuat = tf::createQuaternionMsgFromRollPitchYaw(0,0,PlaneOdometryYaw);
        //     // geometry_msgs::Quaternion PlaneQuatNormalized = PlaneQuat.normalize()
        //     PlaneOdometry.pose.pose.orientation = PlaneQuat;

        //     pubOdometryPlane.publish(PlaneOdometry);
        // }

    }

    void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
    {
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        transformAftMapped[0] = -pitch;
        transformAftMapped[1] = -yaw;
        transformAftMapped[2] = roll;

        transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
        transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
        transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

        transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
        transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
        transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

        transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
        transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
        transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;

        tf::StampedTransform AftMappedTrans;
        AftMappedTrans.stamp_ = odomAftMapped -> header.stamp;
        AftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        AftMappedTrans.setOrigin(tf::Vector3(odomAftMapped->pose.pose.position.x, odomAftMapped->pose.pose.position.y, odomAftMapped->pose.pose.position.z));
        
        
        // PlaneOdometryHandler(odomAftMapped);

        geometry_msgs::PoseStamped converted_pose;
        tf::Transform transform_camera_to_base;
        tf::Transform transform_map_to_odom;

        transform_camera_to_base.setOrigin(tf::Vector3(0, 0, 0));
        transform_camera_to_base.setRotation(tf::Quaternion(-0.5, -0.5, -0.5, 0.5)); //camera to base_loam
        transform_map_to_odom.setOrigin(tf::Vector3(0, 0, 0));
        transform_map_to_odom.setRotation(tf::Quaternion(0.500, 0.500, 0.500, 0.500)); // map to odom_loam

        tf::poseTFToMsg(transform_map_to_odom *AftMappedTrans*transform_camera_to_base , converted_pose.pose); //laserOdometryTrans2 --> AftMappedTrans : laserOdometry loop rate is too fast. 
        std::cout << converted_pose.pose << std::endl;
        if (!std::isnan(converted_pose.pose.position.x) && !std::isinf(converted_pose.pose.position.x))
        {
            loamOdometry.header  = laserOdometry2.header;
            loamOdometry.header.frame_id = "odom_loam_";
            loamOdometry.child_frame_id = "base_odom_";
            loamOdometry.pose.pose = converted_pose.pose;

            pubReOdometry.publish(loamOdometry);

            laserOdometryTrans3.stamp_ = odomAftMapped->header.stamp;
            laserOdometryTrans3.setRotation(tf::Quaternion(converted_pose.pose.orientation.x, converted_pose.pose.orientation.y, converted_pose.pose.orientation.z, converted_pose.pose.orientation.w));
            laserOdometryTrans3.setOrigin(tf::Vector3(converted_pose.pose.position.x, converted_pose.pose.position.y, converted_pose.pose.position.z));
            tfBroadcaster3.sendTransform(laserOdometryTrans3);
            
            //Inner Product
            double UnitCurDist, UnitPrevDist;
            double ProdInner, angle, DistDelta, angle_sign;
            tf::Vector3 CrossProd;
            geometry_msgs::Vector3 geoVect;
            

            auto UnitCur = tf::Vector3(converted_pose.pose.position.x - PosePrevPrevX, 
                                       converted_pose.pose.position.y - PosePrevPrevY, 
                                       converted_pose.pose.position.z - PosePrevPrevZ);
            auto UnitPrev = tf::Vector3(PosePrevX - PosePrevPrevX, 
                                        PosePrevY - PosePrevPrevY,
                                        PosePrevZ - PosePrevPrevZ);
            
            UnitCurDist = tf::tfDistance(tf::Vector3(converted_pose.pose.position.x,converted_pose.pose.position.y,converted_pose.pose.position.z), 
                                        tf::Vector3(PosePrevPrevX,PosePrevPrevY,PosePrevPrevZ));
            UnitPrevDist = tf::tfDistance(tf::Vector3(PosePrevX,PosePrevY,PosePrevZ), 
                                        tf::Vector3(PosePrevPrevX,PosePrevPrevY,PosePrevPrevZ));

            DistDelta = sqrt(pow(converted_pose.pose.position.x - PosePrevX, 2) + 
                            pow(converted_pose.pose.position.y - PosePrevY, 2) +  
                            pow(converted_pose.pose.position.z - PosePrevZ, 2));

            if (CountForPlaneOdom > 3 )
            {
                ProdInner = tf::tfDot(UnitCur, UnitPrev);
                if (UnitCurDist != 0 && UnitPrevDist != 0)
                {
                    angle = acos(ProdInner/ (UnitCurDist * UnitPrevDist));    
                }
                CrossProd = tf::tfCross(UnitCur, UnitPrev);
                vector3TFToMsg(CrossProd, geoVect);
                if (geoVect.z > 0)
                { angle_sign = -1 ;}  
                else
                { angle_sign = 1;}
                
            }                         

            PosePrevPrevX = PosePrevX;
            PosePrevPrevY = PosePrevY;
            PosePrevPrevZ = PosePrevZ;
            PosePrevX = converted_pose.pose.position.x;
            PosePrevY = converted_pose.pose.position.y;
            PosePrevZ = converted_pose.pose.position.z;

            std::cout << "Prod: " << ProdInner << ", UnitCur: "<< UnitCurDist << ", UnitPrev: " <<  UnitPrevDist<< std::endl;
            std::cout << "cross product : \n" << geoVect <<  "\n Count: "<< CountForPlaneOdom << std::endl;
            //Pub Plane Odometry
            PlaneOdometryYaw = PlaneOdometryYaw + angle * angle_sign;
            if (PlaneOdometryYaw > M_PI * 2)
            {
                PlaneOdometryYaw = PlaneOdometryYaw - 2 * M_PI;
            }
            else if (PlaneOdometryYaw < -M_PI * 2)
            {
                PlaneOdometryYaw = PlaneOdometryYaw + 2 * M_PI;
            }        
            PlaneOdometryX = PlaneOdometryX +  DistDelta * cos(PlaneOdometryYaw);
            PlaneOdometryY = PlaneOdometryY +  DistDelta * sin(PlaneOdometryYaw);
            std::cout << "X: " << PlaneOdometryX << ", Y: " << PlaneOdometryY << ", Yaw: " <<  PlaneOdometryYaw  * 57.3<< std::endl;
            std::cout << "DistDelta: " << DistDelta << ", angle: " <<  angle <<  ", sin: "<< sin(angle) << std::endl;
            
            PlaneOdometry.header  = odomAftMapped->header;
            PlaneOdometry.header.frame_id = "odom_loam_";
            PlaneOdometry.child_frame_id = "base_odom_";
            PlaneOdometry.pose.pose.position.x = PlaneOdometryX;
            PlaneOdometry.pose.pose.position.y = PlaneOdometryY;
            geometry_msgs::Quaternion PlaneQuat = tf::createQuaternionMsgFromRollPitchYaw(0,0,PlaneOdometryYaw);
            // geometry_msgs::Quaternion PlaneQuatNormalized = PlaneQuat.normalize()
            PlaneOdometry.pose.pose.orientation = PlaneQuat;

            pubOdometryPlane.publish(PlaneOdometry);
        }


        CountForPlaneOdom ++;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");
    
    TransformFusion TFusion;

    ROS_INFO("\033[1;32m---->\033[0m Transform Fusion Started.");

    ros::spin();

    return 0;
}


        // /* --------------------------------------------------------------------------------------------------------------- */
        // geometry_msgs::PoseStamped converted_pose;
        // tf::Transform transform;
        // transform.setOrigin(tf::Vector3(0, 0, 0));
        // transform.setRotation(tf::Quaternion(-0.5, -0.5, -0.5, 0.5)); //camera to base_loam
        // tf::Transform transform_map_to_odom;
        // transform_map_to_odom.setOrigin(tf::Vector3(0, 0, 0));
        // transform_map_to_odom.setRotation(tf::Quaternion(0.500, 0.500, 0.500, 0.500)); //camera to base_loam


        // tf::poseTFToMsg(transform_map_to_odom *AftMappedTrans*transform , converted_pose.pose); //laserOdometryTrans2 --> AftMappedTrans
        // std::cout << converted_pose.pose << std::endl;

        // loamOdometry.header  = laserOdometry2.header;
        // loamOdometry.header.frame_id = "odom_loam_";
        // loamOdometry.child_frame_id = "base_odom_";
        // loamOdometry.pose.pose = converted_pose.pose;

        // pubReOdometry.publish(loamOdometry);

        // laserOdometryTrans3.stamp_ = loamOdometry.header.stamp;
        // laserOdometryTrans3.setRotation(tf::Quaternion(converted_pose.pose.orientation.x, converted_pose.pose.orientation.y, converted_pose.pose.orientation.z, converted_pose.pose.orientation.w));
        // laserOdometryTrans3.setOrigin(tf::Vector3(converted_pose.pose.position.x, converted_pose.pose.position.y, converted_pose.pose.position.z));
        // tfBroadcaster3.sendTransform(laserOdometryTrans3);
        // // CountForPlaneOdom++;
        // //Inner Product

        // double UnitCurDist, UnitPrevDist;
        // double ProdInner, angle, DistDelta, angle_sign;
        // tf::Vector3 CrossProd;
        // geometry_msgs::Vector3 geoVect;

        // auto UnitCur = tf::Vector3(converted_pose.pose.position.x - PosePrevPrevX, 
        //                            converted_pose.pose.position.y - PosePrevPrevY, 
        //                            converted_pose.pose.position.z - PosePrevPrevZ);
        // auto UnitPrev = tf::Vector3(PosePrevX - PosePrevPrevX, 
        //                             PosePrevY - PosePrevPrevY,
        //                             PosePrevZ - PosePrevPrevZ);
        
        // UnitCurDist = tf::tfDistance(tf::Vector3(converted_pose.pose.position.x,converted_pose.pose.position.y,converted_pose.pose.position.z), 
        //                              tf::Vector3(PosePrevPrevX,PosePrevPrevY,PosePrevPrevZ));
        // UnitPrevDist = tf::tfDistance(tf::Vector3(PosePrevX,PosePrevY,PosePrevZ), 
        //                               tf::Vector3(PosePrevPrevX,PosePrevPrevY,PosePrevPrevZ));

        // DistDelta = sqrt(pow(converted_pose.pose.position.x - PosePrevX, 2) + 
        //                  pow(converted_pose.pose.position.y - PosePrevY, 2) +  
        //                  pow(converted_pose.pose.position.z - PosePrevZ, 2));

        // if (CountForPlaneOdom > 2)
        // {
        //     // double angle = tf::tfAngle(UnitCur, UnitPrev);
        //     ProdInner = tf::tfDot(UnitCur, UnitPrev);
        //     angle = acos(ProdInner/ (UnitCurDist * UnitPrevDist));
        //     CrossProd = tf::tfCross(UnitCur, UnitPrev);
        //     vector3TFToMsg(CrossProd, geoVect);
        //     if (geoVect.z > 0)
        //     { angle_sign = 1 ;}  
        //     else
        //     { angle_sign = -1;}
        // }                         

        // PosePrevPrevX = PosePrevX;
        // PosePrevPrevY = PosePrevY;
        // PosePrevPrevZ = PosePrevZ;
        // PosePrevX = converted_pose.pose.position.x;
        // PosePrevY = converted_pose.pose.position.y;
        // PosePrevZ = converted_pose.pose.position.z;

        // std::cout << "Prod: " << ProdInner << ", UnitCur: "<< UnitCurDist << ", UnitPrev: " <<  UnitPrevDist<< std::endl;
        // std::cout << "cross product : " << geoVect << std::endl;
        // //Pub Plane Odometry
        // PlaneOdometryYaw = PlaneOdometryYaw + angle * angle_sign;
        // if (PlaneOdometryYaw > M_PI * 2)
        // {
        //     PlaneOdometryYaw = PlaneOdometryYaw - 2 * M_PI;
        // }
        // else if (PlaneOdometryYaw < -M_PI * 2)
        // {
        //     PlaneOdometryYaw = PlaneOdometryYaw + 2 * M_PI;
        // }        
        // PlaneOdometryX = PlaneOdometryX +  DistDelta * cos(PlaneOdometryYaw);
        // PlaneOdometryY = PlaneOdometryY +  DistDelta * sin(PlaneOdometryYaw);
        // std::cout << "X: " << PlaneOdometryX << ", Y: " << PlaneOdometryY << ", Yaw: " <<  PlaneOdometryYaw  * 57.3<< std::endl;
        // std::cout << "DistDelta: " << DistDelta << ", angle: " <<  angle <<  ", sin: "<< sin(angle) << std::endl;
        

        // PlaneOdometry.header  = laserOdometry2.header;
        // PlaneOdometry.header.frame_id = "odom_loam_";
        // PlaneOdometry.child_frame_id = "base_odom_";
        // PlaneOdometry.pose.pose.position.x = PlaneOdometryX;
        // PlaneOdometry.pose.pose.position.y = PlaneOdometryY;

        // geometry_msgs::Quaternion PlaneQuat = tf::createQuaternionMsgFromRollPitchYaw(0,0,PlaneOdometryYaw);
        // PlaneOdometry.pose.pose.orientation = PlaneQuat;

        // pubOdometryPlane.publish(PlaneOdometry);


        // /* ------------------------------------------------------------------------------------------------------------------------- */