/*
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_obstacle_distance
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: July, 2015
 *
 * \brief
 *   This header contains a definition of a FrameToCollision management class.
 ****************************************************************/

#include <stdint.h>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <urdf/model.h>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <fcl/collision_object.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/collision_data.h>

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>


#include "cob_obstacle_distance/marker_shapes/marker_shapes_interface.hpp"
#include "cob_obstacle_distance/marker_shapes/marker_shapes.hpp"
#include "cob_obstacle_distance/shapes_manager.hpp"
#include "cob_obstacle_distance/obstacle_distance_data_types.hpp"

class FrameToCollision
{
    private:
        typedef boost::shared_ptr<const urdf::Link> PtrConstLink_t;
        typedef boost::shared_ptr<urdf::Link> PtrLink_t;
        typedef std::vector<boost::shared_ptr<urdf::Link> > VecPtrLink_t;
        typedef boost::shared_ptr<urdf::Collision> PtrCollision_t;
        typedef boost::shared_ptr<urdf::Geometry> PtrGeometry_t;
        typedef boost::shared_ptr<urdf::Mesh> PtrMesh_t;
        typedef boost::shared_ptr<urdf::Box> PtrBox_t;
        typedef boost::shared_ptr<urdf::Sphere> PtrSphere_t;
        typedef boost::shared_ptr<urdf::Cylinder> PtrCylinder_t;
        typedef std::unordered_map<std::string, std::vector<std::string> >::iterator MapIter_t;

        urdf::Model model_;
        bool success_;
        std::string root_frame_id_;
        std::unordered_map<std::string, std::vector<std::string> > self_collision_frames_; /// first: key of part to be checked with, second ignore links

        /**
         * Private method to create a specific marker shape for the output pointer.
         * @param frame_of_interest The frame of interest name (link name).
         * @param pose The pose of the frame (root of the frame).
         * @param col The color.
         * @param geometry A pointer to a URDF geometry object.
         * @param segment_of_interest_marker_shape The pointer for that a marker shape shall be created.
         */
        void createSpecificMarkerShape(const std::string& frame_of_interest,
                                       const geometry_msgs::Pose& pose,
                                       const std_msgs::ColorRGBA& col,
                                       const PtrGeometry_t& geometry,
                                       PtrIMarkerShape_t& segment_of_interest_marker_shape);

    public:
        typedef std::unordered_map<std::string, std::vector<std::string> > MapSelfCollisions_t;

        FrameToCollision();
        ~FrameToCollision();

        inline MapSelfCollisions_t::iterator getSelfCollisionsIterBegin()
        {
            return this->self_collision_frames_.begin();
        }

        inline MapSelfCollisions_t::iterator getSelfCollisionsIterEnd()
        {
            return this->self_collision_frames_.end();
        }

        bool ignoreSelfCollisionPart(const std::string& frame_of_interest, const std::string& self_collision_obstacle_frame);

        /**
         * Initialize the FrameToCollision model by an URDF file.
         * @param root_frame_id The id of the root frame as base for further transformations.
         * @param urdf_file_name The URDF file name (containing path).
         * @return State of success.
         */
        bool initFile(const std::string& root_frame_id, const std::string& urdf_file_name);

        /**
         * Initialize the FrameToCollision model by an ROS parameter containting the URDF description.
         * @param root_frame_id The id of the root frame as base for further transformations.
         * @param urdf_param The ROS parameter containing the URDF description.
         * @return State of success.
         */
        bool initParameter(const std::string& root_frame_id, const std::string& urdf_param);

        /**
         * From the parameters self collision dictionary the keys are extracted as the self-collision "obstacles".
         * While the values are the parts to be ignored for self-collision checking.
         * Additionally the parts to be ignored are extended by the parent and child elements of the link corresponding to the key.
         * @param self_collision_params A XML RPC data structure representing the self_collision params (from YAML or parameter server)
         * @param sm The shapes manager that will by extended by self-collision "obstacles"
         * @return State of success.
         */
        bool initSelfCollision(XmlRpc::XmlRpcValue& self_collision_params, boost::scoped_ptr<ShapesManager>& sm);


        /**
         * Tries to find the given frame_of_interest in the links parsed from URDF.
         * According to the data there a MarkerShape is created and will be assigned to the pointer.
         * @param abs_pos The absolute position (from root frame) of the shape.
         * @param quat_pos The orientation (from root frame) of the shape.
         * @param frame_of_interest The name of the frame of interest (e.g. link name).
         * @param segment_of_interest_marker_shape The pointer for that a marker shape shall be created.
         * @return State of success.
         */
        bool getMarkerShapeFromUrdf(const Eigen::Vector3d& abs_pos,
                                    const Eigen::Quaterniond& quat_pos,
                                    const std::string& frame_of_interest,
                                    PtrIMarkerShape_t& segment_of_interest_marker_shape);

        /**
         * Tries to create a MarkerShape by a given shape_type.
         * If shape_type is MESH_RESOURCE then the behaviour is similar to getMarkerShapeFromUrdf(..).
         * @param shape_type The type of the shape (visualization_marker types).
         * @param abs_pos The absolute position (from root frame) of the shape.
         * @param quat_pos The orientation (from root frame) of the shape.
         * @param frame_of_interest The name of the frame of interest (e.g. link name if shape_type MESH_RESOURCE else only for id).
         * @param segment_of_interest_marker_shape The pointer for that a marker shape shall be created.
         * @return State of success.
         */
        bool getMarkerShapeFromType(const uint32_t& shape_type,
                                    const Eigen::Vector3d& abs_pos,
                                    const Eigen::Quaterniond& quat_pos,
                                    const std::string& frame_of_interest,
                                    const Eigen::Vector3d& dimension,
                                    PtrIMarkerShape_t& segment_of_interest_marker_shape);

        /**
         * Tries to create a MarkerShape by a given shape_type (similar to above but with pose).
         * @param shape_type The type of the shape (visualization_marker types).
         * @param pose The pose of the shape (with respect to the root_frame).
         * @param frame_of_interest The name of the frame of interest (e.g. link name if shape_type MESH_RESOURCE else only for id).
         * @param segment_of_interest_marker_shape The pointer for that a marker shape shall be created.
         * @return State of success.
         */
        bool getMarkerShapeFromType(const uint32_t& shape_type,
                                    const geometry_msgs::Pose& pose,
                                    const std::string& frame_of_interest,
                                    const Eigen::Vector3d& dimension,
                                    PtrIMarkerShape_t& segment_of_interest_marker_shape);

};
