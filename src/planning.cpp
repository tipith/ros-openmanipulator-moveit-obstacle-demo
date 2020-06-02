#include <planning.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace my_planning
{
        void MyPlanningClass::goToPoseGoal()
        {
            move_group.setPoseTarget(target_pose1);
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(!success) //execute
                throw std::runtime_error("No plan found");

            move_group.move(); //blocking
        }
        
        void MyPlanningClass::goToPoseGoal(geometry_msgs::Pose &pose)
        {
            move_group.setPoseTarget(pose);
            ros::Duration(0.5).sleep();
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            /*while (!success) //keep trying until a plan is found
            {
                
                success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            }*/
            
            if(!success) //execute
                throw std::runtime_error("No plan found");

            move_group.move(); //blocking
        }

        void MyPlanningClass::goToJointState()
        {
            robot_state::RobotState current_state = *move_group.getCurrentState();
            //moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            std::vector<double> joint_positions;
            joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
            current_state.copyJointGroupPositions(joint_model_group, joint_positions);
            //joint_positions = move_group.getCurrentJointValues();

            joint_positions[0] = -1.0;
            joint_positions[3] = 0.7;

            move_group.setJointValueTarget(joint_positions);
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(!success)
                throw std::runtime_error("No plan found");

            move_group.move(); //blocking
        }

        void MyPlanningClass::cartesianPath()
        {
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(target_pose1);

            geometry_msgs::Pose target_pose2 = target_pose1;

            target_pose2.position.z -= 0.2;
            waypoints.push_back(target_pose2);

            target_pose2.position.y -= 0.2;
            waypoints.push_back(target_pose2);

            target_pose2.position.z += 0.2;
            target_pose2.position.y += 0.2;
            target_pose2.position.x -= 0.2;
            waypoints.push_back(target_pose2);

            move_group.setMaxVelocityScalingFactor(0.1);

            // We want the Cartesian path to be interpolated at a resolution of 1 cm
            // which is why we will specify 0.01 as the max step in Cartesian
            // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
            // Warning - disabling the jump threshold while operating real hardware can cause
            // large unpredictable motions of redundant joints and could be a safety issue
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            move_group.move();
            ROS_INFO_STREAM("Percentage of path followed: " << fraction);
        }

        void MyPlanningClass::resetValues()
        {
            //set the start state and operational speed
            move_group.setStartStateToCurrentState();
            move_group.setMaxVelocityScalingFactor(1.0);
        }

        void MyPlanningClass::makeCylinder(std::string blk_name, float height, geometry_msgs::Pose& pose)
         {
            moveit_msgs::CollisionObject cyl;
            //set the relative frame
            cyl.header.frame_id = "odom";
            cyl.header.frame_id = move_group.getPlanningFrame();
            cyl.id = blk_name;

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = height;
            primitive.dimensions[1] = 0.02;

            cyl.primitives.push_back(primitive);
            cyl.primitive_poses.push_back(pose);
            cyl.operation = cyl.ADD;

            std::vector<moveit_msgs::CollisionObject> collisionObjects;
            collisionObjects.push_back(cyl);
            ros::Duration(2).sleep();
            virtual_world.addCollisionObjects(collisionObjects);
            ROS_INFO_STREAM("Added: " << blk_name);
         }

         void MyPlanningClass::makeBox(std::string blk_name, double *pose)
         {
             moveit_msgs::CollisionObject box;
            //set the relative frame
            box.header.frame_id = move_group.getPlanningFrame();
            box.id = blk_name;

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.2;
            primitive.dimensions[1] = 0.2;
            primitive.dimensions[2] = 1.0;

            geometry_msgs::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = pose[0];
            box_pose.position.y = pose[1];
            box_pose.position.z = pose[2];

            box.primitives.push_back(primitive);
            box.primitive_poses.push_back(box_pose);
            box.operation = box.ADD;

            std::vector<moveit_msgs::CollisionObject> collisionObjects;
            collisionObjects.push_back(box);
            ros::Duration(2).sleep();
            virtual_world.addCollisionObjects(collisionObjects);
            ROS_INFO_STREAM("Added: " << blk_name);
         }

        void MyPlanningClass::addObjects()
        {
            float h = 0.7;

            geometry_msgs::Pose pose;
            pose.orientation.w = 1.0;
            pose.position.x = 0.10;
            pose.position.y = -0.20;
            pose.position.z = h / 2;

            makeCylinder("block_1", h, pose);
            pose.position.y = 0.20;
            makeCylinder("block_2", h, pose);
            pose.position.y = 0.00;
            makeCylinder("block_3", h, pose);
        }

        void MyPlanningClass::removeObjects()
        {
            ros::Duration(2).sleep();
            virtual_world.removeCollisionObjects({ "block_1", "block_2", "block_3" });
        }

        void MyPlanningClass::attach()
        {
            double z_offset_cylinder = .0;
            tf2::Quaternion orientation;


            moveit_msgs::CollisionObject cylinder;
            cylinder.id = "log";
            cylinder.header.frame_id = "gripper_link_sub";
            cylinder.primitives.resize(1);
            cylinder.primitive_poses.resize(1);
            cylinder.primitives[0].type = cylinder.primitives[0].CYLINDER;
            cylinder.primitives[0].dimensions.resize(2);
            cylinder.primitives[0].dimensions[0] = 0.35;   // height (along x)
            cylinder.primitives[0].dimensions[1] = 0.01;  // radius
            cylinder.primitive_poses[0].position.x = 0.1;
            cylinder.primitive_poses[0].position.y = 0.03;
            cylinder.primitive_poses[0].position.z = 0.0 + z_offset_cylinder;
            orientation.setRPY(0, 0.0 / 180.0 * M_PI, 0);
            cylinder.primitive_poses[0].orientation = tf2::toMsg(orientation);
            cylinder.operation = moveit_msgs::CollisionObject::ADD;
            virtual_world.applyCollisionObject(cylinder);

            moveit_msgs::AttachedCollisionObject att_coll_object;
            att_coll_object.object.id = "log";
            att_coll_object.link_name = "gripper_link_sub";
            att_coll_object.touch_links = std::vector<std::string>{ "end_effector_link", "gripper_link", "gripper_link_sub" };

            att_coll_object.object.operation = att_coll_object.object.ADD;
            ROS_INFO_STREAM("Attaching cylinder to robot.");
            virtual_world.applyAttachedCollisionObject(att_coll_object);
        }

        void MyPlanningClass::detach()
        {
            moveit_msgs::AttachedCollisionObject att_coll_object;
            att_coll_object.object.id = "log";
            att_coll_object.link_name = "gripper_link_sub";
            att_coll_object.object.operation = att_coll_object.object.REMOVE;
            ROS_INFO_STREAM("Detaching cylinder to robot.");
            virtual_world.applyAttachedCollisionObject(att_coll_object);

            moveit_msgs::CollisionObject co;
            co.id = "log";
            co.operation = moveit_msgs::CollisionObject::REMOVE;
            virtual_world.applyCollisionObject(co);
        }
}