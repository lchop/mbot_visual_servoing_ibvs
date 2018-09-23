//
// Created by lchopot on 28-06-2018.
//
#include <VS_IBVS_node/vs_ibvs_node.h>

#undef DEBUG
#define NAN_ALLOWED_RATIO 0.25 //if in vector you have more than 25% you dont take depth value

namespace VS_IBVS {

        IBVS_mbot::IBVS_mbot() : cam_px(800), cam_py(800), nh_(""), pnh_("~")
        {
            error.data = 1.0; //init error value
            //Flags
            is_running_ = false;
            depth_image_received_ = false;
            roi_received_ = false;
            init_done = false;
            depth_error = false;

            //Parameters see config file for more info !
            pnh_.param<bool>("filter_class", filter_class_, false);
            pnh_.param<std::string>("filter_class_name", filter_class_name_, "");

            pnh_.param<double>("node_frequency", node_frequency, 10.0);

            pnh_.getParam("Display", Display);
            pnh_.getParam("plot", plot);
            pnh_.getParam("save", save);

            pnh_.getParam("error_max", error_max);
            pnh_.getParam("end_effector_velocities_max", end_effector_velocities_max);
            pnh_.getParam("depth_desired", depth_desired);
            pnh_.getParam("commande_gripper", command_gripper);

            pnh_.getParam("gain_for_high_value", gain_for_high_value);
            pnh_.getParam("gain_for_low_value", gain_for_low_value);
            pnh_.getParam("gain_slope", gain_slope);
            pnh_.getParam("gain_low_pass_filter", gain_low_pass_filter);

            pnh_.getParam("Blob_tracking", Blob_tracking);
            pnh_.getParam("Yolo_Center_ROI", Yolo_Center_ROI);

            pnh_.getParam("MASTER_URI", master_uri);
            pnh_.getParam("CamInfo_Topic", CamInfo_Topic);
            pnh_.getParam("RgbImage_Topic", RgbImage_Topic);
            pnh_.getParam("DepthImage_Topic", DepthImage_Topic);
            pnh_.getParam("DetectionResultsYolo_Topic", DetectionResultsYolo_Topic);

            //Pub Sub
            //output event
            event_out_pub_ = pnh_.advertise<std_msgs::String>("event_out", 5);
            //subscribe to events
            event_in_sub_ = pnh_.subscribe("event_in", 5, &IBVS_mbot::EventInCallBack, this);
            //publish target_vel on a topic called ibvs/target_vel
            pub_vel = nh_.advertise<geometry_msgs::TwistStamped>("ibvs/target_vel", 1);
            //publish the quadratique error of the visual servoing law
            pub_error = nh_.advertise<std_msgs::Float64>("ibvs/error",1);
            //publish the command to close the gripper
            pub_gripper = nh_.advertise<std_msgs::Float64>("left_arm_gripper_joint_position_controller/command", 1, true);
        }

        void IBVS_mbot::GrabberImageFromRos()
        {
            g.setMasterURI(master_uri);
            g.setCameraInfoTopic(CamInfo_Topic);
            g.setImageTopic(RgbImage_Topic);
            g.setRectify(true);
            g.setImageTransport("jpeg");
            g.open(I);
            std::cout << "Image rgb received, size: " << I.getWidth() << " " << I.getHeight() << std::endl;
        }

        void IBVS_mbot::GetCameraParameters()
        {
            if (!g.getCameraInfo(cam))
                cam.initPersProjWithoutDistortion(cam_px,cam_py,I.getWidth()/2, I.getHeight()/2);
            std::cout << "cam param!" << cam << std::endl;
        }

        void IBVS_mbot::InitVispTask(vpServo::vpServoType a_ServoType , vpServo::vpServoIteractionMatrixType a_InteractionType, vpServo::vpServoInversionType a_ServoInversionType)
        {
            vpAdaptiveGain lambda(gain_for_low_value, gain_for_high_value, gain_slope);
            task.setServo(a_ServoType);
            task.setInteractionMatrixType(a_InteractionType, a_ServoInversionType);
            task.setLambda(lambda);
            task.print();
        }

        void IBVS_mbot::DisplayImage(std::string& text_to_print_on_image)
        {
            vpDisplay::display(I);
            vpDisplay::displayText(I, 10, 10, text_to_print_on_image, vpColor::red);
            vpDisplay::flush(I);
        }

        void IBVS_mbot::InitTracking_Dot_BlobTracking()
        {
            dot.setGraphics(true);
            dot.setEllipsoidShapePrecision(0.9);  // to track a blob without any constraint on the shape
            dot.setGrayLevelPrecision(0.9);  // to set the blob gray level bounds for Init_4Dot_BlobTrackingbinarisation
            dot.setEllipsoidBadPointsPercentage(0.2); // to be accept 20% of bad inner and outside points with bad gray level
            dot.setGrayLevelMin(0); // the gray lvl to be in the blob (0 here because of black points)
            dot.setMaxSizeSearchDistancePrecision(0.2);
            dot.initTracking(I);
            cog = dot.getCog();

            vpDisplay::flush(I);
        }

        void IBVS_mbot::CreateCurrentFeatures_Dot_BlobTracking()
        {
            /* Creation of the current feature for Blob tracking detection */

            double coef = 1./95.; // At 1 m the blob has a surface of 95 (approximatively) (used to calculate depth)
            double surface; // surface of the blob used to compute depth
            double Z; // Depth vpDot2 feature

            //Create vpFeature for s = (x,y)
            vpFeatureBuilder::create(p_blob, cam, dot.getCog());
            //Surface of the blob estimated from the image moment m00 and converted in meters
            surface = 1./sqrt(dot.m00/(cam.get_px()*cam.get_py()));
            Z = coef * surface; // Calculating the depth from each current feature point
            p_blob.set_Z(Z);
            //Create VpFeatureDepth, one more parameter for one point, now s = (x,y,z)
            s_Z_blob.buildFrom(p_blob.get_x(), p_blob.get_y(), Z, 0);
        }

        void IBVS_mbot::CreateDesiredFeatures_Dot_BlobTracking()
        {
            /* Creation of the desired feature for Blob tracking detection */

            //Create a point in the image plane in m.
            vpPoint point;
            point.set_x(0);
            point.set_y(0);

            //Create vpFeature for s* = (x,y)
            vpFeatureBuilder::create(pd_blob, point);
            pd_blob.set_Z(depth_desired);
            //Create VpFeatureDepth, one more parameter for one point, now s* = (x,y,z)
            s_Zd_blob.buildFrom(pd_blob.get_x(), pd_blob.get_y(), depth_desired, 0);
        }

        void IBVS_mbot::UpdateCurrentFeatures_Dot_Blob_Tracking()
        {
            double coef = 1./95.; // At 1 m the blob has a surface of 95 (approximatively) (used to calculate depth)
            double surface; // surface of the blob used to compute depth
            double Z; // Depth for each vpDot2 features

            //Track blob
            dot.track(I); //track the blob in I
            cog = dot.getCog(); // get center of gravity of blob

            //Update vpFeature s = (x,y)
            vpFeatureBuilder::create(p_blob, cam, dot);// Update the current features
            //Update the depth of the feature
            surface = 1. / sqrt(dot.m00 / (cam.get_px() * cam.get_py()));
            Z = coef * surface;
            p_blob.set_Z(Z);
            //Update VpFeatureDepth
            s_Z_blob.buildFrom(p_blob.get_x(), p_blob.get_y(), Z, log(Z/depth_desired)) ;
        }

        void IBVS_mbot::Init_Dot_BlobTracking()
        {
            /* Initialization for blob tracking use */
            std::string text_to_print_on_image = "Click on the 4 dots";

            GetCameraParameters();
            InitVispTask(vpServo::vpServoType::EYEINHAND_CAMERA, vpServo::vpServoIteractionMatrixType::MEAN,
                         vpServo::vpServoInversionType::PSEUDO_INVERSE);

            if(!text_to_print_on_image.empty()) //check if string not empty and Display enabled
                DisplayImage(text_to_print_on_image);
            InitTracking_Dot_BlobTracking();
            CreateCurrentFeatures_Dot_BlobTracking();
            CreateDesiredFeatures_Dot_BlobTracking();
            AddFeaturesToTask();
        }

        vpRect IBVS_mbot::RoiFromYOLO(const darknet_ros_py::RecognizedObjectArrayStamped rec)
        {
            /* Get YOLO bounding box and store this in a vpRect to be able to work on it with visp lib
             * input: YOLO array with objects
             * output: ROI, bounding box from yolo in a visp format*/

            vpRect roi_area;
            //Filtering with the class
            for (const darknet_ros_py::RecognizedObject &object : rec.objects.objects)
            {
                if (filter_class_)
                {
                    if (object.class_name != filter_class_name_)
                        continue;
                }
                const sensor_msgs::RegionOfInterest &roi = object.bounding_box;

                // bounding box easy access
                const int bb_x_min = roi.x_offset;
                const int bb_y_min = roi.y_offset;
                const int bb_width = roi.width;
                const int bb_height = roi.height;

                roi_area.set(bb_x_min,bb_y_min,bb_width,bb_height);
            }
            //return bounding box
            return roi_area;
        }

        float getDepthBySorting(const cv::Mat& mat)
        {
            /* Compute the depth by sorting
             * input: bounding box
             * output: depth for the center point*/

            cv::Mat flat;
            cv::Mat flat_ord;
            cv::Point BottomLeft;
            cv::Point TopRight;
            BottomLeft.x = (cv::Point(mat.size()).x*3/8);
            BottomLeft.y = (cv::Point(mat.size()).y*3/8);
            TopRight.x = (cv::Point(mat.size()).x*5/8);
            TopRight.y = (cv::Point(mat.size()).y*5/8);
            // Select a quarter of the bounding box from its center (mat.size is (x,y))
            const cv::Rect half(BottomLeft, TopRight);

            // Needs copy because it is not continuous
            const cv::Mat half_mat(mat, half);
            // Check NANs
            unsigned long nan_count = 0;
            std::vector<float> half_filtered_vec;
            for (int row = 0; row < half_mat.rows; ++row) {
                for (int col = 0; col < half_mat.cols; ++col) {
                    const auto val = half_mat.at<float>(row, col);
                    if (val==0 || std::isnan(val))
                        nan_count++;
                    else
                        half_filtered_vec.push_back(val);
                }
            }

            // If number of NANs is too high, return -1 to signal no-publish
            const int num_values = half_mat.rows * half_mat.cols;

            if (nan_count > (1 - NAN_ALLOWED_RATIO) * num_values) {
                ROS_WARN_THROTTLE(5.0,
                                  "Too many NANs (%d out of %d values) in the provided bounding-boxed depth map, do not publish",
                                  nan_count, num_values);
                return -1;
            }

            if (half_filtered_vec.empty()) {
                ROS_ERROR("NAN-Filtered vector is empty!!?? Contact a developer");
                return -1;
            }

            // Sort the NAN-filtered vector
            std::sort(half_filtered_vec.begin(), half_filtered_vec.end());

            // Return the depth at the 25th percentile
            return half_filtered_vec.at(half_filtered_vec.size() / 4);
        }

        void IBVS_mbot::GetROICenterDepth()
        {
            /* Call the getDepthBySorting function
             * Verification if the depth is a plausible value
             * If not correct send the arm to go back, because may be to close or may be gripper in front
             * if correct store in a class variable to be used after*/

            vpColVector v_back;

            vpRect roi_area = RoiFromYOLO(last_received_array_);
            double depth;
            const cv::Mat depmat = last_received_depth_->image;
#ifdef DEBUG
            cv::imshow("depth",depmat);
            cv::waitKey(30);
#endif
            const cv::Rect bb_rect(roi_area.getTopLeft().get_u(), roi_area.getTopLeft().get_v(), roi_area.getWidth(),
                                   roi_area.getHeight());
            cv::Mat roi_mat; //roiArea;

            try
            {
                // obtain the image ROI:
                roi_mat = cv::Mat(depmat, bb_rect);
#ifdef DEBUG
                cv::imshow("roi_mat", roi_mat);
                cv::waitKey(30);
#endif
                depth = getDepthBySorting(roi_mat)*0.001; // we get the depth value in mm from CV_32FC1 image format convert in m

                std::cout << "distance: " << depth <<std::endl;
                if (depth <=  0 || std::isnan(depth))
                {
                    std_msgs::String e_out;
                    e_out.data = "e_nans";
                    event_out_pub_.publish(e_out);
                    ROS_INFO("Depth value incorrect, sending velocities to go back");
                    for (int i = 0; i<6;i++)
                        vel_back[i] = 0;
                    vel_back[0] = -0.05;
                    ApplyVelocitiesInMsgAndPublish(vel_back, "left_arm_camera_link");
                    depth_error = true;
                }
                else
                {
                    depth_center_roi = depth;
                    depth_error =false;
                }


            } catch (const cv::Exception &ex)
            {
                ROS_ERROR("%s", ex.what());
            }

        }

        void IBVS_mbot::CreateCurrentFeatures_CenterBoundingBox()
        {
            /* Create current features for the Center Bounding Box method*/

            vpRect roi_rect;
            vpImagePoint center;
            double x, y;

            //Get ROI from YOLO and positon of the center of this ROI
            roi_rect = RoiFromYOLO(last_received_array_);
            if (Display)
                vpDisplay::displayRectangle(I,roi_rect.getTopLeft(),roi_rect.getBottomRight(), vpColor::green);

            roi_rect.getCenter(x, y);

            //Init for low-pass filter
            x_center = x;
            y_center = y;
            center.set_uv(x_center, y_center);

            //Get depth of the center of the ROI
            GetROICenterDepth();

            //Create current Image features (2 features, x positon and y positon)
            vpFeatureBuilder::create(p, cam, center);
            p.set_Z(depth_center_roi);

            //Create current Depth features ( 1 feature, z positon)
            s_Z.buildFrom(p.get_x(), p.get_y(), depth_center_roi, 0);
        }

        void IBVS_mbot::CreateDesiredFeatures_CenterBoundingBox()
        {
            /*  Create desired feature for the Center Bounding Box method */

            vpPoint center;
            // Point in the center of the gripper in the image frame in m.
            center.set_x(0.1);
            center.set_y(0.2);
            //Create desired Image features s* =(x,y)
            vpFeatureBuilder::create(pd, center);
            pd.set_Z(depth_center_roi);

            //Create desired Depth feature s* = (x,y,z), set to 0 because it's defined as the log(Z/Z*)
            s_Zd.buildFrom(pd.get_x(), pd.get_y(), depth_center_roi, 0);
        }

        void IBVS_mbot::UpdateCurrentFeatures_CenterBoundingBox()
        {
            /*  Update current feature for the Center Bounding Box method */

            vpRect roi_rect;
            vpImagePoint center;
            double x, y;

            //Get the new ROI and the new position of the center of this ROI
            roi_rect = RoiFromYOLO(last_received_array_);
            if (Display)
                vpDisplay::displayRectangle(I,roi_rect.getTopLeft(),roi_rect.getBottomRight(), vpColor::green);
            roi_rect.getCenter(x, y);
            //low pass filter
            x_center = x_center + gain_low_pass_filter*(x - x_center);
            y_center = y_center + gain_low_pass_filter*(y - y_center);
            center.set_uv(x_center, y_center);

            //Update Depth
            GetROICenterDepth();

            //Update current Image features with new position in the image
            vpFeatureBuilder::create(p, cam, center);
            p.set_Z(depth_center_roi);

            //Update Depth feature with new depth value
            s_Z.buildFrom(p.get_x(), p.get_y(), depth_center_roi, log(depth_center_roi/depth_desired));
        }

        void IBVS_mbot::Init_CenterBoundingBox()
        {
            /*Initialization for the Center Bounding Box method*/

            GetCameraParameters();
            InitVispTask(vpServo::vpServoType::EYEINHAND_CAMERA, vpServo::vpServoIteractionMatrixType::MEAN,
                         vpServo::vpServoInversionType::PSEUDO_INVERSE);

            CreateCurrentFeatures_CenterBoundingBox();
            CreateDesiredFeatures_CenterBoundingBox();
            AddFeaturesToTask();
        }

        void IBVS_mbot::CreatePlot_Features_CartesianVelocities(vpPlot& plotter)
        {
            /* Real time curve plotting initialization and configuration*/
            plotter.setTitle(0, "Visual features error");
            plotter.setTitle(1, "Camera velocities");
            plotter.initGraph(0, 8);
            plotter.initGraph(1, 6);
            plotter.setLegend(0, 0, "x1");
            plotter.setLegend(0, 1, "y1");
            plotter.setLegend(0, 2, "x2");
            plotter.setLegend(0, 3, "y2");
            plotter.setLegend(0, 4, "x3");
            plotter.setLegend(0, 5, "y3");
            plotter.setLegend(0, 6, "x4");
            plotter.setLegend(0, 7, "y4");
            plotter.setLegend(1, 0, "v_x");
            plotter.setLegend(1, 1, "v_y");
            plotter.setLegend(1, 2, "v_z");
            plotter.setLegend(1, 3, "w_x");
            plotter.setLegend(1, 4, "w_y");
            plotter.setLegend(1, 5, "w_z");
        }

        void IBVS_mbot::AddFeaturesToTask()
        {
            /* Adding feature to visual task for both method*/

            if (Blob_tracking)
            {
                task.addFeature(p_blob, pd_blob); // Add the features to the visual task
                task.addFeature(s_Z_blob, s_Zd_blob); // Add depth features to the visual task
                task.print(); //Print the task fully initialized
            }
            else if (Yolo_Center_ROI)
            {
                task.addFeature(p, pd); // Add the features to the visual task
                task.addFeature(s_Z, s_Zd); // Add depth features to the visual task
                task.print(); //Print the task fully initialized
            }
        }

        void IBVS_mbot::ArmEndEffectorVelocitiesLimitation(float end_effector_velocities_max, vpColVector& v_in_out)
        {
            /* Arm velocities limitation*/

            for (int i = 0; i < 6; i++)
            {
                if (v_in_out[i] > end_effector_velocities_max) // restrict the arm speed.
                    v_in_out[i] = end_effector_velocities_max;

                else if (v_in_out[i] < -end_effector_velocities_max)
                    v_in_out[i] = -end_effector_velocities_max;
            }
        }

        vpColVector IBVS_mbot::TransformFromVispCameraFrameToMbotCameraFrame(vpColVector v_in)
        {
            /*The camera frame of the actual camera in the urdf and the camera frame where the velocities are computed are not the same */

            vpColVector v_out;
            vpMatrix tf_matrix(6,6);

            for (unsigned int i=0; i<tf_matrix.getRows(); i++)
                for (unsigned int j=0; j<tf_matrix.getCols(); j++)
                    tf_matrix[i][j] = 0;

            tf_matrix[0][2] = 1;
            tf_matrix[1][0] = -1;
            tf_matrix[2][1] = -1;
            tf_matrix[3][5] = 1;
            tf_matrix[4][3] = -1;
            tf_matrix[5][4] = -1;

            v_out = tf_matrix*v_in;
            return v_out;
        }

        void IBVS_mbot::ApplyVelocitiesInMsgAndPublish(vpColVector& velocities_to_apply, std::string velocitie_frame_to_be_published)
        {

            /*Apply the velocity vector compute to a msg
             * You have the possibility to publish this msg in different frame using tf
             * input: velocities to apply to msg and frame to be published, by default the velocities are compute in camera frame "left_arm_camera_link"
             * output:msg to be published
             * */
            msg_vel_output.header.stamp = ros::Time::now();
            msg_vel_output.header.frame_id = "left_arm_camera_link"; //the frame where the velocities are computed
            // v[0], v[1], v[2] correspond to translation velocities in m/s
            // v[3], v[4], v[5] correspond to rotation velocities in rad/s
            msg_vel_output.twist.linear.x = velocities_to_apply[0];
            msg_vel_output.twist.linear.y = velocities_to_apply[1];
            msg_vel_output.twist.linear.z = velocities_to_apply[2];
            msg_vel_output.twist.angular.x = velocities_to_apply[3];
            msg_vel_output.twist.angular.y = velocities_to_apply[4];
            msg_vel_output.twist.angular.z = velocities_to_apply[5];

            //std::cout << msg_vel_output <<std::endl;

            if (velocitie_frame_to_be_published != "left_arm_camera_link") {
                geometry_msgs::TwistStamped vel_tf; //if vel vector is transformed, will be stored here

                geometry_msgs::Vector3Stamped linear_in;
                linear_in.header.stamp = ros::Time::now();
                linear_in.header.frame_id = "left_arm_camera_link";
                linear_in.vector = msg_vel_output.twist.linear;
                geometry_msgs::Vector3Stamped angular_in;
                angular_in.header.stamp = ros::Time::now();
                angular_in.header.frame_id = "left_arm_camera_link";
                angular_in.vector = msg_vel_output.twist.angular;
                geometry_msgs::Vector3Stamped linear_out;
                geometry_msgs::Vector3Stamped angular_out;

                try {
                    listener.waitForTransform(velocitie_frame_to_be_published, "left_arm_camera_link", ros::Time::now(),
                                              ros::Duration(0.1));
                    listener.transformVector(velocitie_frame_to_be_published, linear_in, linear_out);
                    listener.transformVector(velocitie_frame_to_be_published, angular_in, angular_out);
                }
                catch (tf::TransformException ex) {
                    ROS_ERROR("%s", ex.what());
                    ros::Duration(1.0).sleep();
                }
                vel_tf.header.stamp = ros::Time::now();
                vel_tf.header.frame_id = velocitie_frame_to_be_published;
                vel_tf.twist.linear = linear_out.vector;
                vel_tf.twist.angular = angular_out.vector;

                msg_vel_output = vel_tf;
            }

            pub_vel.publish(msg_vel_output);
        }

        void IBVS_mbot::publish_error()
        {
            pub_error.publish(error);
        }

        void IBVS_mbot::PublishGripperCommand()
        {
            /* Publish command to close the gripper**/

            std_msgs::Float64 command;
            command.data = command_gripper;
            pub_gripper.publish(command);
        }

        void IBVS_mbot::CreateSubscriber()
        {
            /* Create Subscriber when synchronized launched syncCallback, except RGB subscribes in ROS grabber function v*/

            depth_img_sub_.subscribe(pnh_, DepthImage_Topic, 2);
            object_array_sub_.subscribe(pnh_, DetectionResultsYolo_Topic, 2);
            sync_.reset(
                    new message_filters::Synchronizer<DepthToBBSyncPolicy>(
                            DepthToBBSyncPolicy(10), depth_img_sub_, object_array_sub_
                    )
            );
            sync_->registerCallback(
                    boost::bind(&IBVS_mbot::syncCallback, this, _1, _2)
            );
        }

        void IBVS_mbot::destroySubscribers()
        {
            /* Destroy subscribers*/

            sync_.reset();
            depth_img_sub_.unsubscribe();
            object_array_sub_.unsubscribe();
        }

        void IBVS_mbot::syncCallback(const sensor_msgs::ImageConstPtr &depth,
                                     const darknet_ros_py::RecognizedObjectArrayStampedConstPtr &rec)
        {
            /* Actual callback that is behing called depth and ROI are synchronized */

            for (const auto &obj: rec->objects.objects)
            {
                if (filter_class_)
                {
                    if (obj.class_name != filter_class_name_)
                        continue;
                }
                // Save latest received ROI
                last_received_array_= *rec;
                roi_received_ = true;
            }

            try {
                // Save latest depth received
                last_received_depth_ = cv_bridge::toCvCopy(*depth, sensor_msgs::image_encodings::TYPE_32FC1);
                depth_image_received_ = true;
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("Could not convert from encoding '%s'.", depth->encoding.c_str());
                return;
            }
        }

        void IBVS_mbot::EventInCallBack(const std_msgs::StringConstPtr &msg)
        {
            std_msgs::String e_out;

            if (msg->data == "e_stop")
            {
                ROS_INFO("Received event to stop - shut down all subscribers");
                e_out.data = "e_success";
                destroySubscribers();
                // flag to stop
                is_running_ = false;
            }
            else if (msg->data == "e_start")
            {
                ROS_INFO("Received event to start - created all subscribers");
                e_out.data = "e_success";
                CreateSubscriber();
                // flag to start
                is_running_ = true;
            }
            else
            {
                ROS_WARN_STREAM("Wrong event received");
                e_out.data = "e_failure";
            }

            event_out_pub_.publish(e_out);
        }

        void IBVS_mbot::loop()
        {
            double iter = 0;

            ros::Rate loop_rate(node_frequency);
            ROS_INFO("Node will run at : %lf [hz]", IBVS_mbot::node_frequency);

            if (plot && Display)
            {
                plotter.init(2, 250 * 2, 500, 100, 200, "Real time curves plotter");
                ROS_INFO("Curves will be plot");
                CreatePlot_Features_CartesianVelocities(plotter);
            }

            while (ros::ok()) {
                if (is_running_) {
                    if (Yolo_Center_ROI)
                    {
                        if (roi_received_ && depth_image_received_)
                        {
                            Init_CenterBoundingBox();
                            init_done = true;
                        }
                    }
                    else if (Display && Blob_tracking)
                    {
                        Init_Dot_BlobTracking();
                        init_done = true;
                    }
                    else if (!Blob_tracking && !Yolo_Center_ROI)
                    {
                        return;
                    }
                    if (init_done) {
                        while (error.data > error_max)
                        {
                            g.acquire(I);
                            if (Display)
                                vpDisplay::display(I);

                            if (Blob_tracking)
                            {
                                UpdateCurrentFeatures_Dot_Blob_Tracking();
                                update_done =true;
                            }
                            else if (Yolo_Center_ROI)
                            {
                                if (roi_received_ && depth_image_received_)
                                {
                                    UpdateCurrentFeatures_CenterBoundingBox();
                                    update_done = true;
                                    depth_image_received_ = false;
                                    roi_received_ = false;
                                }
                            }
                            if (update_done)
                            {
                                vel = task.computeControlLaw(); // Compute the control law
//                                std::cout << "before tf" << vel<<std::endl;
                                if (Display)
                                    vpServoDisplay::display(task, cam, I);
                                //Compute the quadratique error
                                error.data = task.getError().sumSquare();
                                std::cout << "\n" << "error quadratic: " << error << "\n" << std::endl;

                                ArmEndEffectorVelocitiesLimitation(end_effector_velocities_max, vel);
                                vel = TransformFromVispCameraFrameToMbotCameraFrame(vel);
//                                std::cout << "after tf:"<< vel <<std::endl;
                                if (!depth_error)
                                    ApplyVelocitiesInMsgAndPublish(vel, "left_arm_camera_link");

                                publish_error();

                                if (Display && plot)
                                {
                                    plotter.plot(0, iter, task.getError());
                                    plotter.plot(1, iter, vel);
                                }

                                update_done = false;
                                iter++;

                                if (Display)
                                    vpDisplay::flush(I);

                                if (Display && vpDisplay::getClick(I, false))
                                {
                                    task.print();
                                    return;
                                }// A click in the viewer to exit

                                loop_rate.sleep();
                            }
                            else if (Yolo_Center_ROI && !roi_received_)
                            {
                                if (Display)
                                    vpDisplay::flush(I);
                                loop_rate.sleep();
                            }
                            else if (Yolo_Center_ROI && !depth_image_received_)
                            {
                                if (Display)
                                    vpDisplay::flush(I);
                                loop_rate.sleep();
                            }

                        }
                        PublishGripperCommand();
                        task.print();
                        task.kill();
                        ros::spinOnce();
                        if (plot && Display)
                        {
                            if (save)
                            {
                                plotter.saveData(0, "./plot_saved/error.dat");
                                plotter.saveData(1, "./plot_saved/vc.dat");
                            }
                            vpDisplay::getClick(plotter.I);
                        }
                        return;
                    }
                }
            }
        }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ibvs_node");

    VS_IBVS::IBVS_mbot mbot_ibvs;
    ROS_INFO("Node initialized");
    mbot_ibvs.GrabberImageFromRos();
    if (mbot_ibvs.Display)
        vpDisplayOpenCV d(mbot_ibvs.I);

    mbot_ibvs.loop();
    ros::shutdown();
}