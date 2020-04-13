//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>



//AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager(): arm1_{"arm1", "arm2"}/*, arm2_{"arm2"}*/

{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);
    conveyor_part_found  = false;

}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN_STREAM(">>>>> OrderCallback");
    
    received_orders_.push_back(*order_msg);
    for (const auto &order:received_orders_){

        // ROS_INFO_STREAM(received_orders_);
        auto order_id = order.order_id;
        ROS_INFO_STREAM(order_id);
        auto shipments = order.shipments;

        for (const auto &shipment: shipments){
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back();//--this returns a char
            ROS_INFO_STREAM(agv);
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';

            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);
            for (const auto &product: products){
                ROS_INFO_STREAM("Product"<<product);
                // ros::spinOnce();
                // product_frame_list_ = camera_.get_product_frame_list();
                product_type_pose_.first = product.type;
                //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                product_type_pose_.second = product.pose;
                // product_list_type_pose_[product.type] = product.pose;
                productlist_type.push_back(product.type);
                productlist_pose.push_back(product.pose);
                ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
                    }
            for (auto it = productlist_type.cbegin(); it != productlist_type.cend(); it++){
                std::cout << *it << ',' << std::endl;;
                if(*it == "piston_rod_part"){
                    piston_count_++;
                }
                else if(*it == "gear_part"){
                    gear_count_++;
                }
            }
            bin1_part = camera_.LogicalCamera1PartType();
            bin2_part = camera_.LogicalCamera2PartType();
            bin3_part = camera_.LogicalCamera3PartType();
            bin4_part = camera_.LogicalCamera4PartType();
            bin5_part = camera_.LogicalCamera5PartType();
            bin6_part = camera_.LogicalCamera6PartType();
            // ROS_INFO_STREAM("Piston count in order: " << piston_count_);
            // ROS_INFO_STREAM("Gear count in order " << gear_count_);
            ROS_INFO_STREAM("Part read from bin 1: " << bin1_part);
            ROS_INFO_STREAM("Part read from bin 2: " << bin2_part);
            ROS_INFO_STREAM("Part read from bin 3: " << bin3_part);   
            ROS_INFO_STREAM("Part read from bin 4: " << bin4_part);
            ROS_INFO_STREAM("Part read from bin 5: " << bin5_part);
            ROS_INFO_STREAM("Part read from bin 6: " << bin6_part);         
        }
    }
}


/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove itle()
    ROS_WARN_STREAM("Came Here");
    
    if (!product_frame_list_.empty() && product_frame_list_.count(product_type)!=0) {
        std::string frame = product_frame_list_[product_type].back();
        // ROS_INFO_STREAM("Frame >>>> " << frame);
        product_frame_list_[product_type].pop_back();
        return frame;
    } else {
         ROS_ERROR_STREAM("No product frame found for " << product_type);
         ROS_INFO_STREAM("Getting " << product_type << " from the conveyor Belt");
//         arm1_.sendRobotToConveyor();
//         auto belt_parts_ = camera_.belt_parts;

//         if( not belt_parts_.empty() ){
//			 for(auto part_type:belt_parts_){
//				 if( part_type == product_type ) {
//					 conveyor_part_found = true;
//				 }
//			 }
//         }
//         else return "NOO";

//         if(conveyor_part_found){
//        	 bool failed_pick = arm1_.PickPartconveyor(product_type);
//			 while(!failed_pick){
//				 failed_pick = arm1_.PickPartconveyor(product_type);  //robot_controller
//			 }
//		}
//		else return "NOO";

         bool failed_pick = arm1_.PickPartconveyor(product_type);
		 while(!failed_pick){
			 failed_pick = arm1_.PickPartconveyor(product_type);  //robot_controller
		 }
            bin_pose.position.x=-0.2;
            bin_pose.position.y=-0.2;
            bin_pose.position.z=0.1;
            bin_pose.orientation.x=-0.703527;
            bin_pose.orientation.y=0.0254473;
            bin_pose.orientation.z=-0.710205;
            bin_pose.orientation.w=-0.0032328;

            geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;
            StampedPose_in.header.frame_id = "/bin6_frame";
            StampedPose_in.pose = bin_pose;
            part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);

            // Drop in the adjacent bin
            arm1_.DropPart(StampedPose_out.pose);

            ros::spinOnce();
            product_frame_list_ = camera_.get_product_frame_list();
            // dist=dist+0.8;
        // }
            ROS_INFO_STREAM("Length"<<product_frame_list_.size()); 
            std::string frame = product_frame_list_[product_type].back();
            // ROS_INFO_STREAM("Frame >>>> " << frame);
            product_frame_list_[product_type].pop_back();
            return frame;

        
  }

}

bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    std::string product_type = product_type_pose.first;
    // ROS_WARN_STREAM("Product type >>>> " << product_type);
    std::string product_frame = GetProductFrame(product_type);

//    if ( product_frame == "NOO" ) return false;
    // ROS_WARN_STREAM("Product frame >>>> " << product_frame);
    auto part_pose = camera_.GetPartPose("/world", product_frame);


    if(product_type == "pulley_part")
        part_pose.position.z += 0.08;

    //--task the robot to pick up this part again from the bin
    bool failed_pick = arm1_.PickPart(part_pose);

    // ROS_WARN_STREAM("Picking up state " << failed_pick);
    // ros::Duration(0.5).sleep();

    // while(!failed_pick){
    //     auto part_pose = camera_.GetPartPose("/world",product_frame);  //sensor.cpp
    //     failed_pick = arm1_.PickPart(part_pose);  //robot_controller
    // }
     
    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        // ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        // StampedPose_out.pose.position.x += 0.2;
        // StampedPose_out.pose.position.y += 0.2;
        // ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        // StampedPose_out.pose.position.z += 0.1;
        // StampedPose_out.pose.position.y += 0.2;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }

    // This is checking if part is faulty ior not
    bool success = false;
    if( not success) {
    	success = arm1_.DropPart(StampedPose_out.pose); //robot_controller
    }


    return true;

}

void AriacOrderManager::ExecuteOrder() {
    ROS_WARN(">>>>>> Executing order...");
    //scanned_objects_ = camera_.GetParts();

    //-- used to check if pick and place was successful
    bool pick_n_place_success{false};
    int count_piston = 0;
    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;
        ros::spinOnce();
    ros::Duration(1.0).sleep();
    product_frame_list_ = camera_.get_product_frame_list();

    for (const auto &order:received_orders_){

        // ROS_INFO_STREAM(received_orders_);
        auto order_id = order.order_id;
        ROS_INFO_STREAM(order_id);
        auto shipments = order.shipments;

        for (const auto &shipment: shipments){
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back();//--this returns a char
            //-- if agv is any then we use AGV1, else we convert agv id to int
            //--agv-'0' converts '1' to 1 and '2' to 2
            ROS_INFO_STREAM(agv);
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';

            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);
            for (const auto &product: products){
                ROS_INFO_STREAM("Product"<<product);
                // ros::spinOnce();
                // product_frame_list_ = camera_.get_product_frame_list();
                product_type_pose_.first = product.type;
                //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                product_type_pose_.second = product.pose;
                // product_list_type_pose_[product.type] = product.pose;
                // productlist_type_.push_back(product.type);
                // productlist_pose_.push_back(product.pose);
                ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
        
                    // ros::spinOnce();
                    // product_frame_list_ = camera_.get_product_frame_list();
//                bool pick_n_place_success = false;
//                while (not pick_n_place_success){
                pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id);
//                }

                //--todo: What do we do if pick and place fails?
            }
            // for (auto it = productlist_type_.cbegin(); it != productlist_type_.cend(); it++){
            //     std::cout << *it << ',' << std::endl;;
            //     if(*it == "piston_rod_part"){
            //         count_piston++;
            //     }
            // }
            // std::cout << "Piston count " << count_piston << std::endl; 
            // for (std::map<std::string, geometry_msgs::Pose>::iterator it=product_list_type_pose_.begin(); it!=product_list_type_pose_.end(); ++it)
            //     std::cout << it->first << " => " << it->second << '\n';
            SubmitAGV(agv_id);
            ROS_INFO_STREAM("Submitting AGV");
            int finish=1;
        }
    }
}

void AriacOrderManager::SubmitAGV(int num) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        // ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        // ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    // srv.request.kit_type = "order_0_kit_0";
    start_client.call(srv);

    if (!srv.response.success) {
        // ROS_ERROR_STREAM("Service failed!");
    } else{
        // ROS_INFO("Service succeeded.");
    }
}

std::vector<std::string> AriacOrderManager::GetProductType(){
    return productlist_type;
}

std::vector<geometry_msgs::Pose> AriacOrderManager::GetProductPose(){
    return productlist_pose;
}
