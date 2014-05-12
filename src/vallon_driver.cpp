/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita and Jorge Fraga on 08/11/2012
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <metal_detector_msgs/Coil.h>
#include <metal_detector_msgs/SetCoilsZero.h>
#include <cereal_port/CerealPort.h>

#include <iostream>
#include <fstream>
#include <string>
#include "math.h"


#define LEFT_COIL 	0
#define MIDDLE_COIL 1
#define RIGHT_COIL 	2

struct coil
{
	int channel[3];
	int zero[3];
};

coil coils[3];

tf::TransformBroadcaster * tf_broadcaster_ptr;
ros::Publisher * coil_pub_ptr;
std::string frame_id;

void streamCallback(std::basic_string<char>* msg)
{
	if(msg->length() != 61)
		return;

	int coil_n = (int)msg->at(7) - 48;

	int signs;

	//CHANNEL 1
	if(msg->at(13) == '+')
		signs = 1;
	if(msg->at(13) == '-')
		signs = -1;

	std::string value_str = msg->substr(14,6);
	int channel_value = 0;

	for(int i = 0; i < 6 ; i++)
	{
		if(value_str[i] > 60 )
		{
			channel_value += (value_str[i] - 55)*pow(16,5-i);
		}
		else
			channel_value += (value_str[i] - 48)*pow(16,5-i);
	}

	coils[coil_n-1].channel[0] = signs*channel_value;

	// CHANNEL 2
	if(msg->at(22) == '+')
		signs = 1;
	if(msg->at(22) == '-')
		signs = -1;

	value_str = msg->substr(23,6);
	channel_value = 0;

	for(int i = 0; i < 6 ; i++)
	{
		if(value_str[i] > 60 )
		{
			channel_value += (value_str[i] - 55)*pow(16,5-i);
		}
		else
			channel_value += (value_str[i] - 48)*pow(16,5-i);
	}

	coils[coil_n-1].channel[1] = signs*channel_value;

	// CHANNEL 3
	if(msg->at(31) == '+')
		signs = 1;
	if(msg->at(31) == '-')
		signs = -1;

	value_str = msg->substr(32,6);
	channel_value = 0;

	for(int i = 0; i < 6 ; i++)
	{
		if(value_str[i] > 60 )
		{
			channel_value += (value_str[i] - 55)*pow(16,5-i);
		}
		else
			channel_value += (value_str[i] - 48)*pow(16,5-i);
	}

	coils[coil_n-1].channel[2] = signs*channel_value;

	// Message data
	ros::Time current_time = ros::Time::now();

	std::string frame = frame_id;
	switch (coil_n-1)
	{
		case MIDDLE_COIL:
            frame = "middle_coil";
			break;
		case LEFT_COIL:
            frame = "left_coil";
			break;
		case RIGHT_COIL:
            frame = "right_coil";
			break;
	}

	// Publish coil data
	metal_detector_msgs::Coil coil_msg;
	
	coil_msg.header.stamp = current_time;
	coil_msg.header.frame_id = frame;

	coil_msg.channel.resize(3);
	coil_msg.zero.resize(3);

	for(int i=0 ; i<3 ; i++)
	{
		coil_msg.channel[i] = coils[coil_n-1].channel[i];
		coil_msg.zero[i] = coils[coil_n-1].zero[i];
	}

	coil_pub_ptr->publish(coil_msg);

    // This is not necessary, this should be taken care of on the URDF
	// Publish the transform over tf
    /*geometry_msgs::TransformStamped coil_transform;

	coil_transform.header.stamp = current_time;
	coil_transform.header.frame_id = frame_id;

	coil_transform.child_frame_id = frame;
		
	coil_transform.transform.translation.x = 0.0;
	switch (coil_n-1)
	{
		case MIDDLE_COIL:
			coil_transform.transform.translation.y = 0.0;
			break;
		case LEFT_COIL:
			coil_transform.transform.translation.y = 0.18;
			break;
		case RIGHT_COIL:
			coil_transform.transform.translation.y = -0.18;
			break;
	}
	coil_transform.transform.translation.z = 0.0;
	coil_transform.transform.rotation = tf::createQuaternionMsgFromYaw(0);
		
	// Send the transform
    tf_broadcaster_ptr->sendTransform(coil_transform);*/
}

bool setCoilsZero(metal_detector_msgs::SetCoilsZero::Request &req, metal_detector_msgs::SetCoilsZero::Response &res)
{
	if(req.coil.size() == 0)
	{
		for(int i=0 ; i<3 ; i++)
		{
			for(int j=0 ; j<3 ; j++)
			{
				coils[i].zero[j] = coils[i].channel[j];
			}
		}

		res.status_message = "Set coils zero was successful using current coil values.";
		return true;
	}
	else if(req.coil.size() == 3)
	{
		for(int i=0 ; i<3 ; i++)
		{
			if(req.coil[i].zero.size() != 3)
			{
				res.status_message = "Set coils zero failed, number of coil channels is invalid.";
				return false;
			}
			for(int j=0 ; j<3 ; j++)
			{
				coils[i].zero[j] = req.coil[i].zero[j];
			}
		}

		res.status_message = "Set coils zero was successful using the provided values.";
		return true;
	}
	else
	{
		res.status_message = "Set coils zero failed, number of coils is invalid.";
		return false;
	}
	
	return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "vallon_vmp3_node");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	tf::TransformBroadcaster tf_broadcaster;
	tf_broadcaster_ptr = &tf_broadcaster;

	ros::Publisher coil_pub = n.advertise<metal_detector_msgs::Coil>("coils", 50);
	coil_pub_ptr = &coil_pub;
	ros::ServiceServer service = n.advertiseService("set_coils_zero", setCoilsZero);

	for(int i=0 ; i<3 ; i++)
	{
		for(int j=0 ; j<3 ; j++)
		{
			coils[i].channel[j] = 0;
			coils[i].zero[j] = 0;
		}
	}

	XmlRpc::XmlRpcValue middle_coil_zero;
    	if( n.getParam("middle_coil_zero", middle_coil_zero) )
    	{   
        	ROS_ASSERT(middle_coil_zero.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
		for(int i=0 ; i<middle_coil_zero.size() ; ++i) 
		{
		    ROS_ASSERT(middle_coil_zero[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
		    coils[0].zero[i] = static_cast<int>(middle_coil_zero[i]);
		}
    	}

	XmlRpc::XmlRpcValue left_coil_zero;
    	if( n.getParam("left_coil_zero", left_coil_zero) )
    	{   
        	ROS_ASSERT(left_coil_zero.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
		for(int i=0 ; i<left_coil_zero.size() ; ++i) 
		{
		    ROS_ASSERT(left_coil_zero[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
		    coils[1].zero[i] = static_cast<int>(left_coil_zero[i]);
		}
    	}

	XmlRpc::XmlRpcValue right_coil_zero;
    	if( n.getParam("right_coil_zero", right_coil_zero) )
    	{   
        	ROS_ASSERT(right_coil_zero.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
		for(int i=0 ; i<right_coil_zero.size() ; ++i) 
		{
		    ROS_ASSERT(right_coil_zero[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
		    coils[2].zero[i] = static_cast<int>(right_coil_zero[i]);
		}
    	}

	pn.param<std::string>("frame_id", frame_id, "metal_detector");

	std::string port;
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
    	int baudrate;
	pn.param("baudrate", baudrate, 115200);

	cereal::CerealPort md_port;
	md_port.open(port.c_str(), baudrate);
	if(!md_port.portOpen())
	{
		ROS_FATAL("Vallon MD -- Failed to open serial port %s at %d baud!", port.c_str(), baudrate);
		ROS_BREAK();
	}
	ROS_INFO("Vallon MD -- Successfully connected to the Vallon MD!");
	
	md_port.startReadBetweenStream(streamCallback, '$', '\n');

	ros::spin();

	md_port.close();
	return(0);
}

// EOF

