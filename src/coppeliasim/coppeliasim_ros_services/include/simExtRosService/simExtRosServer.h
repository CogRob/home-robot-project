// This file is part of the ROS PLUGIN for V-REP
// 
// Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// A big thanks to Svetlin Penkov for his precious help!
// 
// The ROS PLUGIN is licensed under the terms of GNU GPL:
// 
// -------------------------------------------------------------------
// The ROS PLUGIN is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// THE ROS PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with the ROS PLUGIN.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was updated for Coppeliasim release V4.0.0 

#ifndef ROS_SERVER_H
#define ROS_SERVER_H

#include "simExtRosService/simExtRosSubscriber.h"

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <image_transport/image_transport.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include "coppeliasim_msgs_srvs/VisionSensorDepthBuff.h"
#include "coppeliasim_msgs_srvs/ForceSensorData.h"
#include "coppeliasim_msgs_srvs/ProximitySensorData.h"
#include "coppeliasim_msgs_srvs/VisionSensorData.h"
#include "coppeliasim_msgs_srvs/VrepInfo.h"
#include "coppeliasim_msgs_srvs/ObjectGroupData.h"
// #include "coppeliasim_msgs_srvs/ScriptFunctionCallData.h"

// API services:
#include "coppeliasim_msgs_srvs/simRosAddStatusbarMessage.h"
#include "coppeliasim_msgs_srvs/simRosAuxiliaryConsoleClose.h"
#include "coppeliasim_msgs_srvs/simRosAuxiliaryConsoleOpen.h"
#include "coppeliasim_msgs_srvs/simRosAuxiliaryConsolePrint.h"
#include "coppeliasim_msgs_srvs/simRosAuxiliaryConsoleShow.h"
#include "coppeliasim_msgs_srvs/simRosBreakForceSensor.h"
#include "coppeliasim_msgs_srvs/simRosClearFloatSignal.h"
#include "coppeliasim_msgs_srvs/simRosClearIntegerSignal.h"
#include "coppeliasim_msgs_srvs/simRosClearStringSignal.h"
#include "coppeliasim_msgs_srvs/simRosCloseScene.h"
#include "coppeliasim_msgs_srvs/simRosCopyPasteObjects.h"
#include "coppeliasim_msgs_srvs/simRosDisplayDialog.h"
#include "coppeliasim_msgs_srvs/simRosEndDialog.h"
#include "coppeliasim_msgs_srvs/simRosEraseFile.h"
#include "coppeliasim_msgs_srvs/simRosGetArrayParameter.h"
#include "coppeliasim_msgs_srvs/simRosGetBooleanParameter.h"
#include "coppeliasim_msgs_srvs/simRosGetCollisionHandle.h"
// #include "coppeliasim_msgs_srvs/simRosGetCollectionHandle.h"
#include "coppeliasim_msgs_srvs/simRosGetDialogInput.h"
#include "coppeliasim_msgs_srvs/simRosGetDialogResult.h"
#include "coppeliasim_msgs_srvs/simRosGetDistanceHandle.h"
#include "coppeliasim_msgs_srvs/simRosGetFloatingParameter.h"
#include "coppeliasim_msgs_srvs/simRosGetFloatSignal.h"
#include "coppeliasim_msgs_srvs/simRosGetIntegerParameter.h"
#include "coppeliasim_msgs_srvs/simRosGetIntegerSignal.h"
#include "coppeliasim_msgs_srvs/simRosGetJointMatrix.h"
#include "coppeliasim_msgs_srvs/simRosGetJointState.h"
#include "coppeliasim_msgs_srvs/simRosGetLastErrors.h"
#include "coppeliasim_msgs_srvs/simRosGetModelProperty.h"
#include "coppeliasim_msgs_srvs/simRosGetObjectChild.h"
#include "coppeliasim_msgs_srvs/simRosGetObjectFloatParameter.h"
#include "coppeliasim_msgs_srvs/simRosGetObjectHandle.h"
#include "coppeliasim_msgs_srvs/simRosGetObjectIntParameter.h"
//#include "coppeliasim_msgs_srvs/simRosGetObjectOrientation.h"
#include "coppeliasim_msgs_srvs/simRosGetObjectParent.h"
//#include "coppeliasim_msgs_srvs/simRosGetObjectPosition.h"
#include "coppeliasim_msgs_srvs/simRosGetObjectPose.h"
#include "coppeliasim_msgs_srvs/simRosGetObjects.h"
#include "coppeliasim_msgs_srvs/simRosGetObjectSelection.h"
#include "coppeliasim_msgs_srvs/simRosGetStringParameter.h"
#include "coppeliasim_msgs_srvs/simRosGetStringSignal.h"
#include "coppeliasim_msgs_srvs/simRosGetUIButtonProperty.h"
#include "coppeliasim_msgs_srvs/simRosGetUIEventButton.h"
#include "coppeliasim_msgs_srvs/simRosGetUIHandle.h"
#include "coppeliasim_msgs_srvs/simRosGetUISlider.h"
#include "coppeliasim_msgs_srvs/simRosGetVisionSensorDepthBuffer.h"
#include "coppeliasim_msgs_srvs/simRosGetVisionSensorImage.h"
#include "coppeliasim_msgs_srvs/simRosLoadModule.h"
#include "coppeliasim_msgs_srvs/simRosUnloadModule.h"
#include "coppeliasim_msgs_srvs/simRosLoadModel.h"
#include "coppeliasim_msgs_srvs/simRosLoadScene.h"
#include "coppeliasim_msgs_srvs/simRosLoadUI.h"
#include "coppeliasim_msgs_srvs/simRosPauseSimulation.h"
#include "coppeliasim_msgs_srvs/simRosReadCollision.h"
#include "coppeliasim_msgs_srvs/simRosReadDistance.h"
#include "coppeliasim_msgs_srvs/simRosReadForceSensor.h"
#include "coppeliasim_msgs_srvs/simRosReadProximitySensor.h"
#include "coppeliasim_msgs_srvs/simRosReadVisionSensor.h"
#include "coppeliasim_msgs_srvs/simRosRemoveObject.h"
#include "coppeliasim_msgs_srvs/simRosRemoveModel.h"
#include "coppeliasim_msgs_srvs/simRosRemoveUI.h"
#include "coppeliasim_msgs_srvs/simRosSetArrayParameter.h"
#include "coppeliasim_msgs_srvs/simRosSetBooleanParameter.h"
#include "coppeliasim_msgs_srvs/simRosSetFloatingParameter.h"
#include "coppeliasim_msgs_srvs/simRosSetFloatSignal.h"
#include "coppeliasim_msgs_srvs/simRosSetIntegerParameter.h"
#include "coppeliasim_msgs_srvs/simRosSetIntegerSignal.h"
#include "coppeliasim_msgs_srvs/simRosSetJointForce.h"
#include "coppeliasim_msgs_srvs/simRosSetJointPosition.h"
#include "coppeliasim_msgs_srvs/simRosSetJointTargetPosition.h"
#include "coppeliasim_msgs_srvs/simRosSetJointTargetVelocity.h"
#include "coppeliasim_msgs_srvs/simRosSetModelProperty.h"
#include "coppeliasim_msgs_srvs/simRosSetObjectFloatParameter.h"
#include "coppeliasim_msgs_srvs/simRosSetObjectIntParameter.h"
#include "coppeliasim_msgs_srvs/simRosSetObjectPose.h"
#include "coppeliasim_msgs_srvs/simRosSetObjectParent.h"
#include "coppeliasim_msgs_srvs/simRosSetObjectPosition.h"
#include "coppeliasim_msgs_srvs/simRosSetObjectSelection.h"
#include "coppeliasim_msgs_srvs/simRosGetInfo.h"
#include "coppeliasim_msgs_srvs/simRosSetSphericalJointMatrix.h"
#include "coppeliasim_msgs_srvs/simRosSetStringSignal.h"
#include "coppeliasim_msgs_srvs/simRosAppendStringSignal.h"
#include "coppeliasim_msgs_srvs/simRosSetUIButtonLabel.h"
#include "coppeliasim_msgs_srvs/simRosSetUIButtonProperty.h"
#include "coppeliasim_msgs_srvs/simRosSetUISlider.h"
#include "coppeliasim_msgs_srvs/simRosSetVisionSensorImage.h"
#include "coppeliasim_msgs_srvs/simRosStartSimulation.h"
#include "coppeliasim_msgs_srvs/simRosStopSimulation.h"
#include "coppeliasim_msgs_srvs/simRosSynchronous.h"
#include "coppeliasim_msgs_srvs/simRosSynchronousTrigger.h"
#include "coppeliasim_msgs_srvs/simRosTransferFile.h"
#include "coppeliasim_msgs_srvs/simRosEnablePublisher.h"
#include "coppeliasim_msgs_srvs/simRosDisablePublisher.h"
//#include "coppeliasim_msgs_srvs/simRosGetObjectQuaternion.h"
#include "coppeliasim_msgs_srvs/simRosSetObjectQuaternion.h"
#include "coppeliasim_msgs_srvs/simRosEnableSubscriber.h"
#include "coppeliasim_msgs_srvs/simRosDisableSubscriber.h"
#include "coppeliasim_msgs_srvs/simRosSetJointState.h"
#include "coppeliasim_msgs_srvs/simRosCreateDummy.h"
#include "coppeliasim_msgs_srvs/simRosGetAndClearStringSignal.h"
#include "coppeliasim_msgs_srvs/simRosGetObjectGroupData.h"
// #include "coppeliasim_msgs_srvs/simRosCallScriptFunction.h"

class SSpecificPublisherData 
{
    public:
        SSpecificPublisherData() {}
        virtual ~SSpecificPublisherData();
};

struct SPublisherData
{
	int cmdID;
	int auxInt1;
	int auxInt2;
	std::string auxStr;
	int publishCnt; // -1=publisher is asleep
	std::string topicName;
	ros::Publisher generalPublisher;
	image_transport::Publisher imagePublisher;
	SSpecificPublisherData* specificPublisherData;
	int dependencyCnt;
};

class ROS_server
{
	public:
		static bool initialize();
		static void shutDown();

		static void instancePass();
		static void simulationAboutToStart();
		static bool mainScriptAboutToBeCalled();
		static void simulationEnded();

		static std::string addPublisher(const char* topicName,int queueSize,int streamCmd,int auxInt1,int auxInt2,const char* auxString,int publishCnt);
		static int removePublisher(const char* topicName,bool ignoreReferenceCounter);
		static int wakePublisher(const char* topicName,int publishCnt);

		static int addSubscriber(const char* topicName,int queueSize,int streamCmd,int auxInt1,int auxInt2,const char* auxString,int callbackTag_before,int callbackTag_after);
		static bool removeSubscriber(int subscriberID);

	private:
		ROS_server() {}; 
		
		static ros::NodeHandle* node;
        static tf::TransformBroadcaster* tf_broadcaster;

		static image_transport::ImageTransport* images_streamer;
		static int imgStreamerCnt;

		static void enableAPIServices();
		static void disableAPIServices();
		static void spinOnce();

		static bool getObjectGroupData(int objectType,int dataType,std::vector<int>& handles,std::vector<int>& intData,std::vector<float>& floatData,std::vector<std::string>& stringData);

		static int getPublisherIndexFromCmd(int streamCmd,int auxInt1,int auxInt2,const char* auxString);
		static int getPublisherIndexFromTopicName(const char* topicName);
		static void removeAllPublishers();
		static bool launchPublisher(SPublisherData& pub,int queueSize);
		static void shutDownPublisher(SPublisherData& pub);
		static void streamAllData();

		static bool streamVisionSensorImage(SPublisherData& pub, const ros::Time & now,bool& publishedSomething);
		static bool streamLaserCloud(SPublisherData& pub, const ros::Time & now,bool& publishedSomething);
		static bool streamDepthSensorCloud(SPublisherData& pub, const ros::Time & now,bool& publishedSomething);

		static std::vector<SPublisherData> publishers;
		static ros::Publisher infoPublisher; // special publisher that is active also when simulation is not running!
		static ros::Publisher clockPublisher; // special publisher that is active also when simulation is not running!

		static void removeAllSubscribers();
		static std::vector<CSubscriberData*> subscribers;
		static int lastSubscriberID;

		static int _handleServiceErrors_start();
		static void _handleServiceErrors_end(int errorReportMode);
		static std::vector<std::string> _last50Errors;

		static bool _waitTriggerEnable;
		static bool _waitForTrigger;

		static int _simulationFrameID;
		

		//===================================================================================================
		//================================== API services ===================================================
		//===================================================================================================

		static ros::ServiceServer simRosAddStatusbarMessageServer;
		static bool simRosAddStatusbarMessageService(coppeliasim_msgs_srvs::simRosAddStatusbarMessage::Request &req,coppeliasim_msgs_srvs::simRosAddStatusbarMessage::Response &res);

		static ros::ServiceServer simRosAuxiliaryConsoleCloseServer;
		static bool simRosAuxiliaryConsoleCloseService(coppeliasim_msgs_srvs::simRosAuxiliaryConsoleClose::Request &req,coppeliasim_msgs_srvs::simRosAuxiliaryConsoleClose::Response &res);

		static ros::ServiceServer simRosAuxiliaryConsoleOpenServer;
		static bool simRosAuxiliaryConsoleOpenService(coppeliasim_msgs_srvs::simRosAuxiliaryConsoleOpen::Request &req,coppeliasim_msgs_srvs::simRosAuxiliaryConsoleOpen::Response &res);

		static ros::ServiceServer simRosAuxiliaryConsolePrintServer;
		static bool simRosAuxiliaryConsolePrintService(coppeliasim_msgs_srvs::simRosAuxiliaryConsolePrint::Request &req,coppeliasim_msgs_srvs::simRosAuxiliaryConsolePrint::Response &res);

		static ros::ServiceServer simRosAuxiliaryConsoleShowServer;
		static bool simRosAuxiliaryConsoleShowService(coppeliasim_msgs_srvs::simRosAuxiliaryConsoleShow::Request &req,coppeliasim_msgs_srvs::simRosAuxiliaryConsoleShow::Response &res);

		static ros::ServiceServer simRosBreakForceSensorServer;
		static bool simRosBreakForceSensorService(coppeliasim_msgs_srvs::simRosBreakForceSensor::Request &req,coppeliasim_msgs_srvs::simRosBreakForceSensor::Response &res);

		static ros::ServiceServer simRosClearFloatSignalServer;
		static bool simRosClearFloatSignalService(coppeliasim_msgs_srvs::simRosClearFloatSignal::Request &req,coppeliasim_msgs_srvs::simRosClearFloatSignal::Response &res);

		static ros::ServiceServer simRosClearIntegerSignalServer;
		static bool simRosClearIntegerSignalService(coppeliasim_msgs_srvs::simRosClearIntegerSignal::Request &req,coppeliasim_msgs_srvs::simRosClearIntegerSignal::Response &res);

		static ros::ServiceServer simRosClearStringSignalServer;
		static bool simRosClearStringSignalService(coppeliasim_msgs_srvs::simRosClearStringSignal::Request &req,coppeliasim_msgs_srvs::simRosClearStringSignal::Response &res);

		static ros::ServiceServer simRosCloseSceneServer;
		static bool simRosCloseSceneService(coppeliasim_msgs_srvs::simRosCloseScene::Request &req,coppeliasim_msgs_srvs::simRosCloseScene::Response &res);

		static ros::ServiceServer simRosCopyPasteObjectsServer;
		static bool simRosCopyPasteObjectsService(coppeliasim_msgs_srvs::simRosCopyPasteObjects::Request &req,coppeliasim_msgs_srvs::simRosCopyPasteObjects::Response &res);

		static ros::ServiceServer simRosDisplayDialogServer;
		static bool simRosDisplayDialogService(coppeliasim_msgs_srvs::simRosDisplayDialog::Request &req,coppeliasim_msgs_srvs::simRosDisplayDialog::Response &res);

		static ros::ServiceServer simRosEndDialogServer;
		static bool simRosEndDialogService(coppeliasim_msgs_srvs::simRosEndDialog::Request &req,coppeliasim_msgs_srvs::simRosEndDialog::Response &res);

		static ros::ServiceServer simRosEraseFileServer;
		static bool simRosEraseFileService(coppeliasim_msgs_srvs::simRosEraseFile::Request &req,coppeliasim_msgs_srvs::simRosEraseFile::Response &res);

		static ros::ServiceServer simRosGetArrayParameterServer;
		static bool simRosGetArrayParameterService(coppeliasim_msgs_srvs::simRosGetArrayParameter::Request &req,coppeliasim_msgs_srvs::simRosGetArrayParameter::Response &res);

		static ros::ServiceServer simRosGetBooleanParameterServer;
		static bool simRosGetBooleanParameterService(coppeliasim_msgs_srvs::simRosGetBooleanParameter::Request &req,coppeliasim_msgs_srvs::simRosGetBooleanParameter::Response &res);

		static ros::ServiceServer simRosGetCollisionHandleServer;
		static bool simRosGetCollisionHandleService(coppeliasim_msgs_srvs::simRosGetCollisionHandle::Request &req,coppeliasim_msgs_srvs::simRosGetCollisionHandle::Response &res);

		// static ros::ServiceServer simRosGetCollectionHandleServer;
		// static bool simRosGetCollectionHandleService(coppeliasim_msgs_srvs::simRosGetCollectionHandle::Request &req,coppeliasim_msgs_srvs::simRosGetCollectionHandle::Response &res);

		static ros::ServiceServer simRosGetDialogInputServer;
		static bool simRosGetDialogInputService(coppeliasim_msgs_srvs::simRosGetDialogInput::Request &req,coppeliasim_msgs_srvs::simRosGetDialogInput::Response &res);

		static ros::ServiceServer simRosGetDialogResultServer;
		static bool simRosGetDialogResultService(coppeliasim_msgs_srvs::simRosGetDialogResult::Request &req,coppeliasim_msgs_srvs::simRosGetDialogResult::Response &res);

		static ros::ServiceServer simRosGetDistanceHandleServer;
		static bool simRosGetDistanceHandleService(coppeliasim_msgs_srvs::simRosGetDistanceHandle::Request &req,coppeliasim_msgs_srvs::simRosGetDistanceHandle::Response &res);

		static ros::ServiceServer simRosGetFloatingParameterServer;
		static bool simRosGetFloatingParameterService(coppeliasim_msgs_srvs::simRosGetFloatingParameter::Request &req,coppeliasim_msgs_srvs::simRosGetFloatingParameter::Response &res);

		static ros::ServiceServer simRosGetFloatSignalServer;
		static bool simRosGetFloatSignalService(coppeliasim_msgs_srvs::simRosGetFloatSignal::Request &req,coppeliasim_msgs_srvs::simRosGetFloatSignal::Response &res);

		static ros::ServiceServer simRosGetIntegerParameterServer;
		static bool simRosGetIntegerParameterService(coppeliasim_msgs_srvs::simRosGetIntegerParameter::Request &req,coppeliasim_msgs_srvs::simRosGetIntegerParameter::Response &res);

		static ros::ServiceServer simRosGetIntegerSignalServer;
		static bool simRosGetIntegerSignalService(coppeliasim_msgs_srvs::simRosGetIntegerSignal::Request &req,coppeliasim_msgs_srvs::simRosGetIntegerSignal::Response &res);

		static ros::ServiceServer simRosGetJointMatrixServer;
		static bool simRosGetJointMatrixService(coppeliasim_msgs_srvs::simRosGetJointMatrix::Request &req,coppeliasim_msgs_srvs::simRosGetJointMatrix::Response &res);

		static ros::ServiceServer simRosGetJointStateServer;
		static bool simRosGetJointStateService(coppeliasim_msgs_srvs::simRosGetJointState::Request &req,coppeliasim_msgs_srvs::simRosGetJointState::Response &res);

		static ros::ServiceServer simRosGetLastErrorsServer;
		static bool simRosGetLastErrorsService(coppeliasim_msgs_srvs::simRosGetLastErrors::Request &req,coppeliasim_msgs_srvs::simRosGetLastErrors::Response &res);

		static ros::ServiceServer simRosGetModelPropertyServer;
		static bool simRosGetModelPropertyService(coppeliasim_msgs_srvs::simRosGetModelProperty::Request &req,coppeliasim_msgs_srvs::simRosGetModelProperty::Response &res);

		static ros::ServiceServer simRosGetObjectChildServer;
		static bool simRosGetObjectChildService(coppeliasim_msgs_srvs::simRosGetObjectChild::Request &req,coppeliasim_msgs_srvs::simRosGetObjectChild::Response &res);

		static ros::ServiceServer simRosGetObjectFloatParameterServer;
		static bool simRosGetObjectFloatParameterService(coppeliasim_msgs_srvs::simRosGetObjectFloatParameter::Request &req,coppeliasim_msgs_srvs::simRosGetObjectFloatParameter::Response &res);

		static ros::ServiceServer simRosGetObjectHandleServer;
		static bool simRosGetObjectHandleService(coppeliasim_msgs_srvs::simRosGetObjectHandle::Request &req,coppeliasim_msgs_srvs::simRosGetObjectHandle::Response &res);

		static ros::ServiceServer simRosGetObjectIntParameterServer;
		static bool simRosGetObjectIntParameterService(coppeliasim_msgs_srvs::simRosGetObjectIntParameter::Request &req,coppeliasim_msgs_srvs::simRosGetObjectIntParameter::Response &res);

//		static ros::ServiceServer simRosGetObjectOrientationServer;
//		static bool simRosGetObjectOrientationService(coppeliasim_msgs_srvs::simRosGetObjectOrientation::Request &req,coppeliasim_msgs_srvs::simRosGetObjectOrientation::Response &res);

		static ros::ServiceServer simRosGetObjectParentServer;
		static bool simRosGetObjectParentService(coppeliasim_msgs_srvs::simRosGetObjectParent::Request &req,coppeliasim_msgs_srvs::simRosGetObjectParent::Response &res);

//		static ros::ServiceServer simRosGetObjectPositionServer;
//		static bool simRosGetObjectPositionService(coppeliasim_msgs_srvs::simRosGetObjectPosition::Request &req,coppeliasim_msgs_srvs::simRosGetObjectPosition::Response &res);

		static ros::ServiceServer simRosGetObjectPoseServer;
		static bool simRosGetObjectPoseService(coppeliasim_msgs_srvs::simRosGetObjectPose::Request &req,coppeliasim_msgs_srvs::simRosGetObjectPose::Response &res);

		static ros::ServiceServer simRosGetObjectsServer;
		static bool simRosGetObjectsService(coppeliasim_msgs_srvs::simRosGetObjects::Request &req,coppeliasim_msgs_srvs::simRosGetObjects::Response &res);

		static ros::ServiceServer simRosGetObjectSelectionServer;
		static bool simRosGetObjectSelectionService(coppeliasim_msgs_srvs::simRosGetObjectSelection::Request &req,coppeliasim_msgs_srvs::simRosGetObjectSelection::Response &res);

		static ros::ServiceServer simRosGetStringParameterServer;
		static bool simRosGetStringParameterService(coppeliasim_msgs_srvs::simRosGetStringParameter::Request &req,coppeliasim_msgs_srvs::simRosGetStringParameter::Response &res);

		static ros::ServiceServer simRosGetStringSignalServer;
		static bool simRosGetStringSignalService(coppeliasim_msgs_srvs::simRosGetStringSignal::Request &req,coppeliasim_msgs_srvs::simRosGetStringSignal::Response &res);

		static ros::ServiceServer simRosGetUIButtonPropertyServer;
		static bool simRosGetUIButtonPropertyService(coppeliasim_msgs_srvs::simRosGetUIButtonProperty::Request &req,coppeliasim_msgs_srvs::simRosGetUIButtonProperty::Response &res);

		static ros::ServiceServer simRosGetUIEventButtonServer;
		static bool simRosGetUIEventButtonService(coppeliasim_msgs_srvs::simRosGetUIEventButton::Request &req,coppeliasim_msgs_srvs::simRosGetUIEventButton::Response &res);

		static ros::ServiceServer simRosGetUIHandleServer;
		static bool simRosGetUIHandleService(coppeliasim_msgs_srvs::simRosGetUIHandle::Request &req,coppeliasim_msgs_srvs::simRosGetUIHandle::Response &res);

		static ros::ServiceServer simRosGetUISliderServer;
		static bool simRosGetUISliderService(coppeliasim_msgs_srvs::simRosGetUISlider::Request &req,coppeliasim_msgs_srvs::simRosGetUISlider::Response &res);

		static ros::ServiceServer simRosGetVisionSensorDepthBufferServer;
		static bool simRosGetVisionSensorDepthBufferService(coppeliasim_msgs_srvs::simRosGetVisionSensorDepthBuffer::Request &req,coppeliasim_msgs_srvs::simRosGetVisionSensorDepthBuffer::Response &res);

		static ros::ServiceServer simRosGetVisionSensorImageServer;
		static bool simRosGetVisionSensorImageService(coppeliasim_msgs_srvs::simRosGetVisionSensorImage::Request &req,coppeliasim_msgs_srvs::simRosGetVisionSensorImage::Response &res);

		static ros::ServiceServer simRosLoadModelServer;
		static bool simRosLoadModelService(coppeliasim_msgs_srvs::simRosLoadModel::Request &req,coppeliasim_msgs_srvs::simRosLoadModel::Response &res);

		static ros::ServiceServer simRosLoadSceneServer;
		static bool simRosLoadSceneService(coppeliasim_msgs_srvs::simRosLoadScene::Request &req,coppeliasim_msgs_srvs::simRosLoadScene::Response &res);

		static ros::ServiceServer simRosLoadUIServer;
		static bool simRosLoadUIService(coppeliasim_msgs_srvs::simRosLoadUI::Request &req,coppeliasim_msgs_srvs::simRosLoadUI::Response &res);

		static ros::ServiceServer simRosPauseSimulationServer;
		static bool simRosPauseSimulationService(coppeliasim_msgs_srvs::simRosPauseSimulation::Request &req,coppeliasim_msgs_srvs::simRosPauseSimulation::Response &res);

		static ros::ServiceServer simRosReadCollisionServer;
		static bool simRosReadCollisionService(coppeliasim_msgs_srvs::simRosReadCollision::Request &req,coppeliasim_msgs_srvs::simRosReadCollision::Response &res);

		static ros::ServiceServer simRosReadDistanceServer;
		static bool simRosReadDistanceService(coppeliasim_msgs_srvs::simRosReadDistance::Request &req,coppeliasim_msgs_srvs::simRosReadDistance::Response &res);

		static ros::ServiceServer simRosReadForceSensorServer;
		static bool simRosReadForceSensorService(coppeliasim_msgs_srvs::simRosReadForceSensor::Request &req,coppeliasim_msgs_srvs::simRosReadForceSensor::Response &res);

		static ros::ServiceServer simRosReadProximitySensorServer;
		static bool simRosReadProximitySensorService(coppeliasim_msgs_srvs::simRosReadProximitySensor::Request &req,coppeliasim_msgs_srvs::simRosReadProximitySensor::Response &res);

		static ros::ServiceServer simRosReadVisionSensorServer;
		static bool simRosReadVisionSensorService(coppeliasim_msgs_srvs::simRosReadVisionSensor::Request &req,coppeliasim_msgs_srvs::simRosReadVisionSensor::Response &res);

		static ros::ServiceServer simRosRemoveObjectServer;
		static bool simRosRemoveObjectService(coppeliasim_msgs_srvs::simRosRemoveObject::Request &req,coppeliasim_msgs_srvs::simRosRemoveObject::Response &res);

		static ros::ServiceServer simRosRemoveModelServer;
		static bool simRosRemoveModelService(coppeliasim_msgs_srvs::simRosRemoveModel::Request &req,coppeliasim_msgs_srvs::simRosRemoveModel::Response &res);

		static ros::ServiceServer simRosRemoveUIServer;
		static bool simRosRemoveUIService(coppeliasim_msgs_srvs::simRosRemoveUI::Request &req,coppeliasim_msgs_srvs::simRosRemoveUI::Response &res);

		static ros::ServiceServer simRosSetArrayParameterServer;
		static bool simRosSetArrayParameterService(coppeliasim_msgs_srvs::simRosSetArrayParameter::Request &req,coppeliasim_msgs_srvs::simRosSetArrayParameter::Response &res);

		static ros::ServiceServer simRosSetBooleanParameterServer;
		static bool simRosSetBooleanParameterService(coppeliasim_msgs_srvs::simRosSetBooleanParameter::Request &req,coppeliasim_msgs_srvs::simRosSetBooleanParameter::Response &res);

		static ros::ServiceServer simRosSetFloatingParameterServer;
		static bool simRosSetFloatingParameterService(coppeliasim_msgs_srvs::simRosSetFloatingParameter::Request &req,coppeliasim_msgs_srvs::simRosSetFloatingParameter::Response &res);

		static ros::ServiceServer simRosSetFloatSignalServer;
		static bool simRosSetFloatSignalService(coppeliasim_msgs_srvs::simRosSetFloatSignal::Request &req,coppeliasim_msgs_srvs::simRosSetFloatSignal::Response &res);

		static ros::ServiceServer simRosSetIntegerParameterServer;
		static bool simRosSetIntegerParameterService(coppeliasim_msgs_srvs::simRosSetIntegerParameter::Request &req,coppeliasim_msgs_srvs::simRosSetIntegerParameter::Response &res);

		static ros::ServiceServer simRosSetIntegerSignalServer;
		static bool simRosSetIntegerSignalService(coppeliasim_msgs_srvs::simRosSetIntegerSignal::Request &req,coppeliasim_msgs_srvs::simRosSetIntegerSignal::Response &res);

		static ros::ServiceServer simRosSetJointForceServer;
		static bool simRosSetJointForceService(coppeliasim_msgs_srvs::simRosSetJointForce::Request &req,coppeliasim_msgs_srvs::simRosSetJointForce::Response &res);

		static ros::ServiceServer simRosSetJointPositionServer;
		static bool simRosSetJointPositionService(coppeliasim_msgs_srvs::simRosSetJointPosition::Request &req,coppeliasim_msgs_srvs::simRosSetJointPosition::Response &res);

		static ros::ServiceServer simRosSetJointTargetPositionServer;
		static bool simRosSetJointTargetPositionService(coppeliasim_msgs_srvs::simRosSetJointTargetPosition::Request &req,coppeliasim_msgs_srvs::simRosSetJointTargetPosition::Response &res);

		static ros::ServiceServer simRosSetJointTargetVelocityServer;
		static bool simRosSetJointTargetVelocityService(coppeliasim_msgs_srvs::simRosSetJointTargetVelocity::Request &req,coppeliasim_msgs_srvs::simRosSetJointTargetVelocity::Response &res);

		static ros::ServiceServer simRosSetModelPropertyServer;
		static bool simRosSetModelPropertyService(coppeliasim_msgs_srvs::simRosSetModelProperty::Request &req,coppeliasim_msgs_srvs::simRosSetModelProperty::Response &res);

		static ros::ServiceServer simRosSetObjectFloatParameterServer;
		static bool simRosSetObjectFloatParameterService(coppeliasim_msgs_srvs::simRosSetObjectFloatParameter::Request &req,coppeliasim_msgs_srvs::simRosSetObjectFloatParameter::Response &res);

		static ros::ServiceServer simRosSetObjectIntParameterServer;
		static bool simRosSetObjectIntParameterService(coppeliasim_msgs_srvs::simRosSetObjectIntParameter::Request &req,coppeliasim_msgs_srvs::simRosSetObjectIntParameter::Response &res);

		static ros::ServiceServer simRosSetObjectPoseServer;
		static bool simRosSetObjectPoseService(coppeliasim_msgs_srvs::simRosSetObjectPose::Request &req,coppeliasim_msgs_srvs::simRosSetObjectPose::Response &res);

		static ros::ServiceServer simRosSetObjectParentServer;
		static bool simRosSetObjectParentService(coppeliasim_msgs_srvs::simRosSetObjectParent::Request &req,coppeliasim_msgs_srvs::simRosSetObjectParent::Response &res);

		static ros::ServiceServer simRosSetObjectPositionServer;
		static bool simRosSetObjectPositionService(coppeliasim_msgs_srvs::simRosSetObjectPosition::Request &req,coppeliasim_msgs_srvs::simRosSetObjectPosition::Response &res);

		static ros::ServiceServer simRosSetObjectSelectionServer;
		static bool simRosSetObjectSelectionService(coppeliasim_msgs_srvs::simRosSetObjectSelection::Request &req,coppeliasim_msgs_srvs::simRosSetObjectSelection::Response &res);

		static ros::ServiceServer simRosGetInfoServer;
		static bool simRosGetInfoService(coppeliasim_msgs_srvs::simRosGetInfo::Request &req,coppeliasim_msgs_srvs::simRosGetInfo::Response &res);

		static ros::ServiceServer simRosSetSphericalJointMatrixServer;
		static bool simRosSetSphericalJointMatrixService(coppeliasim_msgs_srvs::simRosSetSphericalJointMatrix::Request &req,coppeliasim_msgs_srvs::simRosSetSphericalJointMatrix::Response &res);

		static ros::ServiceServer simRosSetStringSignalServer;
		static bool simRosSetStringSignalService(coppeliasim_msgs_srvs::simRosSetStringSignal::Request &req,coppeliasim_msgs_srvs::simRosSetStringSignal::Response &res);

		static ros::ServiceServer simRosAppendStringSignalServer;
		static bool simRosAppendStringSignalService(coppeliasim_msgs_srvs::simRosAppendStringSignal::Request &req,coppeliasim_msgs_srvs::simRosAppendStringSignal::Response &res);

		static ros::ServiceServer simRosSetUIButtonLabelServer;
		static bool simRosSetUIButtonLabelService(coppeliasim_msgs_srvs::simRosSetUIButtonLabel::Request &req,coppeliasim_msgs_srvs::simRosSetUIButtonLabel::Response &res);

		static ros::ServiceServer simRosSetUIButtonPropertyServer;
		static bool simRosSetUIButtonPropertyService(coppeliasim_msgs_srvs::simRosSetUIButtonProperty::Request &req,coppeliasim_msgs_srvs::simRosSetUIButtonProperty::Response &res);

		static ros::ServiceServer simRosSetUISliderServer;
		static bool simRosSetUISliderService(coppeliasim_msgs_srvs::simRosSetUISlider::Request &req,coppeliasim_msgs_srvs::simRosSetUISlider::Response &res);

		static ros::ServiceServer simRosSetVisionSensorImageServer;
		static bool simRosSetVisionSensorImageService(coppeliasim_msgs_srvs::simRosSetVisionSensorImage::Request &req,coppeliasim_msgs_srvs::simRosSetVisionSensorImage::Response &res);

		static ros::ServiceServer simRosStartSimulationServer;
		static bool simRosStartSimulationService(coppeliasim_msgs_srvs::simRosStartSimulation::Request &req,coppeliasim_msgs_srvs::simRosStartSimulation::Response &res);

		static ros::ServiceServer simRosStopSimulationServer;
		static bool simRosStopSimulationService(coppeliasim_msgs_srvs::simRosStopSimulation::Request &req,coppeliasim_msgs_srvs::simRosStopSimulation::Response &res);

		static ros::ServiceServer simRosSynchronousServer;
		static bool simRosSynchronousService(coppeliasim_msgs_srvs::simRosSynchronous::Request &req,coppeliasim_msgs_srvs::simRosSynchronous::Response &res);

		static ros::ServiceServer simRosSynchronousTriggerServer;
		static bool simRosSynchronousTriggerService(coppeliasim_msgs_srvs::simRosSynchronousTrigger::Request &req,coppeliasim_msgs_srvs::simRosSynchronousTrigger::Response &res);

		static ros::ServiceServer simRosTransferFileServer;
		static bool simRosTransferFileService(coppeliasim_msgs_srvs::simRosTransferFile::Request &req,coppeliasim_msgs_srvs::simRosTransferFile::Response &res);

		static ros::ServiceServer simRosEnablePublisherServer;
		static bool simRosEnablePublisherService(coppeliasim_msgs_srvs::simRosEnablePublisher::Request &req,coppeliasim_msgs_srvs::simRosEnablePublisher::Response &res);

		static ros::ServiceServer simRosDisablePublisherServer;
		static bool simRosDisablePublisherService(coppeliasim_msgs_srvs::simRosDisablePublisher::Request &req,coppeliasim_msgs_srvs::simRosDisablePublisher::Response &res);

//		static ros::ServiceServer simRosGetObjectQuaternionServer;
//		static bool simRosGetObjectQuaternionService(coppeliasim_msgs_srvs::simRosGetObjectQuaternion::Request &req,coppeliasim_msgs_srvs::simRosGetObjectQuaternion::Response &res);

		static ros::ServiceServer simRosSetObjectQuaternionServer;
		static bool simRosSetObjectQuaternionService(coppeliasim_msgs_srvs::simRosSetObjectQuaternion::Request &req,coppeliasim_msgs_srvs::simRosSetObjectQuaternion::Response &res);

		static ros::ServiceServer simRosEnableSubscriberServer;
		static bool simRosEnableSubscriberService(coppeliasim_msgs_srvs::simRosEnableSubscriber::Request &req,coppeliasim_msgs_srvs::simRosEnableSubscriber::Response &res);

		static ros::ServiceServer simRosDisableSubscriberServer;
		static bool simRosDisableSubscriberService(coppeliasim_msgs_srvs::simRosDisableSubscriber::Request &req,coppeliasim_msgs_srvs::simRosDisableSubscriber::Response &res);

		static ros::ServiceServer simRosSetJointStateServer;
		static bool simRosSetJointStateService(coppeliasim_msgs_srvs::simRosSetJointState::Request &req,coppeliasim_msgs_srvs::simRosSetJointState::Response &res);

		static ros::ServiceServer simRosCreateDummyServer;
		static bool simRosCreateDummyService(coppeliasim_msgs_srvs::simRosCreateDummy::Request &req,coppeliasim_msgs_srvs::simRosCreateDummy::Response &res);

		static ros::ServiceServer simRosGetAndClearStringSignalServer;
		static bool simRosGetAndClearStringSignalService(coppeliasim_msgs_srvs::simRosGetAndClearStringSignal::Request &req,coppeliasim_msgs_srvs::simRosGetAndClearStringSignal::Response &res);

		static ros::ServiceServer simRosGetObjectGroupDataServer;
		static bool simRosGetObjectGroupDataService(coppeliasim_msgs_srvs::simRosGetObjectGroupData::Request &req,coppeliasim_msgs_srvs::simRosGetObjectGroupData::Response &res);

		// static ros::ServiceServer simRosCallScriptFunctionServer;
		// static bool simRosCallScriptFunctionService(coppeliasim_msgs_srvs::simRosCallScriptFunction::Request &req,coppeliasim_msgs_srvs::simRosCallScriptFunction::Response &res);

		static ros::ServiceServer simRosLoadModuleServer;
		static bool simRosLoadModuleService(coppeliasim_msgs_srvs::simRosLoadModule::Request &req,coppeliasim_msgs_srvs::simRosLoadModule::Response &res);

		static ros::ServiceServer simRosUnloadModuleServer;
		static bool simRosUnloadModuleService(coppeliasim_msgs_srvs::simRosUnloadModule::Request &req,coppeliasim_msgs_srvs::simRosUnloadModule::Response &res);


};

#endif
