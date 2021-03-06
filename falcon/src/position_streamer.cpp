//////////////////////////////////////////////////////////
// Novint Falcon Kinematics/Dynamics based on R.E.Stamper's PhD(1997)
// with some modifications
//
// Using LibniFalcon Beta 4
//
// Alastair Barrow 26/08/09


#include <iostream>
#include <string>
#include <cmath>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"

#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/kinematic/FalconKinematicStamper.h"

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;

FalconDevice falcon;
ros::Publisher twist_pub;

const float YAW_RIGHT = -1;
const float YAW_LEFT = 1;

//////////////////////////////////////////////////////////
/// Ask libnifalcon to get the Falcon ready for action
/// nothing clever here, straight from the examples
bool initialise()
{
	falcon.setFalconFirmware<FalconFirmwareNovintSDK>();

	cout << "Setting up comm interface for Falcon comms" << endl;

	unsigned int count;
	falcon.getDeviceCount(count);
	cout << "Connected Device Count: " << count << endl;

	//Open the device number:
	int deviceNum = 0;
	cout << "Attempting to open Falcon device:  " << deviceNum << endl;
	if(!falcon.open(deviceNum))
	{
		cout << "Cannot open falcon device index " << deviceNum << " - Lib Error Code: " << falcon.getErrorCode() << " Device Error Code: " << falcon.getFalconComm()->getDeviceErrorCode() << endl;
		return false;
	}
	else
	{
		cout << "Connected to Falcon device " << deviceNum << endl ;
	}

	//Load the device firmware:
	//There's only one kind of firmware right now, so automatically set that.
	falcon.setFalconFirmware<FalconFirmwareNovintSDK>();
	//Next load the firmware to the device

	bool skip_checksum = false;
	//See if we have firmware
	bool firmware_loaded = false;
	firmware_loaded = falcon.isFirmwareLoaded();
	if(!firmware_loaded)
	{
		std::cout << "Loading firmware" << std::endl;
		uint8_t* firmware_block;
		long firmware_size;
		{

			firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
			firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;


			for(int i = 0; i < 10; ++i)
			{
				if(!falcon.getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))

				{
					cout << "Firmware loading try failed";
					//Completely close and reopen
					//falcon.close();
					//if(!falcon.open(m_varMap["device_index"].as<int>()))
					//{
					//	std::cout << "Cannot open falcon device index " << m_varMap["device_index"].as<int>() << " - Lib Error Code: " << m_falconDevice->getErrorCode() << " Device Error Code: " << m_falconDevice->getFalconComm()->getDeviceErrorCode() << std::endl;
					//	return false;
					//}
				}
				else
				{
					firmware_loaded = true;
					break;
				}
			}
		}
	}
	else if(!firmware_loaded)
	{
		std::cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << std::endl;
		//return false;
	}
	else
	{
		//return true;
	}
	if(!firmware_loaded || !falcon.isFirmwareLoaded())
	{
		std::cout << "No firmware loaded to device, cannot continue" << std::endl;
		//return false;
	}
	std::cout << "Firmware loaded" << std::endl;

	//Seems to be important to run the io loop once to be sure of sensible values next time:
	falcon.runIOLoop();

	falcon.getFalconFirmware()->setHomingMode(true);
	//falcon.setFalconKinematic<FalconKinematicStamper>();
  falcon.setFalconKinematic<FalconKinematicStamper>();
	falcon.setFalconGrip<libnifalcon::FalconGripFourButton>();

  return true;
}

void publish_twist(float deadzone, std::array<double, 3>& falcon_pos, float rotation)
{
  geometry_msgs::TwistStamped twist;
  twist.header.stamp = ros::Time::now();
  twist.twist.linear.x = (abs(falcon_pos[1]) > deadzone) ? falcon_pos[1] : 0;
  twist.twist.linear.y = (abs(falcon_pos[0]) > deadzone) ? falcon_pos[0] : 0; 
  twist.twist.linear.z = (abs(falcon_pos[2]) > deadzone) ? falcon_pos[2] : 0; 
  twist.twist.angular.z = rotation;
  twist_pub.publish(twist);
}

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "falcon_stream");
  ros::NodeHandle n;
  twist_pub = n.advertise<geometry_msgs::TwistStamped>("falcon/twist", 1);
	if(!initialise())
		return 0;

  std::array<double, 3> pos;
  /*TODO: move falcon to get min and max encoder values
  ros::Duration offset_duration = ros::Duration(10);
  ros::Time begin = ros::Time::now();
  while(ros::Time::now() - begin < offset_duration)
  {
    
    printf("DURATION\n");
  }
  */
  float z_min = 0.0745;
  float z_max = 0.175;
  float z_mid = z_min + (z_max - z_min)/2.0;

  float x_min = -0.051;
  float x_max = 0.051;
  float x_mid = x_min + (x_max - x_min)/2.0;

  float y_min = -0.053;
  float y_max = 0.053;
  float y_mid = y_min + (y_max - y_min)/2.0;
	
  float rot_z = 0;
  unsigned int grip_btn;

  while(ros::ok())
	{
		//Ask libnifalcon to update the encoder positions and apply any forces waiting:
		falcon.runIOLoop();
    
    rot_z = 0;
    std::array<double, 3> pos = falcon.getPosition();
    
    std::array<double, 3> force;
    pos[0] = pos[0]*1.0/(x_max-x_mid);
    pos[1] = pos[1]*1.0/(y_max-y_mid);
    pos[2] = (pos[2]-z_mid)*1.0/(z_max-z_mid); //Normlize output to 1
    
    grip_btn = falcon.getFalconGrip()->getDigitalInputs(); 
    if(grip_btn & libnifalcon::FalconGripFourButton::PLUS_BUTTON)
    {
      rot_z = YAW_RIGHT;
    } 
    else if(grip_btn & libnifalcon::FalconGripFourButton::MINUS_BUTTON)
    {
      rot_z = YAW_LEFT;
    }

    publish_twist(0.15, pos, rot_z); 
    
    force[0] = -4*pos[0];
    force[1] = -7*pos[1];
    force[2] = -5*pos[2];
    falcon.setForce(force);
	}
  falcon.close();

	return 0;
}


