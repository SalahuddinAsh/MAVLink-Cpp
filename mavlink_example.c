seri
/* The default UART header for your MCU */
#include "serialUart.h"
#include "/home/pi/uart/mySerial/MAVLink/ardupilotmega/mavlink.h"
 

void mavlink_send()
{
  mavlink_system_t mavlink_system;
 
  mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
  mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
 
  // Define the system type, in this case an airplane
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight


  int fd = serialOpen("/dev/ttyAMA0", 115200);

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t *pbuf = buf;
 
  // Pack the message
  mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  serialWrite(fd, pbuf, len);

  serialClose(fd);
}



void mavlink_receive(void)
{
  uint8_t received = 0;
  
  mavlink_system_t mavlink_system;
 
  mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
  mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
 
  // Define the system type, in this case an airplane
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  static int packet_drops = 0;
  //static int mode = MAV_MODE_UNINIT; // Defined in mavlink_types.h, which is included by mavlink.h
  mavlink_message_t msg;
  mavlink_status_t status;

  int fd = serialOpen("/dev/ttyAMA0", 115200);

  
  uint8_t rxBuf[17];
  uint8_t bytesReceived = 0;
  uint8_t index = 0;
  
  bytesReceived = serialRead(fd, rxBuf, 17);

  if(bytesReceived == 17)
    {
      printf("The message is received .. start parsing .. \n");
      // Try to get a new message
      
      while(index < 17)
	{
	  uint8_t c = rxBuf[index++];
	  printf("Byte %d, 0x%.2x .. ", index, c);
      
	  if(mavlink_parse_char(MAVLINK_COMM_0, rxBuf[index], &msg, &status)) {

	    printf("One message received\n");

	    // Handle message
	    switch(msg.msgid)
	      {
	      case MAVLINK_MSG_ID_HEARTBEAT:
		{
		  printf("Heartbeat message received\n");
		  // E.g. read GCS heartbeat and go into
		  // comm lost mode if timer times out
		}
		break;
				
	      case MAVLINK_MSG_ID_COMMAND_LONG:
		// EXECUTE ACTION
		break;
			
	      default:
		//Do nothing
		break;
	      }
	  }
	  else
	    printf("Not yet!\n");
	}
    }


  /*
  while(serialHasChar(fd) == 0); // wait here until some data are ready to be received
  
  while(received != 1)
    {
      uint8_t ch;
      ch = serialReadChar(fd);
      // Try to get a new message
      if(mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status)) {

	received = 1;

	printf("One message received\n");

	// Handle message
	switch(msg.msgid)
	  {
	  case MAVLINK_MSG_ID_HEARTBEAT:
	    {
	      printf("Heartbeat message received\n");
	      // E.g. read GCS heartbeat and go into
	      // comm lost mode if timer times out
	    }
	    break;
				
	  case MAVLINK_MSG_ID_COMMAND_LONG:
	    // EXECUTE ACTION
	    break;
			
	  default:
	    //Do nothing
	    break;
	  }
      }
      // And get the next one
    }
  */
 
  // Update global packet drops counter
  //packet_drops += status.packet_rx_drop_count;

}

int main(void)
{
  //mavlink_send();
  mavlink_receive();
}
