from pymavlink import mavutil


# Start a connection listening on a UDP port
connect = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
connect.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connect.target_system, connect.target_component))

# Once connected, use 'connect' to get and send messages

#Enable mode GUIDED
connect.mav.command_long_send(connect.target_system, connect.target_component, 
                                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                    4, 0, 0, 0, 0, 0)
msg = connect.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
#ARM the drone
connect.mav.command_long_send(connect.target_system, connect.target_component, 
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
msg = connect.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)