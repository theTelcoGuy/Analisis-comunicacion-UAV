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
                                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0,
                                    0, 0, 0, 0, 0, 0)
msg = connect.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)