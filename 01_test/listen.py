from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Once connected, use 'the_connection' to get and send messages
while 1:
    msg = the_connection.recv_match(type='ATTITUDE', blocking=True)
    print(msg)
