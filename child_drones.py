#D2 drone relaying
from pymavlink import mavutil
import time

from dronekit import connect,VehicleMode
import logging


# Set up the logging configuration
logging.basicConfig(filename='mission.log', filemode='w+', level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

logcount = 0

def log_example(type,msg):
    global logcount
    if(logcount>20):
        if(type == "D"):
            logging.debug(msg)
        elif(type == "I"):
            logging.info(msg)
        elif(type == "W"):
            logging.warning(msg)
        elif(type == "E"):
            logging.error(msg)
        logcount = 0
    else:
        logcount = logcount+1

log_example("D","Initializing...")
connection_string = "/dev/serial/by-id/usb-CubePilot_CubeOrange+_1F0047000B51323031393637-if00"#args.connect
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=921600)
vehicle.wait_ready(True, raise_exception=False)
latt = "17.418888"
long ="78.654836"
vehicle.airspeed = 5
vehicle.groundspeed = 50
vehicle.parameters['LAND_SPEED'] = 25 ##Descent speed of 30cm/s
vehicle.parameters["WPNAV_SPEED"]=100
log_example("D","Connecting to device")

vehicle.mode = VehicleMode("STABILIZE")
vehicle.armed = False
log_example("W","Mode changed to STABILIZE")

print(vehicle.version)

# Replace 'serial_port' and 'baudrate' with your actual values
# For example, '/dev/ttyUSB0' for Linux or 'COM3' for Windows
serial_port = '/dev/ttyUSB0'  # Change this to your serial port
baudrate = 115200  # Change this to your baud rate

# Create a MAVLink connection
mav = mavutil.mavlink_connection(serial_port, baud=baudrate)

systemid = 101

print(mav)
if(mav):
    log_example("E","Telemetry not found")

count = 0

rawarmedstate = False
relayingrawarmedstate = False

relayingsendid,relayingsendtargetid,relayingsendcomid,relayingsendconfirm,relayingsendparam1,relayingsendparam2,relayingsendparam3,relayingsendparam4,relayingsendparam5,relayingsendparam6,relayingsendparam7 = 101,0,0,0,0,0,0,0,0,0,0
# Function to send a MAVLink message
def send_mavlink_message():
    global relayingsendid,relayingsendtargetid,relayingsendcomid,relayingsendconfirm,relayingsendparam1,relayingsendparam2,relayingsendparam3,relayingsendparam4,relayingsendparam5,relayingsendparam6,relayingsendparam7

    vmode = 0

    if(vehicle.mode == "STABLIZE"):
        vmode=0
    elif(vehicle.mode == "LOITER"):
        vmode=1
    elif(vehicle.mode == "RTL"):
        vmode=2
    elif(vehicle.mode == "GUIDED"):
        vmode=3
    elif(vehicle.mode == "AUTO"):
        vmode=4


   # Pack and send the custom message
    msg = mav.mav.command_long_encode(
        systemid, 100,  # System ID, trget ID
        0,  # Command ID
        0,  # Confirmation
        vehicle._last_heartbeat, -1 if vehicle.battery.level==None else vehicle.battery.level, vmode, 1 if(vehicle.armed==True) else 0, 0, 0, 0  # Parameters 1-6
    )
    mav.mav.send(msg,force_mavlink1=True)
    log_example("D",F"sent {vehicle._last_heartbeat},{vehicle.battery.level}, {vmode} , {vehicle.armed}")

    msg1 = mav.mav.command_long_encode(
        systemid, 100,  # System ID, Component ID
        1,  # Command ID
        0,  # Confirmation
        vehicle.location.global_relative_frame.alt, 
        vehicle.location.global_relative_frame.lat, 
        vehicle.location.global_relative_frame.lon, 
        vehicle.heading, 
        vehicle.gps_0.satellites_visible, 
        0, 0  # Parameters 1-6
    )
    log_example("D",F"sent {vehicle.location.global_relative_frame.alt},{vehicle.location.global_relative_frame.lat}, {vehicle.location.global_relative_frame.lon} , {vehicle.heading}, {vehicle.gps_0.satellites_visible}")

    mav.mav.send(msg1,force_mavlink1=True)

    if(int(relayingsendcomid) == 55):
        print("relaying command received")
        print(relayingsendcomid)
        print(relayingsendparam1)
        log_example("W","Relayed from "+str(relayingsendid)+" to "+str(relayingsendtargetid)+" for arm state "+str(relayingsendparam2)+" via "+str(systemid))
        relayingsendmsg = mav.mav.command_long_encode(systemid,relayingsendtargetid,relayingsendcomid,relayingsendconfirm,relayingsendparam1,relayingsendparam2,relayingsendparam3,relayingsendparam4,relayingsendparam5,relayingsendparam6,relayingsendparam7)
    else:
        log_example("W",F"Timestamp comd={relayingsendcomid} from {relayingsendid} to {relayingsendtargetid} for arm state {relayingsendparam1} via {systemid}")
        relayingsendmsg = mav.mav.command_long_encode(systemid,relayingsendtargetid,relayingsendcomid,relayingsendconfirm,relayingsendparam1,relayingsendparam2,relayingsendparam3,relayingsendparam4,relayingsendparam5,relayingsendparam6,relayingsendparam7)

    mav.mav.send(relayingsendmsg,force_mavlink1=True)
    
    # print(vehicle._last_heartbeat)

# Function to send a MAVLink message
def resend_mavlink_message(id,targetid,comid,confirm,param1,param2,param3,param4,param5,param6,param7):
    global relayingsendid,relayingsendtargetid,relayingsendcomid,relayingsendconfirm,relayingsendparam1,relayingsendparam2,relayingsendparam3,relayingsendparam4,relayingsendparam5,relayingsendparam6,relayingsendparam7

    relayingsendid =id
    relayingsendtargetid=targetid
    relayingsendcomid = comid
    relayingsendconfirm=confirm
    relayingsendparam1=param1
    relayingsendparam2=param2
    relayingsendparam3=param3
    relayingsendparam4=param4
    relayingsendparam5=param5
    relayingsendparam6=param6
    relayingsendparam7=param7

timestamp = 0
# Function to receive and decode custom messages
def receive_custom_messages():
    global timestamp,rawarmedstate,relayingrawarmedstate
    while True:
        msg = mav.recv_match()  # Receive a message
        if msg:
            # print(msg)  # Print the raw message (for debugging)
            # Decode the message to human-readable format

            decoded_msg = msg.to_dict()
            if(decoded_msg['mavpackettype'] == "COMMAND_LONG"):
                log_example("W",F"Received data from {decoded_msg['target_system']}")
                if(decoded_msg["target_component"]==systemid):
                    
                    if(decoded_msg["command"] == 200):
                        timestamp = int(decoded_msg["param1"])
                    elif(decoded_msg["command"] == 55):
                        print(int(decoded_msg["param1"]) != systemid)
                        print(decoded_msg["param1"])
                        print(decoded_msg)
                        if(int(decoded_msg["param1"]) != systemid):
                            
                            resend_mavlink_message(decoded_msg["target_system"], int(decoded_msg["param1"]),decoded_msg["command"],0,decoded_msg["param1"], decoded_msg["param2"], decoded_msg["param3"], decoded_msg["param4"], decoded_msg["param5"], decoded_msg["param6"], decoded_msg["param7"])

                        else:
                            if(int(decoded_msg["param2"]) == 1):
                                if(relayingrawarmedstate == False):
                                    log_example("D","Relayed Drone Armed")
                                    print("Relayed arm drone")
                                    vehicle.mode = VehicleMode("GUIDED")
                                    vehicle.armed = True

                                relayingrawarmedstate = True

                            else:
                                if(relayingrawarmedstate == True):
                                    log_example("D","Relayed Drone DisArmed")
                                    print("Relayed disarm drone")
                                    vehicle.mode = VehicleMode("STABILIZE")
                                    vehicle.armed = False

                                relayingrawarmedstate = False

                    elif(decoded_msg["command"] == 215):
                        if(int(decoded_msg["param1"]) == 1):
                            if(rawarmedstate == False):
                                log_example("D","Drone Armed")
                                print("arm drone")
                                vehicle.mode = VehicleMode("GUIDED")
                                vehicle.armed = True

                            rawarmedstate = True

                        else:
                            if(rawarmedstate == True):
                                log_example("D","Drone DisArmed")
                                print("disarm drone")
                                vehicle.mode = VehicleMode("STABILIZE")
                                vehicle.armed= False

                            rawarmedstate = False
                    
                # else:
                #     resend_mavlink_message(decoded_msg["target_system"], int(decoded_msg["target_system"]),decoded_msg["command"],0,decoded_msg["param1"], decoded_msg["param2"], decoded_msg["param3"], decoded_msg["param4"], decoded_msg["param5"], decoded_msg["param6"], decoded_msg["param7"])
  
               
                    
                send_mavlink_message() 
                continue
                            # time.sleep(1)
        
        send_mavlink_message()     
        time.sleep(0.1)            
            
        #Send a MAVLink message
        # time.sleep(1)

# Usage example
receive_custom_messages()
print("program ended")
