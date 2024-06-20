#D2 drone relaying
import asyncio
import math
import sys
from pymavlink import mavutil
import time

from dronekit import connect,VehicleMode,LocationGlobalRelative
import logging
import threading


systemid = 102


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
connection_string = "/dev/serial/by-id/usb-CubePilot_CubeOrange+_290027000851323031393637-if00"#args.connect
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=921600)
vehicle.wait_ready(True, raise_exception=False)
latt = "17.418888"
long ="78.654836"
vehicle.airspeed = 5
vehicle.groundspeed = 50
vehicle.parameters['LAND_SPEED'] = 25 ##Descent speed of 30cm/s
vehicle.parameters["WPNAV_SPEED"]=75
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


targetAltitude = 1.5 #meters

mission_data = None

print(mav)
if(not mav):
    log_example("E","Telemetry not found")
    # sys.exit()


class handlemission:

    def __init__(self,vehicle,mav):
        self.data=None
        self.vehicle = vehicle
        self.mav = mav
        self.missionstatus=-1
        self.reachedmaxdist=False
        self.currentloc=None
        self.currentmission={}
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run_thread,daemon=True)
        self._thread.start()
        
    def _run_thread(self):
            asyncio.run(self.startmission())

    def update(self,data):
        self.data=data
    
    def get_distance_meters(self,targetLocation,currentLocation):
        dLat=targetLocation.lat - currentLocation.lat
        dLon=targetLocation.lon - currentLocation.lon

        return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5
    # def get_distance_meters(self,targetLocation, currentLocation):
        
    #     lat1, lon1, lat2, lon2 = map(math.radians, [currentLocation.lat, currentLocation.lon, targetLocation.lat, targetLocation.lon])

    #     # Radius of the Earth in meters
    #     R = 6371000

    #     # Convert coordinates from degrees to radians
    #     phi1 = math.radians(lat1)
    #     phi2 = math.radians(lat2)
    #     delta_phi = math.radians(lat2 - lat1)
    #     delta_lambda = math.radians(lon2 - lon1)

    #     # Haversine formula
    #     a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    #     c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    #     return R * c

    def arm_and_takeoff(self,aTargetAltitude):
        
        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
        
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def vehiclego(self,latt,long,alt):
        point1 = LocationGlobalRelative(float(latt),float(long), alt)
        distanceToTargetLocation = self.get_distance_meters(point1,self.vehicle.location.global_relative_frame)

        self.vehicle.simple_goto(point1)
        while True:
            currentDistance = self.get_distance_meters(point1,self.vehicle.location.global_relative_frame)
            print("current distance: ", currentDistance,distanceToTargetLocation,currentDistance<distanceToTargetLocation*.02)
            new_lat, new_lon = self.cal_relaying_loc(self.vehicle.home_location.lat, self.vehicle.home_location.lon, self.vehicle.location.global_relative_frame.lat, self.vehicle.location.global_relative_frame.lon, 5)
            msg = mav.mav.command_long_encode(
                systemid, 255,  # System ID, trget ID
                305,  # Command ID
                1,  # Confirmation
                new_lat, 
                new_lon, 
                alt,
                0, 0, 0 ,0 # Parameters 1-6
                                            )
            sendmsg(msg)
            # print(self.vehicle.home_location,self.vehicle.location.global_relative_frame,new_lat, new_lon)

            current_altitude = vehicle.location.global_relative_frame.alt
            # print("Current Altitude: ", current_altitude)
                                            
            # Adjust the altitude if it's outside the tolerance range
            if abs(current_altitude - alt) > 0.5:
                self.vehicle.simple_takeoff(alt)
                        # Descending can be trickier and may need a more refined control
                        # Implement a descending logic if needed
            # self.vehicle.simple_takeoff(alt)
            
            if currentDistance<=0.5:
                print("Reached target location.")
                # time.sleep(2)
                break
        return self.vehicle.location.global_relative_frame

        
    def sendendmission(self):
        while True:
            msg = mav.mav.command_long_encode(
                 systemid, 255,  # System ID, trget ID
                                                400,  # Command ID
                                                1,  # Confirmation
                                                0, 
                                                0, 
                                                0, 
                                                0,
                                                0, 0, 0  # Parameters 1-6
                                            )
            sendmsg(msg)

    def landvehicle(self):
        self.vehicle.mode = VehicleMode("LAND")
        while True:
            # print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt <=0.74:
                print("Reached ground")
                break

        self.vehicle.armed = False
        endmissionthread=threading.Thread(target=self.sendendmission,daemon=True)
        endmissionthread.start()
        endmissionthread.join()
    
    def stop(self):
        print(f"Stopping thread for {self.name}")
        self._stop_event.set()
        self._thread.join()
        print(f"Thread for {self.name} has stopped")

    def initial_bearing(self,lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)

        x = math.sin(delta_lambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)

        return (math.degrees(math.atan2(x, y)) + 360) % 360

    def destination_point(self,lat1, lon1, distance, bearing):
        R = 6371000  # Radius of the Earth in meters
        phi1 = math.radians(lat1)
        lambda1 = math.radians(lon1)
        theta = math.radians(bearing)
        delta = distance / R

        phi2 = math.asin(math.sin(phi1) * math.cos(delta) + math.cos(phi1) * math.sin(delta) * math.cos(theta))
        lambda2 = lambda1 + math.atan2(math.sin(theta) * math.sin(delta) * math.cos(phi1),
                                    math.cos(delta) - math.sin(phi1) * math.sin(phi2))

        lat2 = math.degrees(phi2)
        lon2 = math.degrees(lambda2)

        return (lat2, lon2)

    def intermediate_point(self,point1, point2, distance_meters):
        lat1, lon1 = point1
        lat2, lon2 = point2
        bearing = self.initial_bearing(lat1, lon1, lat2, lon2)
        new_point = self.destination_point(lat1, lon1, distance_meters, bearing)
        return new_point


    def haversine(self,lat1, lon1, lat2, lon2):
        """
        Calculate the great circle distance between two points 
        on the earth (specified in decimal degrees)
        """
        # Convert decimal degrees to radians 
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Haversine formula 
        dlat = lat2 - lat1 
        dlon = lon2 - lon1 
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a)) 
        r = 6371000  # Radius of earth in meters. Use 3956 for miles. Determines return value units.
        return c * r
       

    def cal_relaying_loc(self,lat1, lon1, lat2, lon2, distance_meters):
    
        # Function to convert degrees to radians
        def degrees_to_radians(degrees):
            return degrees * math.pi / 180
        
        # Function to convert radians to degrees
        def radians_to_degrees(radians):
            return radians * 180 / math.pi
        
        # Earth radius in meters
        earth_radius = 6371000
        
        # Convert latitude and longitude from degrees to radians
        lat1_rad = degrees_to_radians(lat1)
        lon1_rad = degrees_to_radians(lon1)
        lat2_rad = degrees_to_radians(lat2)
        lon2_rad = degrees_to_radians(lon2)
        
        # Calculate bearing from point 1 to point 2
        delta_lon = lon2_rad - lon1_rad
        x = math.cos(lat2_rad) * math.sin(delta_lon)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
        initial_bearing = math.atan2(x, y)
        
        # Adjust the bearing by 180 degrees to get the direction behind the second point
        reverse_bearing = (initial_bearing + math.pi) % (2 * math.pi)
        
        # Calculate new latitude in radians
        new_lat_rad = math.asin(math.sin(lat2_rad) * math.cos(distance_meters / earth_radius) +
                                math.cos(lat2_rad) * math.sin(distance_meters / earth_radius) * math.cos(reverse_bearing))
        
        # Calculate new longitude in radians
        new_lon_rad = lon2_rad + math.atan2(math.sin(reverse_bearing) * math.sin(distance_meters / earth_radius) * math.cos(lat2_rad),
                                            math.cos(distance_meters / earth_radius) - math.sin(lat2_rad) * math.sin(new_lat_rad))
        
        # Convert new latitude and longitude from radians to degrees
        new_lat = radians_to_degrees(new_lat_rad)
        new_lon = radians_to_degrees(new_lon_rad)
        
        return new_lat, new_lon

    async def startmission(self):
        while not self._stop_event.is_set():
            if(self.data != None):
                alt = self.data["param7"]
                startmission = self.data["confirmation"]
                maintaindist = self.data["param6"]
                maxdist = self.data["param5"]
                targetid = self.data["target_component"]
                destlat,destlng = (self.data["param3"],self.data["param4"])

                clat = self.vehicle.location.global_relative_frame.lat
                clon = self.vehicle.location.global_relative_frame.lon

                totaldist=self.haversine(clat,clon,float(destlat),float(destlng))

                if(totaldist>float(maintaindist)):
                    intermediate_lat, intermediate_lon = self.intermediate_point((clat, clon), (float(destlat),float(destlng)), int(maintaindist))
                    self.currentmission["destcords"]=(intermediate_lat,intermediate_lon)
                    self.currentmission["totaldist"]=totaldist
                    self.currentmission["finaldist"]=maxdist
                    self.currentmission["maintaindist"]=maintaindist
                    self.currentmission["vehiclescount"]=int(maxdist)/int(maintaindist)

                if(int(startmission)==1):
                    # print(targetid,self.currentmission['destcords'])
                    if(int(targetid) == systemid):
                            if(self.missionstatus==-1):
                                self.missionstatus = 1
                                self.currentmission["waitforRes"]=False
                                self.arm_and_takeoff(float(alt))
                                while True:
                                    cloc = self.vehiclego(self.currentmission["destcords"][0],self.currentmission["destcords"][1],float(alt))
                                    clatt,clng = (cloc.lat,cloc.lon)
                                    self.currentmission["totaldist"] -=float(self.currentmission["maintaindist"])
                                    print(self.currentmission["totaldist"],self.currentmission["maintaindist"])
                                    if(self.currentmission["totaldist"]-0.5<=0):
                                        break
                                    else:
                                        while(self.currentmission["waitforRes"]==False):
                                            new_lat, new_lon = self.cal_relaying_loc(self.vehicle.home_location.lat, self.vehicle.home_location.lon, clatt, clng, 5)

                                            msg = mav.mav.command_long_encode(
                                                systemid, 100,  # System ID, trget ID
                                                302,  # Command ID
                                                0,  # Confirmation
                                                targetid, 
                                                new_lat, 
                                                new_lon, 
                                                0,
                                                0, 0, 0  # Parameters 1-6
                                            )
                                            sendmsg(msg)
                                            current_altitude = vehicle.location.global_relative_frame.alt
                                            # print("Current Altitude: ", current_altitude)
                                            
                                            # Adjust the altitude if it's outside the tolerance range
                                            if abs(current_altitude - alt) > 0.5:
                                                self.vehicle.simple_takeoff(alt)
                                                    # Descending can be trickier and may need a more refined control
                                                    # Implement a descending logic if needed
                                            # self.vehicle.simple_takeoff(alt)
                                            # print("continue mission")

                                            # print("sent bs request ")
                                            # print("continue mission")
                                        print("continue mission")
                                        self.vehicle.simple_takeoff(alt)  # Take off to target altitude
        
                                        while True:
                                            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                                            # Break and return from function just below target altitude.
                                            if self.vehicle.location.global_relative_frame.alt >= alt * 0.95:
                                                print("Reached target altitude")
                                                break
                                        if(self.currentmission["totaldist"]>=int(maxdist)):
                                            intermediate_lat, intermediate_lon = self.intermediate_point(clatt, clng, float(destlat),float(destlng), int(maintaindist))
                                            self.currentmission["destcords"]=(intermediate_lat,intermediate_lon)
                                        else:
                                            self.currentmission["destcords"]=(destlat,destlng)
                                point1 = LocationGlobalRelative(float(self.vehicle.home_location.lat),float(self.vehicle.home_location.lon), alt)             
                                distanceToTargetLocation = handlemsion.get_distance_meters(vehicle.location.global_relative_frame,point1)
                                print("mission completed",point1,distanceToTargetLocation,self.currentmission['finaldist'])
                                if(distanceToTargetLocation>=totaldist-0.5):
                                    self.landvehicle()

                            
                            # print("mission loaded")
                elif(int(startmission)==255):
                    print("mission cancelled")






handlemsion = handlemission(vehicle,mav=mav)
timestamp = 0
relayingautostatus = 0
relayingthread=None
ignorefollowme=False
sendcontinuemission=None


def landvehicle():
        global rawarmedstate,relayingrawarmedstate
        vehicle.mode = VehicleMode("LAND")
        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt <=0.74:
                print("Reached ground")
                break

        vehicle.armed = False
       
        rawarmedstate = False
        relayingrawarmedstate = False

def arm_takeoff(aTargetAltitude,mode,latt,lng):
        print("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
        
        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
           
        if(mode):
            vehiclego(latt,lng,aTargetAltitude,mode)
        else:    
            landvehicle()
        
def vehiclego(latt,long,alt,mode):
        global ignorefollowme
        point1 = LocationGlobalRelative(float(latt),float(long), alt)
        distanceToTargetLocation = handlemsion.get_distance_meters(point1,vehicle.location.global_relative_frame)

        vehicle.simple_goto(point1)
        while True:
            if(vehicle.location.global_relative_frame.alt<1):
                vehicle.simple_takeoff(alt) 
            currentDistance = handlemsion.get_distance_meters(point1,vehicle.location.global_relative_frame)
            print("current distance: ", currentDistance*.02,distanceToTargetLocation*.02,currentDistance<distanceToTargetLocation*.02)
            if (currentDistance)<0.5:
                print("Reached target location.")
                break
        
        
        return vehicle.location.global_relative_frame    


def sendcontinuemissioncmd(destdroneid):
    global relayingthread,ignorefollowme
    while True:
        if not relayingthread.is_alive():
            print("Thread finished")
            msg = mav.mav.command_long_encode(
                                    systemid, destdroneid,  # System ID, trget ID
                                    304,  # Command ID
                                    1,  # Confirmation
                                    0, 
                                    0, 
                                    0, 
                                    0,
                                    0, 0, 0  # Parameters 1-6
                                )
                                # print(msg)
            sendmsg(msg)
            ignorefollowme=True

count = 0


rawarmedstate = False
relayingrawarmedstate = False

timestampcounter = 0

relayingsendid,relayingsendtargetid,relayingsendcomid,relayingsendconfirm,relayingsendparam1,relayingsendparam2,relayingsendparam3,relayingsendparam4,relayingsendparam5,relayingsendparam6,relayingsendparam7 = 101,0,0,0,0,0,0,0,0,0,0

def sendmsgthread(msg):
    mav.mav.send(msg,force_mavlink1=True)
def sendmsg(msg):
    t=threading.Thread(target=sendmsgthread,args=(msg,),daemon=True)
    t.start()
    

# Function to send a MAVLink message
def send_mavlink_message():
    global timestampcounter,relayingsendid,relayingsendtargetid,relayingsendcomid,relayingsendconfirm,relayingsendparam1,relayingsendparam2,relayingsendparam3,relayingsendparam4,relayingsendparam5,relayingsendparam6,relayingsendparam7

    vmode = 0

    if(str(vehicle.mode).split(":")[1] == "STABLIZE"):
        vmode=0
    elif(str(vehicle.mode).split(":")[1] == "LOITER"):
        vmode=1
    elif(str(vehicle.mode).split(":")[1] == "RTL"):
        vmode=2
    elif(str(vehicle.mode).split(":")[1] == "GUIDED"):
        vmode=3
    elif(str(vehicle.mode).split(":")[1] == "AUTO"):
        vmode=4

   # Pack and send the custom message
    msg = mav.mav.command_long_encode(
        systemid, 100,  # System ID, trget ID
        0,  # Command ID
        0,  # Confirmation
        vehicle._last_heartbeat, 
        (-1 if vehicle.battery.level==None else vehicle.battery.level), 
        vmode, 
        (1 if vehicle.armed else 0),
        timestampcounter, 0, 0  # Parameters 1-6
    )
    timestampcounter+=1
    sendmsg(msg)
    # print(vehicle.armed)
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
        (1 if vehicle.armed else 0), 0  # Parameters 1-6
    )
    log_example("D",F"sent {vehicle.location.global_relative_frame.alt},{vehicle.location.global_relative_frame.lat}, {vehicle.location.global_relative_frame.lon} , {vehicle.heading}, {vehicle.gps_0.satellites_visible}")

    sendmsg(msg1)


    if(int(relayingsendcomid) == 55):
        print("relaying command received")
        # print(relayingsendcomid)
        # print(relayingsendparam1)
        log_example("W","Relayed from "+str(relayingsendid)+" to "+str(relayingsendtargetid)+" for arm state "+str(relayingsendparam2)+" via "+str(systemid))
        relayingsendmsg = mav.mav.command_long_encode(systemid,relayingsendtargetid,relayingsendcomid,relayingsendconfirm,relayingsendparam1,relayingsendparam2,relayingsendparam3,relayingsendparam4,relayingsendparam5,relayingsendparam6,relayingsendparam7)
    else:
        log_example("W",F"Timestamp comd={relayingsendcomid} from {relayingsendid} to {relayingsendtargetid} for arm state {relayingsendparam1} via {systemid}")
        relayingsendmsg = mav.mav.command_long_encode(systemid,relayingsendtargetid,relayingsendcomid,relayingsendconfirm,relayingsendparam1,relayingsendparam2,relayingsendparam3,relayingsendparam4,relayingsendparam5,relayingsendparam6,relayingsendparam7)

    sendmsg(relayingsendmsg)

    
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


# Function to receive and decode custom messages
def receive_custom_messages():
    global timestamp,rawarmedstate,relayingrawarmedstate,mission_data,relayingautostatus
    global relayingthread,ignorefollowme,sendcontinuemission
    while True:
        heartbeat_msg = mavutil.mavlink.MAVLink_heartbeat_message(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,  # Type of the system (ground control station)
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # Autopilot type (not used for GCS)
        0,  # System mode (not used for GCS)
        0,  # Custom mode (not used for GCS)
        0,2)  # System status (not used for GCS)

        # Send the heartbeat message over the MAVLink connection
        # mav.mav.send(heartbeat_msg)

        # # Get the packed message bytes

        # mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
        #                                 109, 1, 1)

        # msg1 = mav.recv_match(type='RADIO_STATUS', blocking=False)
        # if msg1:
        #     # Extract RSSI value from the received RADIO_STATUS message
        #     rssi = msg1.rssi
        #     print("RSSI Value:", msg1)
            
        msg = mav.recv_match()  # Receive a message
        if msg:
            # print(msg)  # Print the raw message (for debugging)
            # Decode the message to human-readable format

            decoded_msg = msg.to_dict()

            if(decoded_msg['mavpackettype'] == "COMMAND_LONG"):
                log_example("W",F"Received data from {decoded_msg['target_system']}")

                if(decoded_msg["command"] == 301):#mission data
                    mission_data = decoded_msg
                    handlemsion.update(mission_data)
                    # print(mission_data)
                if(decoded_msg["target_component"]==255):
                    if(decoded_msg["command"] == 305):
                        if(ignorefollowme==True):
                            print(decoded_msg)
                            if(vehicle.location.global_relative_frame.alt < 1):
                                vehicle.simple_takeoff(int(decoded_msg['param3']))
                            point1 = LocationGlobalRelative(float(decoded_msg['param1']),float(decoded_msg['param2']), int(decoded_msg['param3']))

                            distanceToTargetLocation = handlemsion.get_distance_meters(point1,vehicle.location.global_relative_frame)
                            if(distanceToTargetLocation<=int(handlemsion.currentmission["maintaindist"])):
                                relayingthread = threading.Thread(target=vehiclego,args=(float(decoded_msg['param1']),float(decoded_msg['param2']),int(decoded_msg["param3"]),None),daemon=True)
                                relayingthread.start()
                    if(decoded_msg['command']==400):
                        landvehicle()

                if(decoded_msg["target_component"]==systemid):

                    if(decoded_msg["command"] == 303):
                        destlat = float(decoded_msg["param1"])
                        destlon = float(decoded_msg["param2"])
                        alt = int(decoded_msg['param3'])
                        destdroneid = int(decoded_msg['param5'])

                        

                        if(relayingautostatus == 0):
                            relayingautostatus=1
                            relayingthread = threading.Thread(target=arm_takeoff,args=(alt,1,destlat,destlon,),daemon=True)
                            relayingthread.start()
                            sendcontinuemission = threading.Thread(target=sendcontinuemissioncmd,args=(destdroneid,),daemon=True)
                            sendcontinuemission.start()

                        

                    if(decoded_msg["command"] == 304):
                        if(decoded_msg['confirmation']==1):
                            handlemsion.currentmission["waitforRes"]=True
              


                    
                    # print(decoded_msg)
                    if(decoded_msg["command"] == 200):
                        timestamp = int(decoded_msg["param1"])
                    elif(decoded_msg["command"] == 55):
                        if(int(decoded_msg["param1"]) != systemid):
                            
                            resend_mavlink_message(decoded_msg["target_system"], int(decoded_msg["param1"]),decoded_msg["command"],0,decoded_msg["param1"], decoded_msg["param2"], decoded_msg["param3"], decoded_msg["param4"], decoded_msg["param5"], decoded_msg["param6"], decoded_msg["param7"])

                        else:
                            if(int(decoded_msg["param2"]) == 1):
                                if(relayingrawarmedstate == False):
                                    log_example("D","Relayed Drone Armed")
                                    print("Relayed arm drone")
                                    arm_takeoff(targetAltitude)

                                relayingrawarmedstate = True

                            else:
                                if(relayingrawarmedstate == True):
                                    log_example("D","Relayed Drone DisArmed")
                                    print("Relayed disarm drone")
                                    landvehicle()


                                relayingrawarmedstate = False

                    elif(decoded_msg["command"] == 215):
                        if(int(decoded_msg["param1"]) == 1):
                            if(rawarmedstate == False):
                                log_example("D","Drone Armed")
                                print("arm drone")
                                arm_takeoff(targetAltitude)


                            rawarmedstate = True

                        else:
                            if(rawarmedstate == True):
                                log_example("D","Drone DisArmed")
                                print("disarm drone")
                                landvehicle()

                            rawarmedstate = False
                    
                # else:
                #     resend_mavlink_message(decoded_msg["target_system"], int(decoded_msg["target_system"]),decoded_msg["command"],0,decoded_msg["param1"], decoded_msg["param2"], decoded_msg["param3"], decoded_msg["param4"], decoded_msg["param5"], decoded_msg["param6"], decoded_msg["param7"])
  
               
                    
                # send_mavlink_message() 
                continue
                            # time.sleep(1)
        
        # send_mavlink_message()     
        # time.sleep(0.2)            
            
        #Send a MAVLink message
        # time.sleep(1)

# Usage example


def sendmessages():
    while True:
        send_mavlink_message()
        # time.sleep(0.5)

thread = threading.Thread(target=sendmessages,daemon=True)
thread.start()

receive_custom_messages()
print("program ended")

