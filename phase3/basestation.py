
import math
from pathlib import Path
from PIL import Image, ImageTk
from datetime import datetime


# from tkinter import *
# Explicit imports to satisfy Flake8
from tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage,Label, messagebox, ttk
import  tkinter as tk
from tkinter import font
import tkintermapview
import time
import threading
from pymavlink import mavutil

import logging
import os
import matplotlib.pyplot as plt

# path for the database to use
script_directory = os.path.dirname(os.path.abspath(__file__))
database_path = os.path.join(script_directory, "offline_tiles.db")

# Set up the logging configuration
logging.basicConfig(filename='mission.log', filemode='w+', level=logging.WARNING,
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

devicescount = 0


OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH = OUTPUT_PATH / Path(r"/home/pi/Desktop/Base_station/assets/frame0")


def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)


window = Tk()

# window.geometry("1280x860")
screen_width = window.winfo_screenwidth()

screen_height = window.winfo_screenheight() - window.winfo_toplevel().winfo_height()
screen_height = window.winfo_screenheight()-70

print(f"{screen_width}x{screen_height}")
window.geometry(f"{screen_width}x{screen_height}")

window.configure(bg = "#D79275")
logo = tk.PhotoImage(file=ASSETS_PATH / "icon.png")
window.call('wm', 'iconphoto', window._w, logo)
window.title("Drone Relaying Base station")

objectmessages = ["initialized...","starting port"]
objectd1drone = []
objectd2drone=[]

serial_port = '/dev/ttyUSB0'  # Change this to your serial port
baudrate = 115200  # Change this to your baud rate

# Create a MAVLink connection
mav = mavutil.mavlink_connection(serial_port, baud=baudrate)


print(mav)
if(mav != None):
    log_example("D","No telemetry connected")

objectdata = {
    "bsmessages":[],
    "devices":{}
}


map_widget = None

deviceid = -1
timestamp = 0

armingstate = 0

relayingarmingstate = 0
relayingtargetid = 0
destinationtargetid = 0

# Create a list of options for the dropdown
available_vehicles = []
planning_marker=None
plannin_gui_update_status = None
planning_dest_id=None
planning_map_widget = None 
planning_circle_cords=None
planning_dest_marker=None
planning_mission_info = None
planning_mission_info_data = {}
maintain_radius=0


def sendmsgthread(msg):
   mav.mav.send(msg,force_mavlink1=True)

def sendmsg(msg):
    t=threading.Thread(target=sendmsgthread,args=(msg,),daemon=True)
    t.start()

# Function to send a MAVLink message
def send_mavlink_message(deviceid):
    global timestamp,armingstate,planning_mission_info_data
    # print(planning_mission_info_data)

    if(planning_mission_info_data!=None and len(planning_mission_info_data.keys())>0 and "desid" in  planning_mission_info_data):
        if(planning_mission_info_data["missionstatus"] == 1):
           
            msg2 = mav.mav.command_long_encode(
            100, 0 if planning_mission_info_data["desid"] == -1 else planning_mission_info_data["desid"],  # System ID, Component ID
            301,  # Command ID timestamp
            int(planning_mission_info_data["startmission"]),
            round(planning_mission_info_data["initloc"][0],6),  # Confirmation
            round(planning_mission_info_data["initloc"][1],6), 
            round(planning_mission_info_data["destloc"][0],6), 
            round(planning_mission_info_data["destloc"][1],6),
            int(planning_mission_info_data["maxdist"]),
            int(planning_mission_info_data["maintaindist"]), 
            int(planning_mission_info_data["alt"])        # Parameters 1-6
            )
            print("sending mission",msg2)
            sendmsg(msg2)
            # if(planning_mission_info_data["startmission"] == 255):
                # planning_mission_info_data={}
        
        if("relayingdata" in planning_mission_info_data):
           
            msg2 = mav.mav.command_long_encode(
            100, 0 if planning_mission_info_data["desid"] == -1 else planning_mission_info_data["desid"],  # System ID, Component ID
            301,  # Command ID timestamp
            int(planning_mission_info_data["startmission"]),
            round(planning_mission_info_data["initloc"][0],6),  # Confirmation
            round(planning_mission_info_data["initloc"][1],6), 
            round(planning_mission_info_data["destloc"][0],6), 
            round(planning_mission_info_data["destloc"][1],6),
            int(planning_mission_info_data["maxdist"]),
            int(planning_mission_info_data["maintaindist"]), 
            int(planning_mission_info_data["alt"])        # Parameters 1-6
            )
            # print("sending mission",msg2)
            sendmsg(msg2)

    msg1 = mav.mav.command_long_encode(
        100, 0 if deviceid == -1 else deviceid,  # System ID, Component ID
        200,  # Command ID timestamp
        0,  # Confirmation
        timestamp, 0, 0, 0, 0, 0, 0  # Parameters 1-6
    )
    
    sendmsg(msg1)
    log_example("W",F"Timestamp sent to {0 if deviceid == -1 else deviceid}")

    if(relayingtargetid == 0 and deviceid>0):
        msg2 = mav.mav.command_long_encode(
            100, deviceid,  # System ID, Component ID
            215,  # Command ID arming state
            0,  # Confirmation
            objectdata["devices"][deviceid]["setarmstate"], 0, 0, 0, 0, 0, 0  # Parameters 1-6
        )
        sendmsg(msg2)
        log_example("W",F"Drone {deviceid} sent as normal {objectdata['devices'][deviceid]['setarmstate']}")


    if(relayingtargetid != 0):
        relayingmsg = mav.mav.command_long_encode(
            100, relayingtargetid,  # System ID, Component ID
            55,  # Command ID relaying
            0,  # Confirmation
            destinationtargetid,relayingarmingstate, 0, 0, 0, 0, 0  # Parameters 1-6
        )
        sendmsg(relayingmsg)
        log_example("W",F"Drone {relayingtargetid} sent as relaying {relayingarmingstate}")


checkisarmed = 0


pingmeasuring={}
sendrelayingdronethread=None

def sendrelaydronecmd(msg2):
    while True:
        sendmsg(msg2)


# Function to receive and decode custom messages
def receive_custom_messages():
    global deviceid,targetid,armingstate,checkisarmed,pingmeasuring,sendrelayingdronethread
    heartbeat_msg = mavutil.mavlink.MAVLink_heartbeat_message(
        mavutil.mavlink.MAV_TYPE_GCS,  # Type of the system (ground control station)
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # Autopilot type (not used for GCS)
        0,  # System mode (not used for GCS)
        0,  # Custom mode (not used for GCS)
        0,2)  # System status (not used for GCS)

        # Send the heartbeat message over the MAVLink connection
    
    while True:
       
        # mav.mav.send(heartbeat_msg)

            # Get the packed message bytes

        # mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
        #                                     109, 1, 1)

        # msg1 = mav.recv_match(type='RADIO_STATUS', blocking=False)
        # if msg1:
        #     # Extract RSSI value from the received RADIO_STATUS message
        #     rssi = msg1.rssi
        #     print("RSSI Value:", msg1)

        msg = mav.recv_match()  # Receive a message
        if msg:
            # Decode the message to human-readable format
            decoded_msg = msg.to_dict()
            
            # if(decoded_msg["mavpackettype"]=="RADIO_STATUS"):
            #     objectdata["rssi"]  =msg
                # print("got data",msg)
            if(decoded_msg["mavpackettype"]=="COMMAND_LONG"):
                if(decoded_msg["target_component"] == 100):
                    log_example("W",F"Received data from {decoded_msg['target_system']}")
                    

                    deviceid = decoded_msg["target_system"]
                    if(deviceid in objectdata["devices"]):
                        objectdata["devices"][deviceid]["status"] = True
                        if(decoded_msg["command"] == 0):
                            objectdata["devices"][deviceid]["heartbeat"] = decoded_msg["param1"]
                            # objectdata["devices"][deviceid]["battery"] = int(decoded_msg["param2"])
                            objectdata["devices"][deviceid]["mode"] = int(decoded_msg["param3"])
                            objectdata["devices"][deviceid]["timestamp"] = float(decoded_msg["param5"])
                            objectdata["devices"][deviceid]["armstatus"] = int(decoded_msg["param4"])

                            current_timestamp = datetime.now()

                            if(deviceid not in pingmeasuring):
                                pingmeasuring[deviceid]={
                                    "pingtimestamps" :[],
                                    "pingdifferences" :[],
                                    "previoustimestamp":datetime.now()
                                }
                            pingmeasuring[deviceid]['pingtimestamps'].append(current_timestamp)

                            difference =  current_timestamp-pingmeasuring[deviceid]['previoustimestamp']

                            # Get the difference in seconds as an integer
                            difference_in_milliseconds = int(difference.total_seconds() * 1000)

                            # Print the difference in seconds
                            # print(deviceid,"Difference in seconds:", difference_in_milliseconds)
                            pingmeasuring[deviceid]['pingdifferences'].append(difference_in_milliseconds)
                            pingmeasuring[deviceid]['previoustimestamp']=datetime.now()

                            # Plotting the graph
                            plt.plot(pingmeasuring[deviceid]['pingtimestamps'], pingmeasuring[deviceid]['pingdifferences'], marker='o')
                            plt.xlabel('Timestamp')
                            plt.ylabel('Difference (seconds)')
                            plt.title(str(deviceid)+' ping status')
                            plt.xticks(rotation=45)
                            plt.grid(True)
                            plt.tight_layout()

                            # Save the plot as an image file with a specific name
                            plt.savefig(str(deviceid)+'_ping.png')
                            plt.clf()
                            import pandas as pd

                            if(len(pingmeasuring[deviceid]['pingtimestamps'])>10):
                            

                                df = pd.DataFrame({
                                    'Timestamp': pingmeasuring[deviceid]['pingtimestamps'],
                                    'Data': pingmeasuring[deviceid]['pingtimestamps']
                                })
                                

                                df['Data'] = pd.to_numeric(df['Data'], errors='coerce')

                                # Drop rows with NaN values in the 'Data' column (if any)
                                df = df.dropna(subset=['Data'])

                                # Calculate the moving average with a window size of 10
                                df['Moving_Average'] = df['Data'].rolling(window=10).mean()

                                # Display the DataFrame
                                # print(df)

                                # Save the DataFrame with the moving average to a new CSV file
                                output_file = 'output_with_moving_average.csv'
                                df.to_csv(output_file, index=False)

                            
                            # print("arming status",int(decoded_msg["param4"]))
                            if(int(decoded_msg["param4"]) ==1):
                                checkisarmed = 1
                            if(int(decoded_msg["param4"] == 0)):
                                if(checkisarmed == 1):
                                    print(F"{deviceid} mission completed disarming")
                                    targetid = deviceid
                                    armingstate = 0
                                    objectdata["devices"][deviceid]["setarmstate"] = 0
                                    relayingtargetid == 0
                                    checkisarmed = 0
                                

                        elif(decoded_msg["command"] == 1):
                            objectdata["devices"][deviceid]["altitude"] = int(decoded_msg["param1"])
                            if not objectdata["devices"][deviceid]["gps"]:
                                objectdata["devices"][deviceid]["gps"].append(float(decoded_msg["param2"]))
                                objectdata["devices"][deviceid]["gps"].append(float(decoded_msg["param3"]))
                            else:
                                objectdata["devices"][deviceid]["gps"][0] = float(decoded_msg["param2"])
                                objectdata["devices"][deviceid]["gps"][1] = float(decoded_msg["param3"])

                            objectdata["devices"][deviceid]["heading"] = int(decoded_msg["param4"])
                            objectdata["devices"][deviceid]["satellites"] = int(decoded_msg["param5"])
                            objectdata["devices"][deviceid]["armstatus"] = int(decoded_msg["param6"])
                            # print("arming status",int(decoded_msg["param6"]))
                            if(int(decoded_msg["param6"]) ==1):
                                checkisarmed = 1
                            if(int(decoded_msg["param6"] == 0)):
                                if(checkisarmed == 1):
                                    print(F"{deviceid} mission completed disarming")
                                    targetid = deviceid
                                    armingstate = 0
                                    objectdata["devices"][deviceid]["setarmstate"] = 0
                                    relayingtargetid == 0
                        elif(decoded_msg["command"] == 302):
                            print(decoded_msg,planning_mission_info_data)
                            if("relayingdata" not in planning_mission_info_data):
                                planning_mission_info_data['relayingdata']=[]
                            
                            if("desid" in planning_mission_info_data and int(decoded_msg["param1"]) ==  int(planning_mission_info_data['desid'])):
                                devicesids = objectdata["devices"].keys()
                                filtered_devicesids = [int(item) for item in devicesids if int(item) != int(decoded_msg["param1"])]
                                nextid = -1
                                if(len(planning_mission_info_data['relayingdata'])>0):
                                    
                                    for item in planning_mission_info_data['relayingdata']:
                                        if(int(item["destid"]) not in filtered_devicesids):
                                            nextid= int(item)
                                            break
                                else:
                                    nextid = filtered_devicesids[0]

                                if(nextid!=-1):
                                    planning_mission_info_data['relayingdata'].insert(0, {
                                        "destid":nextid,
                                        "lat":float(decoded_msg['param2']),
                                        "lng":float(decoded_msg['param3'])                                    
                                    })

                                vdata = planning_mission_info_data['relayingdata'][0]

                                msg2 = mav.mav.command_long_encode(
                                    100, vdata["destid"],  # System ID, Component ID
                                    303,  # Command ID arming state
                                    0,  # Confirmation
                                    vdata["lat"], vdata["lng"], int(planning_mission_info_data["alt"]), int(vdata["destid"]), int(planning_mission_info_data['desid']), 0, 0  # Parameters 1-6
                                )

                                if(sendrelayingdronethread==None or not sendrelayingdronethread.is_alive()):
                                    sendrelayingdronethread = threading.Thread(target=sendrelaydronecmd,args=(msg2,),daemon=True)
                                    sendrelayingdronethread.start()
                                


                                

                           
                    else:
                        objectdata["devices"][deviceid]={
                                    "status":True,
                                    "timestamp":-1,
                                    "heartbeat":-1,
                                    "previousheartbeat":-1,
                                    "heartbeatcount":0,
                                    # "battery":-1,
                                    "gps":[0,0],
                                    # "attitude":-1,
                                    "altitude":-1,
                                    "armstatus":0,
                                    "heading":-1,
                                    "mode":0,
                                    "satellites":0,
                                    "id": decoded_msg["target_system"],
                                    "setarmstate":0,
                                    "setrelayarmstate":0

                                }
                        
                        
                        # objectdata["devices"][deviceid]["gps"].append(0)
                        # objectdata["devices"][deviceid]["gps"].append(0)

                    # print(str(decoded_msg["target_component"]))
                    
        # if(len(objectdata["devices"].keys())<=0):
        #     send_mavlink_message()
        # time.sleep(0.1)

              
thread = threading.Thread(target=receive_custom_messages,daemon=True)
thread.start()



from collections import defaultdict

connectioncheck = defaultdict(int)

def sendmessages():
    while True:
        keys_to_modify = list(objectdata["devices"].keys())
        # print(keys_to_modify)
        if(len(keys_to_modify)>0):
            for device_idx in keys_to_modify:
                send_mavlink_message(device_idx)
                if objectdata["devices"][device_idx]["previousheartbeat"] == objectdata["devices"][device_idx]["timestamp"]:

                    # if connectioncheck[device_idx] >= 20:
                    #     print(f"{device_idx} disconnected")
                    #     # Remove the device from objectdata
                    #     if(map_widget!=None and device_idx in markershandle):
                    #         markershandle[device_idx].delete()
                    #         markershandle.pop(device_idx)
                    #     del objectdata["devices"][devices_idx]
                    # print("")
                    connectioncheck[device_idx] += 1

                else:
                    connectioncheck[device_idx] = 0  # Reset counter if heartbeat changes
                    if(len(objectdata["devices"].keys())>0):
                        objectdata["devices"][device_idx]["previousheartbeat"] = objectdata["devices"][device_idx]["timestamp"]
            
                
        time.sleep(0.5)



thread = threading.Thread(target=sendmessages,daemon=True)
thread.start()


btnpressstatus = 100

markershandle = {}

d2openphtotbtn = PhotoImage(file=relative_to_assets("d2open.png"))
d1openphotobtn = PhotoImage(file=relative_to_assets("d1open.png"))
msgopenphotobtn = PhotoImage(file=relative_to_assets("msgopen.png"))



def login():
        global objectdata, btnpressstatus, map_widget, secondary_canvas, maincanvas,available_vehicles,planning_dest_id
        global planning_map_widget,planning_circle_cords,planning_dest_marker

        # Check the credentials (you'd replace this with your actual login logic)
        username = entry_1.get()
        password = entry_2.get()
        if username != "user":
            messagebox.showinfo("Alert", "wrong user name")
            return
        elif password != "pass":
            messagebox.showinfo("Alert", "wrong user password")
            return



        # If login successful, hide the login screen elements
        canvas.place_forget()
        entry_1.place_forget()
        entry_2.place_forget()
        button_1.place_forget()

        maincanvas = tk.Canvas(
            window,
            bg="#8ABF93",
            height=screen_height,
            width=screen_width,
            bd=0,
            highlightthickness=0,
            relief="ridge"
        )

        secondary_canvas = tk.Canvas(
            window,
            bg="#8ABF93",
            height=screen_height,
            width=screen_width,
            bd=0,
            highlightthickness=0,
            relief="ridge"
        )

        def update_text(id):
            global btnpressstatus
            btnpressstatus = id

        def show_main_view():
            global plannin_gui_update_status,planning_marker,textarea,objectdata
            if(plannin_gui_update_status!=None):
                    plannin_gui_update_status.set()
                    planning_marker.delete()
            planning_map_widget.set_zoom(0)

            maincanvas.place(x=0, y=0)
            planning_map_widget.place_forget()
            map_widget.place(relx=0.5,
            rely=0.5,
            anchor=tk.CENTER,
            x=175.0,
            y=20.0)
            secondary_canvas.place_forget()
            planning_map_widget.place_forget()
            # maincanvas.itemconfig(textarea, text="\n".join(objectdata["bsmessages"]))


       

        def autoplanning_ui_update(key,stop_event):
            global available_vehicles,plannin_gui_update_status
            while not stop_event.is_set():
                value = objectdata["devices"][key]
                plane_image_main = Image.open(relative_to_assets("drone.png"))
                plane_image_main.resize((80, 80))
                try:
                    rotated_image = plane_image_main.rotate(-value["heading"])  # Negative heading to rotate clockwise
                                
                except Exception as e:
                    print(f"Error rotating image: {e}")
                # plane_image_main.rotate(int(objectdata["devices"][key]["heading"]))

                plane_image = ImageTk.PhotoImage(rotated_image)

                
                if(len(objectdata["devices"].keys())>0 and value and planning_marker!=None):
                    planning_marker.set_text("D1 drone" if key==101 else "D2 drone")
                    planning_marker.change_icon(plane_image)
                    # planning_marker.set_position(value["gps"][0],value["gps"][1])
                    # print(value)


        def on_dropdown_select(event, selected_option):
                global available_vehicles,plannin_gui_update_status,planning_marker,planning_dest_id
                # Handle the selection event
                print(f'Selected option: {selected_option.get()}',type(selected_option.get()),objectdata["devices"])
                key = int(selected_option.get())
                planning_dest_id = key
                keys_to_modify = objectdata["devices"][key]

                if(plannin_gui_update_status!=None):
                    plannin_gui_update_status.set()
                if(planning_marker!=None):
                    planning_marker.delete()
                plane_image_main = Image.open(relative_to_assets("drone.png"))
                plane_image_main.resize((80, 80))
                try:
                    rotated_image = plane_image_main.rotate(-objectdata["devices"][key]["heading"])  # Negative heading to rotate clockwise
                except Exception as e:
                    print(f"Error rotating image: {e}")

                plane_image = ImageTk.PhotoImage(rotated_image)
                planning_map_widget.delete("markers")   
                # print(keys_to_modify)           
                planning_map_widget.set_zoom(20)
                planning_marker=planning_map_widget.set_position(float(keys_to_modify["gps"][0]), float(keys_to_modify["gps"][1]), text="D1 drone", icon=plane_image, marker=True, data=key)
                plannin_gui_update_status = threading.Event()
                threading.Thread(target=autoplanning_ui_update,args=(key,plannin_gui_update_status),daemon=True).start()



        def show_secondary_view(): # show planning view button
            global available_vehicles,planning_map_widget,planning_mission_info_data,planning_mission_info
            global planning_dest_marker,combobox
            keys_to_modify = list(objectdata["devices"].keys())
            if len(keys_to_modify) > 0:
                available_vehicles=keys_to_modify
            
            # Create a StringVar to hold the value of the selected option
            selected_option = tk.StringVar()

            # Create the Combobox widget
            combobox = ttk.Combobox(secondary_canvas, state='readonly',textvariable=selected_option,width=25, values=available_vehicles)
            combobox.place(x=5.0, y= 60.0)

            # Bind the selection event to a handler function
            combobox.bind('<<ComboboxSelected>>', lambda event: on_dropdown_select(event, selected_option))


            maincanvas.place_forget()
            map_widget.place_forget()
            secondary_canvas.itemconfig(planning_mission_info, text="\n".join([f"{key}: {value}" for key, value in planning_mission_info_data.items()]))


            secondary_canvas.place(x=0, y=0)
            planning_map_widget.place(x=175.0, y=20.0, relx=0.5, rely=0.5, anchor=tk.CENTER)

            if(planning_mission_info_data!=None) and len(planning_mission_info_data.keys())>0:
                set_plan_altitude.config(state='normal')
                set_plan_maintain_dist.config(state='normal')
                loadmission.config(state='normal')
                cancelmission.config(state='normal')
                combobox.config(state="normal")

                if(planning_mission_info_data["startmission"]==1):
                    cancelmission.config(state='disabled')
                
                if(planning_mission_info_data["missionstatus"]==1):
                    plane_image_main = Image.open(relative_to_assets("drone.png"))
                    plane_image_main.resize((80, 80))
                    try:
                        rotated_image = plane_image_main.rotate(-objectdata["devices"][planning_mission_info_data["desid"]]["heading"])  # Negative heading to rotate clockwise
                    except Exception as e:
                        print(f"Error rotating image: {e}")

                    plane_image = ImageTk.PhotoImage(rotated_image)
                    
                    selected_option.set(planning_mission_info_data["desid"])
                    if(planning_dest_marker!=None):
                        planning_dest_marker.delete()
                    lat=planning_mission_info_data["destloc"][0]
                    lon=planning_mission_info_data["destloc"][1]
                    planning_dest_marker = planning_map_widget.set_marker(lat, lon, text=f"({lat:.5f}, {lon:.5f})")
                    initlat = objectdata["devices"][planning_mission_info_data["desid"]]['gps'][0]
                    initlng = objectdata["devices"][planning_mission_info_data["desid"]]['gps'][1]
                    planning_map_widget.set_marker(initlat, initlng, text=f"({lat:.5f}, {lon:.5f})")
                    planning_map_widget.delete("markers")              
                
                    planning_map_widget.set_position(initlat, initlng, text="drone", icon=plane_image, marker=True)
                    planning_map_widget.set_zoom(20)

                    radius_var.set(planning_mission_info_data["maintaindist"])
                    # print(planning_mission_info_data)
                    altitude_var.set(planning_mission_info_data["alt"])
                    set_plan_altitude.config(state='readonly')
                    set_plan_maintain_dist.config(state='readonly')
                    loadmission.config(state='disabled')
                    combobox.config(state="disabled")





        maincanvas.place(x=0, y=0)
        maincanvas.create_rectangle(
            0.0, 35.0, 225.0, 200.0,
            fill="#6BE8E1",
            outline="white"
        )

        current_timeview = maincanvas.create_text( #bs timestamp 
            25.350006103515625,
            63.88349914550781,
            anchor="nw",
            text="Timestamp: -1",
            fill="#000000",
            font= font.Font(family="Helvetica", size=12, weight="bold")
        )

        maincanvas.create_text( # Bs id
            25.26873779296875,
            103.49514770507812,
            anchor="nw",
            text="ID : 100",
            fill="#000000",
            font=font.Font(family="Helvetica", size=12, weight="bold")
        )

        connection_count = maincanvas.create_text( # connection count
            25.57501220703125,
            141.126220703125,
            anchor="nw",
            text="Connection count: 2",
            fill="#000000",
            font=font.Font(family="Helvetica", size=12, weight="bold")
        )

        maincanvas.create_text( # signal strength
            25.57501220703125,
            178.7572784423828,
            anchor="nw",
            text="rssi : ?",
            fill="#000000",
            font=font.Font(family="Helvetica", size=12, weight="bold")
        )

        # autonomous planning view 

        secondary_canvas.create_rectangle(
            0.0, 0.0, 1536.0, 35.0,
            fill="#2E56E6",
            outline="white"
        )

        secondary_canvas.create_text(
            17.9532470703125, 5.0,
            anchor="nw",
            text="Base Station",
            fill="#FFFFFF",
            font=font.Font(family="Helvetica", size=20, weight="bold")
        )
        
        sec_main_view_button = Button(
            secondary_canvas,
            text="Action",
            command=lambda: show_main_view(),
            background="blue",
            foreground="white",
            font=font.Font(family="Helvetica", size=10, weight="bold")
        )
        sec_main_view_button.place(
            x=200, y=5,
            width=90, height=25
        )

        sec_secondary_view_button = Button(
            secondary_canvas,
            text="plan",
            command=lambda: show_secondary_view(),
            background="blue",
            foreground="white",
            font=font.Font(family="Helvetica", size=10, weight="bold")
        )
        sec_secondary_view_button.place(
            x=300, y=5,
            width=110, height=25
        )

        planning_map_widget = tkintermapview.TkinterMapView(window, width=screen_width-105, height=screen_height-35, corner_radius=0, use_database_only=False, max_zoom=14, database_path=database_path)
        planning_map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        # planning_map_widget.set_position(17.464435, 78.594023)
        planning_map_widget.set_zoom(2)

        def is_point_inside_polygon(latitude, longitude, polygon):
            if polygon==None:
                return False
            
            n = len(polygon)
            inside = False

            p1_lat, p1_lon = polygon[0]
            for i in range(n + 1):
                p2_lat, p2_lon = polygon[i % n]
                if longitude > min(p1_lon, p2_lon):
                    if longitude <= max(p1_lon, p2_lon):
                        if latitude <= max(p1_lat, p2_lat):
                            if p1_lon != p2_lon:
                                xints = (longitude - p1_lon) * (p2_lat - p1_lat) / (p2_lon - p1_lon) + p1_lat
                            if p1_lat == p2_lat or latitude <= xints:
                                inside = not inside
                p1_lat, p1_lon = p2_lat, p2_lon

            return inside

        # Function to calculate distance between two geographic coordinate points using Haversine formula
        def haversine_distance(lat1, lon1, lat2, lon2):
            # Convert latitude and longitude from degrees to radians
            lat1_rad = math.radians(lat1)
            lon1_rad = math.radians(lon1)
            lat2_rad = math.radians(lat2)
            lon2_rad = math.radians(lon2)

            # Haversine formula
            dlon = lon2_rad - lon1_rad
            dlat = lat2_rad - lat1_rad
            a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            distance = 6371 * 1000 * c  # Earth radius in meters
            return round(distance,5)

        def left_click_event(coordinates_tuple):
            global planning_circle_cords,planning_dest_marker,objectdata,planning_mission_info_data
            global maintain_radius
            print("Left click event with coordinates:", coordinates_tuple)
            lat = coordinates_tuple[0]
            lon=coordinates_tuple[1]
            if(len(planning_mission_info_data.keys())>0) :
                if(planning_mission_info_data["startmission"]==1):
                    messagebox.showinfo("Alert", "Mission started no changes accepted now")
                    return
                if(planning_mission_info_data["missionstatus"]==1):
                    messagebox.showinfo("Alert", "Mission already loaded, Cancel mission for any changes")
                    return
            
            if(is_point_inside_polygon(lat,lon,planning_circle_cords)):
                if(len(set_plan_altitude.get())>0):
                    if(planning_dest_marker!=None):
                        planning_dest_marker.delete()
                    planning_dest_marker = planning_map_widget.set_marker(lat, lon, text=f"({lat:.5f}, {lon:.5f})")
                    geodata = objectdata["devices"][planning_dest_id]["gps"]
                    dist = haversine_distance(geodata[0],geodata[1],lat,lon)

                    planning_map_widget.delete_all_path()
                    planning_map_widget.set_path([(geodata[0], geodata[1]), (lat,lon)],name=str(dist)+" km",width=3)

                    # planning_mission_info
                    planning_mission_info_data["desid"]=int(planning_dest_id)
                    planning_mission_info_data["initloc"]=[geodata[0], geodata[1]]
                    planning_mission_info_data["destloc"]=[lat,lon]
                    planning_mission_info_data["totaldist"]=dist
                    planning_mission_info_data["maxdist"]=maintain_radius*len(objectdata["devices"].keys())
                    planning_mission_info_data["maintaindist"]=maintain_radius
                    planning_mission_info_data["totaldrones"]=len(objectdata["devices"].keys())
                    planning_mission_info_data["missionstatus"]=0
                    planning_mission_info_data["startmission"]=0
                    planning_mission_info_data["alt"]=int(set_plan_altitude.get())
                    secondary_canvas.itemconfig(planning_mission_info, text="\n".join([f"{key}: {value}" for key, value in planning_mission_info_data.items()]))
                else:
                    messagebox.showinfo("Alert", "altitude missing")

            else:
                messagebox.showinfo("Alert", "Destination point is out of boundary")

                
        planning_map_widget.add_left_click_map_command(left_click_event)

        secondary_canvas.create_rectangle(
            0.0, 35.0, 225.0, 700.0,
            fill="#D9D6D6",
            outline="white"     
        )
        secondary_canvas.create_text( #bs timestamp 
            0,
            40,
            anchor="nw",
            text="Choose Destination Vehicle",
            fill="#000000",
            font= font.Font(family="Helvetica", size=12, weight="bold")
        )

        secondary_canvas.create_text( 
            0,
            120,
            anchor="nw",
            text="Enter Altitude",
            fill="#000000",
            font= font.Font(family="Helvetica", size=12, weight="bold")
        )       
       
        altitude_var = tk.StringVar()
        set_plan_altitude = Entry(
            secondary_canvas,
            bd=0,
            bg="#DEA48C",
            fg="#000716",
            font="Century 9 bold",
            highlightthickness=0,
            textvariable=altitude_var
        )
       
        set_plan_altitude.place(
            x=5,
            y=140,
            width=150.0,
            height=40.0
        )
        secondary_canvas.create_text(
            160,
            150,
            anchor="nw",
            text="mts",
            fill="#000000",
            font= font.Font(family="Helvetica", size=12, weight="bold")
        )

        secondary_canvas.create_text( 
            0,
            200,
            anchor="nw",
            text="Maintain Distance",
            fill="#000000",
            font= font.Font(family="Helvetica", size=12, weight="bold")
        )    
        # Function to generate circular polygon coordinates
        # def generate_circle_polygon(center_lat, center_lon, radius_m, num_points=360):
        #     circle_points = []
        #     for i in range(num_points):
        #         angle = math.radians(float(i) * 360 / num_points)
        #         dx = radius_m * math.cos(angle)
        #         dy = radius_m * math.sin(angle)
        #         # Convert meters to degrees
        #         delta_lat = dy / 111320  # There are approximately 111.32 km per degree of latitude
        #         delta_lon = dx / (111320 * math.cos(math.radians(center_lat)))
        #         point_lat = center_lat + delta_lat
        #         point_lon = center_lon + delta_lon
        #         circle_points.append((point_lat, point_lon))
        #     return circle_points
        def generate_circle_polygon(center_lat, center_lon, radius_m, num_points=360):
            circle_points = []
            
            # Earth radius in meters
            earth_radius = 6378137
            
            for i in range(num_points):
                angle = math.radians(float(i) * 360 / num_points)
                dx = radius_m * math.cos(angle)
                dy = radius_m * math.sin(angle)
                
                # Calculate delta latitude and longitude in radians
                delta_lat = dy / earth_radius
                delta_lon = dx / (earth_radius * math.cos(math.radians(center_lat)))
                
                # Convert radians to degrees
                point_lat = center_lat + math.degrees(delta_lat)
                point_lon = center_lon + math.degrees(delta_lon)
                
                circle_points.append((point_lat, point_lon))
            
            return circle_points

        # Function to handle radius input change
        def on_radius_change(val):
            global planning_dest_id,objectdata,planning_map_widget,planning_circle_cords,planning_dest_marker,maintain_radius
            global planning_mission_info_data,planning_mission_info
            # print("surya",val.get(),planning_dest_id)
            if(planning_dest_id!=None):
                value = objectdata['devices'][planning_dest_id]
                num_relayss=len(objectdata["devices"].keys())
                if(planning_dest_marker!=None):
                    planning_dest_marker.delete()

                planning_map_widget.delete_all_path()
                secondary_canvas.itemconfig(planning_mission_info, text="Not available")



                # if(num_relayss>1):
                try:
                    radius_meters = int(val.get())*num_relayss
                    maintain_radius = int(val.get())
                    planning_map_widget.delete_all_polygon()
                    print("suryas",radius_meters,value,num_relayss)
                    if radius_meters > 0 and planning_map_widget!=None:
                        circle_polygon = generate_circle_polygon(value["gps"][0], value["gps"][1], radius_meters)
                        planning_circle_cords = circle_polygon
                        planning_map_widget.set_polygon(circle_polygon, fill_color="blue", outline_color="blue", border_width=2)
                except Exception as e:
                    print(e)  # Ignore invalid input


        radius_var = tk.StringVar()
        # radius_var.trace_add(["write","read"], on_radius_change) 
        radius_var.trace("w", lambda name, index, mode, sv=radius_var: on_radius_change(radius_var))
       

        set_plan_maintain_dist = Entry(
            secondary_canvas,
            bd=0,
            bg="#DEA48C",
            fg="#000716",
            font="Century 9 bold",
            highlightthickness=0,
            textvariable=radius_var
        )
       
        set_plan_maintain_dist.place(
            x=5,
            y=220,
            width=150.0,
            height=40.0
        )
        secondary_canvas.create_text(
            160,
            225,
            anchor="nw",
            text="mts",
            fill="#000000",
            font= font.Font(family="Helvetica", size=12, weight="bold")
        )

        secondary_canvas.create_text( 
            0,
            270,
            anchor="nw",
            text="Mission info",
            fill="#000000",
            font= font.Font(family="Helvetica", size=12, weight="bold")
        )  

        planning_mission_info = secondary_canvas.create_text( 
            5,
            300,
            anchor="nw",
            text="Not available",
            fill="#000000",
            font= font.Font(family="Helvetica", size=12, weight="bold")
        )  

        def loadmissionfun():
            global planning_mission_info_data,secondary_canvas,combobox
            set_plan_altitude.config(state='readonly')
            set_plan_maintain_dist.config(state='readonly')
            loadmission.config(state='disabled')
            combobox.config(state="disabled")
            planning_mission_info_data["missionstatus"]=1
            planning_mission_info_data["startmission"]=0
            secondary_canvas.itemconfig(planning_mission_info, text="\n".join([f"{key}: {value}" for key, value in planning_mission_info_data.items()]))
        
        def cancelmissionfun():
            global planning_mission_info_data,secondary_canvas,combobox
            planning_mission_info_data["startmission"]=255
            planning_mission_info_data["missionstatus"]==0
            set_plan_altitude.config(state='normal')
            set_plan_maintain_dist.config(state='normal')
            loadmission.config(state='normal')
            combobox.config(state="normal")
            secondary_canvas.itemconfig(planning_mission_info, text="Mission canceled")

        

        loadmission = Button(
            secondary_canvas,
            # image=PhotoImage(file=relative_to_assets("d2open.png")),
            borderwidth=0,
            highlightthickness=0,
            command=lambda: loadmissionfun(),
            relief="flat",
            text="Load mission",
            background="green"
        )
        loadmission.place(
            x=5.0,
            y=500.0,
            width=90.6624984741211,
            height=37.38615036010742
        )
        cancelmission = Button(
            secondary_canvas,
            # image=PhotoImage(file=relative_to_assets("d2open.png")),
            borderwidth=0,
            highlightthickness=0,
            command=lambda: cancelmissionfun(),
            relief="flat",
            text="Cancel mission",
            background="red"
        )
        cancelmission.place(
            x=110.0,
            y=500.0,
            width=110.6624984741211,
            height=37.38615036010742
        )


        
        #maincanvas view

        maincanvas.create_text(
            17.9532470703125, 5.0,
            anchor="nw",
            text="Base Station",
            fill="#FFFFFF",
            font=font.Font(family="Helvetica", size=20, weight="bold")
        )

        main_view_button = Button(
            maincanvas,
            text="Action",
            command=lambda: show_main_view(),
            background="blue",
            foreground="white",
            font=font.Font(family="Helvetica", size=10, weight="bold")
        )
        main_view_button.place(
            x=200, y=5,
            width=90, height=25
        )

        secondary_view_button = Button(
            maincanvas,
            text="plan",
            command=lambda: show_secondary_view(),
            background="blue",
            foreground="white",
            font=font.Font(family="Helvetica", size=10, weight="bold")
        )
        secondary_view_button.place(
            x=300, y=5,
            width=110, height=25
        )


        maincanvas.create_rectangle(
            0.0, 200.0, 225.0, 650.0,
            fill="#D9D6D6",
            outline="white"
        )

        maincanvas.create_rectangle(
            0.0, 650.0, 225.0, 1000.0,
            fill="#F4F0F0",
            outline="white"
        )

        behaviourhandledestview = maincanvas.create_text(
            30.0, 690.0,
            anchor="nw",
            text="Destination : null",
            fill="#000000",
            font=font.Font(family="Helvetica", size=10, weight="bold")
        )

        behaviourhandlerelayview = maincanvas.create_text(
            30, 710.0,
            anchor="nw",
            text="Relaying : null",
            fill="#000000",
            font=font.Font(family="Helvetica", size=10, weight="bold")
        )

        def armordisarmclick(state):
            global relayingarmingstate, relayingtargetid, destinationtargetid
            relaying_id = -1
            dest_id = -1

            for key, value in objectdata["devices"].items():
                if "relaying" in value:
                    if value["relaying"] == 1:
                        relaying_id = key
                if "dest" in value:
                    if value["dest"] == 1:
                        dest_id = key

            if relaying_id == -1 or dest_id == -1:
                print("not set relaying and dest ids......")
                return

            if state == 0:
                relayingtargetid = relaying_id
                destinationtargetid = dest_id
                relayingarmingstate = 1
                print(f"arming relay id : {relaying_id} , dest id: {dest_id}, state : {relayingarmingstate}")

            else:
                relayingtargetid = relaying_id
                destinationtargetid = dest_id
                relayingarmingstate = 0
                print(f"arming relay id : {relaying_id} , dest id: {dest_id}, state : {relayingarmingstate}")

        armdrone = Button(
            maincanvas,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: armordisarmclick(0),
            relief="flat",
            text="Takeoff",
            background="green"
        )
        armdrone.place(
            x=30.0, y=730.0,
            width=52.6624984741211,
            height=37.38615036010742
        )

        disarmdrone = Button(
            maincanvas,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: armordisarmclick(1),
            relief="flat",
            text="Land",
            background="green"
        )
        disarmdrone.place(
            x=130.0, y=730.0,
            width=52.6624984741211,
            height=37.38615036010742
        )

        maincanvas.create_text(
            70.0, 790.0,
            anchor="nw",
            text="Mission Action",
            fill="#000000",
            font=font.Font(family="Helvetica", size=12, weight="bold")
        )


        def altermission(param1):
            global planning_mission_info_data

            if(len(planning_mission_info_data.keys())>0):
                if(planning_mission_info_data["missionstatus"]==1):
                    if(param1==0):#start mission
                        if(planning_mission_info_data["startmission"]==0):
                            planning_mission_info_data["startmission"]=1
                        else:
                            messagebox.showinfo("Alert", "Mission already started")
                            
                    elif(param1==1):#stop mission
                        if(planning_mission_info_data["startmission"]==1):
                            planning_mission_info_data["startmission"]=0
                        else:
                            messagebox.showinfo("Alert", "Mission not started yet")

                else:
                    messagebox.showinfo("Alert", "Mission planned but not loaded to vehicles")
            else:
                messagebox.showinfo("Alert", "No mission planned yet")

                

        


        missionstartbtn = Button(
            maincanvas,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: altermission(0),
            relief="flat",
            text="START",
            background="green"
        )
        missionstartbtn.place(
            x=30.0, y=820.0,
            width=52.6624984741211,
            height=37.38615036010742
        )

        stopmissionbtn = Button(
            maincanvas,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: altermission(1),
            relief="flat",
            text="STOP",
            background="red"
        )
        stopmissionbtn.place(
            x=130.0, y=820.0,
            width=52.6624984741211,
            height=37.38615036010742
        )

        maincanvas.create_text(
            70.0, 660.0,
            anchor="nw",
            text="Device status",
            fill="#000000",
            font=font.Font(family="Helvetica", size=12, weight="bold")
        )

        d2openbtn = Button(
            maincanvas,
            image=d2openphtotbtn,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: update_text(102),
            relief="flat"
        )
        d2openbtn.place(
            x=150.0, y=210.0,
            width=70.0,
            height=37.38615036010742
        )

        d1openbtn = Button(
            maincanvas,
            image=d1openphotobtn,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: update_text(101),
            relief="flat",
            text="d1open"
        )
        d1openbtn.place(
            x=75, y=210,
            width=70.0,
            height=37.059417724609375
        )

        msgopenbtn = Button(
            maincanvas,
            borderwidth=0,
            image=msgopenphotobtn,
            highlightthickness=0,
            command=lambda: update_text(100),
            relief="flat",
            text="msg open"
        )
        msgopenbtn.place(
            x=0.0, y=210,
            width=70.0,
            height=37.059417724609375
        )

        # create map widget
        map_widget = tkintermapview.TkinterMapView(window, width=screen_width-105, height=screen_height-35, corner_radius=0, use_database_only=False, max_zoom=14, database_path=database_path)
        map_widget.place(
            relx=0.5,
            rely=0.5,
            anchor=tk.CENTER,
            x=175.0,
            y=20.0
        )

        map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        map_widget.set_position(17.464435, 78.594023)
        map_widget.set_zoom(21)

        textarea = maincanvas.create_text(
            0,
            250.0,
            anchor="nw",
            text="Init messages of BS",
            fill="#080808",
            font=font.Font(family="Helvetica", size=12, weight="bold")
        )

        def setdronerelaying(state,data):
            global targetid,armingstate 
            if(state == 0):
                for key, value in objectdata["devices"].items():
                    objectdata["devices"][data]["relaying"] = 0
                objectdata["devices"][data]["relaying"] = 1
                objectdata["devices"][data]["dest"] = 0
                maincanvas.itemconfig(behaviourhandlerelayview, text=F"Relaying : {('D1' if(data==101) else 'D2')}")

            elif(state == 1):
                for key, value in objectdata["devices"].items():
                    objectdata["devices"][data]["dest"] = 0
                objectdata["devices"][data]["dest"] = 1
                objectdata["devices"][data]["relaying"] = 0
                maincanvas.itemconfig(behaviourhandledestview, text=F"Destination : {('D1' if(data==101) else 'D2')}")
            elif(state == 2):
                print(F"{data} arming")
                objectdata["devices"][data]["setarmstate"] = 1
                targetid = data
                armingstate = 1
                relayingtargetid == 0

            elif(state == 3):
                print(F"{data} disarming")
                targetid = data
                armingstate = 0
                objectdata["devices"][data]["setarmstate"] = 0
                relayingtargetid == 0
        
        maincanvas.itemconfig(textarea, text="\n".join(objectdata["bsmessages"]))

        def get_timestamp():
            global timestamp
            while True:
                if btnpressstatus == 100:
                    maincanvas.itemconfig(textarea, text="\n".join(objectdata["bsmessages"]))
                else:
                    if btnpressstatus in objectdata["devices"]:
                        maincanvas.itemconfig(textarea, text="\n".join([f"{key}:{value}" for key, value in objectdata["devices"][btnpressstatus].items()]))
                    else:
                        maincanvas.itemconfig(textarea, text="D1 not yet found" if btnpressstatus == 101 else "D2 not yet found")

                keys_to_modify = list(objectdata["devices"].keys())
                if len(keys_to_modify) > 0:
                    for key in keys_to_modify:
                        value = objectdata["devices"][key]

                        plane_image_main = Image.open(relative_to_assets("drone.png"))
                        plane_image_main.resize((80, 80))
                        try:
                            rotated_image = plane_image_main.rotate(-objectdata["devices"][key]["heading"])  # Negative heading to rotate clockwise
                        except Exception as e:
                            print(f"Error rotating image: {e}")

                        plane_image = ImageTk.PhotoImage(rotated_image)
                        if key not in markershandle:
                            def contextmenu(marker):
                                # Create a context menu
                                context_menu = tk.Menu(window, tearoff=0)
                                context_menu.add_command(label="Set Relay", command=lambda attr=marker.data: setdronerelaying(0, attr))
                                context_menu.add_separator()
                                context_menu.add_command(label="Set Dest", command=lambda attr=marker.data: setdronerelaying(1, attr))
                                context_menu.add_separator()
                                context_menu.add_command(label="Takeoff", command=lambda attr=marker.data: setdronerelaying(2, attr))
                                context_menu.add_separator()
                                context_menu.add_command(label="Land", command=lambda attr=marker.data: setdronerelaying(3, attr))
                                context_menu.add_separator()
                                # context_menu.add_command(label="Menu Item 3", command=buttonclick,cnf={"id":marker.data})
                                canvasaxis = markershandle[key].get_canvas_pos(marker.position)
                                context_menu.tk_popup(int(canvasaxis[0])+300, int(canvasaxis[1])+70)
                                print(f"marker clicked - text: {marker.text}  position: {marker.position} : {marker.data} :{marker.data}")

                            markershandle[key] = map_widget.set_position(objectdata["devices"][key]["gps"][0], objectdata["devices"][key]["gps"][1], text="D1 drone", icon=plane_image, marker=True, command=contextmenu, data=key)

                        else:
                            if len(objectdata["devices"].keys()) > 0 and objectdata["devices"][key]:
                                markershandle[key].set_text("D1 drone" if key == 101 else "D2 drone")
                                markershandle[key].change_icon(plane_image)
                                markershandle[key].set_position(objectdata["devices"][key]["gps"][0], objectdata["devices"][key]["gps"][1])

                timestamp = int(time.time())
                maincanvas.itemconfig(current_timeview, text=f"Timestamp: {str(timestamp)}")
                maincanvas.itemconfig(connection_count, text=f"Connection count: {str(len(objectdata['devices'].keys()))}")
                time.sleep(0.1)

        thread = threading.Thread(target=get_timestamp,daemon=True)
        thread.start()
        update_text(100)




# login page code from here ...
        
canvas = Canvas(
    window,
    bg = "#D79275",
    height = screen_height,
    width = screen_width-40,
    bd = 0,
    highlightthickness = 0,
    relief = "ridge"
)

canvas.place(x = 0, y = 0)
image_image_1 = PhotoImage(
    file=relative_to_assets("image_1.png"))
image_1 = canvas.create_image(
    230.0,
    400.0,
    image=image_image_1
)

entry_image_1 = PhotoImage(
    file=relative_to_assets("entry_1.png"))
entry_bg_1 = canvas.create_image(
    984.0,
    338.0,
    image=entry_image_1
)
entry_1 = Entry(
    bd=0,
    bg="#DEA48C",
    fg="#000716",
    font="Century 16 bold",
    highlightthickness=0
)
entry_1.place(
    x=792.0,
    y=309.0,
    width=384.0,
    height=56.0
)

entry_image_2 = PhotoImage(
    file=relative_to_assets("entry_2.png"))
entry_bg_2 = canvas.create_image(
    984.0,
    471.0,
    image=entry_image_2
)
entry_2 = Entry(
    bd=0,
    bg="#DEA48C",
    fg="#000716",
    highlightthickness=0
)
entry_2.place(
    x=792.0,
    y=442.0,
    width=384.0,
    height=56.0
)

canvas.create_text(
    782.0,
    148.0,
    anchor="nw",
    text="Welcome Back !",
    fill="#000000",
    font=("KaushanScript Regular", 40 * -1)
)

canvas.create_text(
    792.0,
    206.0,
    anchor="nw",
    text="login to continue",
    fill="#FFFFFF",
    font=("Karma SemiBold", 20 * -1)
)

canvas.create_text(
    782.0,
    278.0,
    anchor="nw",
    text="Username",
    fill="#000000",
    font=("Karma SemiBold", 20 * -1)
)

canvas.create_text(
    782.0,
    411.0,
    anchor="nw",
    text="Password",
    fill="#000000",
    font=("Karma SemiBold", 20 * -1)
)

button_image_1 = PhotoImage(
    file=relative_to_assets("loginbtn.png"))
button_1 = Button(
    image=button_image_1,
    borderwidth=0,
    highlightthickness=0,
    command=login,
    relief="flat"
)
button_1.place(
    x=782.0,
    y=523.0,
    width=404.0,
    height=58.0
)
window.resizable(False, False)
window.mainloop()
