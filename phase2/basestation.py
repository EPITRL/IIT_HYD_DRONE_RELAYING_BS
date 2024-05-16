

from pathlib import Path

from PIL import Image, ImageTk







# from tkinter import *

# Explicit imports to satisfy Flake8

from tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage,Label

import  tkinter as tk

from tkinter import font

import tkintermapview

import time

import threading

from pymavlink import mavutil



import logging

import os



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

# Function to send a MAVLink message

def send_mavlink_message(deviceid):

    global timestamp,armingstate

    msg1 = mav.mav.command_long_encode(

        100, 0 if deviceid == -1 else deviceid,  # System ID, Component ID

        200,  # Command ID timestamp

        0,  # Confirmation

        timestamp, 0, 0, 0, 0, 0, 0  # Parameters 1-6

    )

    

    mav.mav.send(msg1,force_mavlink1=True)

    log_example("W",F"Timestamp sent to {0 if deviceid == -1 else deviceid}")



    if(relayingtargetid == 0 and deviceid>0):

        msg2 = mav.mav.command_long_encode(

            100, deviceid,  # System ID, Component ID

            215,  # Command ID arming state

            0,  # Confirmation

            objectdata["devices"][deviceid]["setarmstate"], 0, 0, 0, 0, 0, 0  # Parameters 1-6

        )

        mav.mav.send(msg2,force_mavlink1=True)

        log_example("W",F"Drone {deviceid} sent as normal {objectdata['devices'][deviceid]['setarmstate']}")





    if(relayingtargetid != 0):

        relayingmsg = mav.mav.command_long_encode(

            100, relayingtargetid,  # System ID, Component ID

            55,  # Command ID relaying

            0,  # Confirmation

            destinationtargetid,relayingarmingstate, 0, 0, 0, 0, 0  # Parameters 1-6

        )

        mav.mav.send(relayingmsg,force_mavlink1=True)

        log_example("W",F"Drone {relayingtargetid} sent as relaying {relayingarmingstate}")





checkisarmed = 0

# Function to receive and decode custom messages

def receive_custom_messages():

    global deviceid,targetid,armingstate,checkisarmed

    while True:

        msg = mav.recv_match()  # Receive a message

        if msg:

            # Decode the message to human-readable format

            # print("got data")

            decoded_msg = msg.to_dict()

            if(decoded_msg["mavpackettype"]=="COMMAND_LONG"):

                # print(decoded_msg)

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

                            print("arming status",int(decoded_msg["param4"]))

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

                            print("arming status",int(decoded_msg["param6"]))

                            if(int(decoded_msg["param6"]) ==1):

                                checkisarmed = 1

                            if(int(decoded_msg["param6"] == 0)):

                                if(checkisarmed == 1):

                                    print(F"{deviceid} mission completed disarming")

                                    targetid = deviceid

                                    armingstate = 0

                                    objectdata["devices"][deviceid]["setarmstate"] = 0

                                    relayingtargetid == 0

                       

                           

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



              

thread = threading.Thread(target=receive_custom_messages)

thread.start()







from collections import defaultdict



connectioncheck = defaultdict(int)



def sendmessages():

    while True:

        keys_to_modify = list(objectdata["devices"].keys())

        print(keys_to_modify)

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

                    print("")

                    connectioncheck[device_idx] += 1



                else:

                    connectioncheck[device_idx] = 0  # Reset counter if heartbeat changes

                    if(len(objectdata["devices"].keys())>0):

                        objectdata["devices"][device_idx]["previousheartbeat"] = objectdata["devices"][device_idx]["timestamp"]

            

                

        time.sleep(0.5)







thread = threading.Thread(target=sendmessages)

thread.start()





btnpressstatus = 100



markershandle = {}



d2openphtotbtn = PhotoImage(file=relative_to_assets("d2open.png"))

d1openphotobtn = PhotoImage(file=relative_to_assets("d1open.png"))

msgopenphotobtn = PhotoImage(file=relative_to_assets("msgopen.png"))



def login():

    global objectdata,btnpressstatus,map_widget

    # Check the credentials (you'd replace this with your actual login logic)

    username = entry_1.get()

    password = entry_2.get()

    if username == "user" and password == "pass":

        # If login successful, hide the login screen elements



        canvas.place_forget()

        entry_1.place_forget()

        entry_2.place_forget()

        button_1.place_forget()

        

        maincanvas = Canvas(

            window,

            bg = "#8ABF93",

            height = screen_height,

            width = screen_width,

            bd = 0,

            highlightthickness = 0,

            relief = "ridge"

        )



        

        def update_text(id):

            global btnpressstatus

            btnpressstatus = id

        

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

                



        def get_timestamp():

            global timestamp

            while True:



                if(btnpressstatus == 100):

                    maincanvas.itemconfig(textarea, text="\n".join(objectdata["bsmessages"]))

                else:

                    if(btnpressstatus in objectdata["devices"]):

                        maincanvas.itemconfig(textarea, text="\n".join([f"{key}:{value}" for key, value in objectdata["devices"][btnpressstatus].items()]))

                    else:

                        maincanvas.itemconfig(textarea, text="D1 not yet found" if btnpressstatus==101 else "D2 not yet found")



                keys_to_modify = list(objectdata["devices"].keys())

                if(len(keys_to_modify)>0):

                    for key in keys_to_modify:

                        value = objectdata["devices"][key]

                        plane_image_main = Image.open(relative_to_assets("drone.png"))

                        plane_image_main.resize((80, 80))

                        try:

                            rotated_image = plane_image_main.rotate(-objectdata["devices"][key]["heading"])  # Negative heading to rotate clockwise

                            

                        except Exception as e:

                            print(f"Error rotating image: {e}")

                        # plane_image_main.rotate(int(objectdata["devices"][key]["heading"]))



                        plane_image = ImageTk.PhotoImage(rotated_image)



                    



                        if(key not in markershandle):

                            def contextmenu(marker):

                                # Create a context menu

                                context_menu = tk.Menu(window, tearoff=0)

                                context_menu.add_command(label="Set Relay", command=lambda attr=marker.data:setdronerelaying(0,attr))

                                context_menu.add_separator()

                                context_menu.add_command(label="Set Dest", command=lambda attr=marker.data:setdronerelaying(1,attr))

                                context_menu.add_separator()

                                context_menu.add_command(label="Takeoff", command=lambda attr=marker.data:setdronerelaying(2,attr))

                                context_menu.add_separator()

                                context_menu.add_command(label="Land", command=lambda attr=marker.data:setdronerelaying(3,attr))

                                context_menu.add_separator()

                                # context_menu.add_command(label="Menu Item 3", command=buttonclick,cnf={"id":marker.data})

                                canvasaxis = markershandle[key].get_canvas_pos(marker.position)

                                context_menu.tk_popup(int(canvasaxis[0])+300,int(canvasaxis[1])+70)

                                print(f"marker clicked - text: {marker.text}  position: {marker.position} : {marker.data} :{marker.data}")



                            markershandle[key] = map_widget.set_position(objectdata["devices"][key]["gps"][0],objectdata["devices"][key]["gps"][1], text="D1 drone",icon=plane_image,marker=True,command=contextmenu,data=key)

                            

                            

                        else:

                            # print(markershandle[key])

                            if(len(objectdata["devices"].keys())>0 and objectdata["devices"][key]):

                                markershandle[key].set_text("D1 drone" if key==101 else "D2 drone")

                                markershandle[key].change_icon(plane_image)

                                markershandle[key].set_position(objectdata["devices"][key]["gps"][0],objectdata["devices"][key]["gps"][1])



                    

                timestamp = int(time.time())

                maincanvas.itemconfig(current_timeview, text=F"Timestamp: {str(timestamp)}")

                maincanvas.itemconfig(connection_count, text=F"Connection count: {str(len(objectdata['devices'].keys()))}")

                time.sleep(0.1)





        maincanvas.place(x = 0, y = 0)

        maincanvas.create_rectangle(#for basestation info screen 

            0.0,

            35.0,

            225.0,

            200.0,

            fill="#6BE8E1",

            outline="white")



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



        maincanvas.create_rectangle( #Bs title

            0.0,

            0.0,

            1536.0,

            35.0,

            fill="#2E56E6",

            outline="white")

        

        

        maincanvas.create_text(

            17.9532470703125,

            5.0,

            anchor="nw",

            text="Base Station",

            fill="#FFFFFF",

            font=font.Font(family="Helvetica", size=20, weight="bold")

        )

        



        maincanvas.create_rectangle( #messages panel

            0.0,

            200.0,

            225.0,

            500.0,

            fill="#D9D6D6",

            outline="white")

        



        # drone behaviour handler

        

        maincanvas.create_rectangle(#devce running

            0.0,

            500.0,

            225.0,    

            870.0,

            fill="#F4F0F0",

            outline="white")

        

        behaviourhandledestview = maincanvas.create_text(

            30.0,

            530.0,

            anchor="nw",

            text="Destination : null",

            fill="#000000",

            font=font.Font(family="Helvetica", size=10, weight="bold")

        )



        behaviourhandlerelayview = maincanvas.create_text(

            30.0,

            550.0,

            anchor="nw",

            text="Relaying : null",

            fill="#000000",

            font=font.Font(family="Helvetica", size=10, weight="bold")

        )



        

      



        def armordisarmclick(state):

            global relayingarmingstate,relayingtargetid,destinationtargetid



            relaying_id = -1

            dest_id = -1



            for key, value in objectdata["devices"].items():

                if("relaying" in value):

                    if value["relaying"] == 1:

                        relaying_id = key

                if("dest" in value):

                    if value["dest"] == 1:

                        dest_id = key

            

            if(relaying_id == -1 or dest_id == -1):

                print("not set relaying and dest ids......")



                return

                       

            if(state == 0):

                relayingtargetid = relaying_id

                destinationtargetid = dest_id

                

                relayingarmingstate = 1

                print(F"arming relay id : {relaying_id} , dest id: {dest_id}, state : {relayingarmingstate}")





            else:

                relayingtargetid = relaying_id

                destinationtargetid = dest_id

                relayingarmingstate = 0

                print(F"arming relay id : {relaying_id} , dest id: {dest_id}, state : {relayingarmingstate}")





        

        armdrone = Button(

            # image=PhotoImage(file=relative_to_assets("d2open.png")),

            borderwidth=0,

            highlightthickness=0,

            command=lambda: armordisarmclick(0),

            relief="flat",

            text="Takeoff",

            background="green"

        )

        armdrone.place(

            x=30.0,

            y=570.0,

            width=52.6624984741211,

            height=37.38615036010742

        )



        disarmdrone = Button(

            # image=PhotoImage(file=relative_to_assets("d2open.png")),

            borderwidth=0,

            highlightthickness=0,

            command=lambda: armordisarmclick(1),

            relief="flat",

            text="Land",

            background="green"

        )

        disarmdrone.place(

            x=130.0,

            y=570.0,

            width=52.6624984741211,

            height=37.38615036010742

        )



        maincanvas.create_text(

            70.0,

            510.0,

            anchor="nw",

            text="Device status",

            fill="#000000",

            font=font.Font(family="Helvetica", size=12, weight="bold")

        )





       

        d2openbtn = Button(

            image=d2openphtotbtn,

            borderwidth=0,

            highlightthickness=0,

            command=lambda: update_text(102),

            relief="flat",

            # text="d2open"

        )

        d2openbtn.place(

            x=150.0,

            y=210.0,

            width=70.0,

            height=37.38615036010742

        )



        d1openbtn = Button(

            image=d1openphotobtn,

            borderwidth=0,

            highlightthickness=0,

            command=lambda: update_text(101),

            relief="flat",

            text="d1open"

        )

        d1openbtn.place(

            x=75,

            y=210,

            width=70.0,

            height=37.059417724609375

        )

      

        msgopenbtn = Button(

            image=msgopenphotobtn,

            borderwidth=0,

            highlightthickness=0,

            command=lambda: update_text(100),

            relief="flat",

            text="msg open"

        )

        msgopenbtn.place(

            x=0.0,

            y=210,

            width=70.0,

            height=37.059417724609375

        )



        # create map widget

        map_widget = tkintermapview.TkinterMapView(window, width=screen_width-105, height=screen_height-35,

                            corner_radius=0, use_database_only=False,

                            max_zoom=14, database_path=database_path)

        map_widget.place(

            relx=0.5, 

            rely=0.5, 

            anchor=tk.CENTER,

            x=175.0,

            y=20.0,

            )



        map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)  # google satellitemap_widget.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

        

        # map_widget.set_address("Cherlapalli,Secunderabad,Telangana", marker=False, image_zoom_visibility=(14, float("inf")))

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

        

        thread = threading.Thread(target=get_timestamp)

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

