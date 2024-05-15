
import subprocess

# Replace '/path/to/venv' with the actual path to your virtual environment
venv_path = './venv'

# Activate the virtual environment
activate_script = f"{venv_path}/bin/activate"
activate_cmd = f"source {activate_script}"
subprocess.run(activate_cmd, shell=True)


import requests
from getmac import get_mac_address
import socket



macaddress = get_mac_address().upper()
print(macaddress)

curretipaddress = ""



def send_telegram_message(token, chat_id, message):
    base_url = f"https://api.telegram.org/bot{token}/sendMessage?chat_id={chat_id}&text={message}"
    response = requests.get(base_url)
    if response.status_code == 200:
        print("Message sent successfully!")
    else:
        print("Failed to send message.")

# Replace these values with your bot token, chat ID, and message content
bot_token = "6811299138:AAEEmSK8ZxkOBMipd384sRgCrVRppdNRvPk"
chat_id = "-4076716877"
# https://api.telegram.org/bot6811299138:AAEEmSK8ZxkOBMipd384sRgCrVRppdNRvPk/sendMessage?chat_id=-4076716877&text=hi

def get_ip_address():
    try:
        # Create a socket object
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Connect to a dummy server
        s.connect(("8.8.8.8", 80))
        # Get the local IP address
        ip_address = s.getsockname()[0]
        s.close()
        return ip_address
    except OSError as e:
        print(f"Error: {e}")
        return None

# Call the function to get the IP address
while True:
    pi_ip = get_ip_address()
    if pi_ip:
        if(curretipaddress != pi_ip):
            curretipaddress = pi_ip
            # Call the function with your token, chat ID, and message
            send_telegram_message(bot_token, chat_id, "Drone Realy project Base station ip : " +curretipaddress+",\n macAddress : "+macaddress)

            print(f"Raspberry Pi IP address: {pi_ip}")
    else:
        print("Failed to retrieve IP address.")
