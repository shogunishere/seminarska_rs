# Python script for receiving data over MQTT protocol
# In code for ESP8266, we publish data to the channel, while here, we subscribe to the channel to receive data. 
# Prerequisites: paho.mqtt library
import paho.mqtt.client as mqtt
import json
import pandas as pd

topic = "acceleration"
df = pd.DataFrame(columns=['seconds','accX', 'accY', 'accZ'])

def on_connect(client, userdata, flags, rc):  # The callback for when the client connects to the broker
    print("Connected with result code {0}".format(str(rc)))  # Print result of connection attempt
    client.subscribe(topic)  # Subscribe to the topic “digitest/test1”, receive any messages published on it


def on_message(client, userdata, msg):  # The callback for when a PUBLISH message is received from the server.
    print("Message received-> " + msg.topic + " " + str(msg.payload))  # Print a received msg
    data = [float(x) for x in msg.payload.decode().split(',')]  # Parse CSV string into list of floats
    df.loc[len(df)] = data  # Append data to the DataFrame

client = mqtt.Client("digi_mqtt_test")  # Create instance of client with client ID “digi_mqtt_test”
client.on_connect = on_connect  # Define callback function for successful connection
client.on_message = on_message  # Define callback function for receipt of a message
client.connect("broker.mqtt-dashboard.com", 1883, 60)  # Connect to (broker, port, keepalive-time)
#client.connect('127.0.0.1', 17300)

try:
    client.loop_forever()  # Start networking daemon
except KeyboardInterrupt:
    pass
finally:
    df.to_csv('acceleration_data_walking.csv', index=False)  # Save DataFrame to a CSV file when the script is interrupted