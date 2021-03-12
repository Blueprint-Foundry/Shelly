from __future__ import print_function
import time
import roslibpy
import numpy

# ******************************
# set global variables 
# ******************************
send_time = 0
recieve_time = 0
payload_size = 1
arduino_connected = False

# ******************************
# Open text file
# ******************************
f = open("speed_test.csv", "a")
f.write("Size, Latency (ns),\n")


# ******************************
# Function for Confirming Arduino Connection
# ******************************
def respond_to_recieved_data(recieved_message):
  global arduino_connected
  arduino_connected = True
  print("recieved following message: \n") 
  print(recieved_message) 
  
# ******************************
# Function for Capturing Arduino Echo
# ******************************  
def respond_to_echo_data(recieved_message):
  global send_time
  global recieve_time
  global payload_size
  recieve_time = time.time_ns()
  latency_ns = recieve_time - send_time
  if(f.closed == False):
    f.write(f"{payload_size}, {latency_ns},\n")
  print("\nrecieved following echo: ") 
  print(f"{payload_size}, {latency_ns},\n") 
  print(recieved_message) 
  recieve_time = 0
  payload_size += 1
  time.sleep(1)
  send_time = 0
  
# ******************************
# Set up Rosbridge Nodes and topics
# ******************************  
# Create ROS Bridge Client
client = roslibpy.Ros(host='localhost', port=9090)

# start ROS bridge client
client.run()

# wait until connection is established
while (not client.is_connected):
  print('not connected')
  time.sleep(2)

# alert user connection is established 
if(client.is_connected):
  print('pynode connected to ROS bridge')


#create publishing topic 'pynode_shout' to push data
print("Creating 'pynode_shout' topic for pynode to send data to ESP")
pynodeshout_topic = roslibpy.Topic(client, '/pynode_shout', 'std_msgs/String')
pynodeshout_topic.advertise()

#create subscribing topic 'esp_echo to hear ESP echo
print("Creating 'esp_echo' topic for arduino to send back data pynode data")
especho_topic = roslibpy.Topic(client, '/esp_echo', 'std_msgs/String')
especho_topic.advertise()

#create transmit topic for pynode
print("Creating 'pynode_tx' topic to transmit pynode status")
pynodetx_topic = roslibpy.Topic(client, '/pynode_tx', 'std_msgs/String')
pynodetx_topic.advertise()

#create recieve topic for pynode
print("Creating 'pynode_rx' topic to recieve info to pynode")
pynoderx_topic = roslibpy.Topic(client, '/pynode_rx', 'std_msgs/String')
pynoderx_topic.advertise()

#send message on pynode_shout topic and listen for return every 2 seconds. 
#Once echo is heard, connection is established

#listen to pynode_rx topic
pynoderx_topic.subscribe(respond_to_recieved_data)


# ******************************
# While Loop to listen to first arduino connected response
# ******************************  
attempts = 0

while not arduino_connected:
  if(not arduino_connected):
    print(f'is arduiono connected: {arduino_connected}')
    print(f'attempts to connect: {attempts}')
    pynodetx_topic.publish(roslibpy.Message({'data': 'pynode alive'}))
    time.sleep(5)
    attempts += 1
  
  
# ******************************
# While Loop to perform latency test
# ******************************    
#listen for payload echo on esp_echo topic
especho_topic.subscribe(respond_to_echo_data)
  
attempts = 0
  

#pynode_payload = str(numpy.ones(payload_size, dtype = str))
while (payload_size < 135):
	if(send_time == 0):
	  time.sleep(1)
	  send_time = time.time_ns()
	  pynode_payload = "x"*payload_size 
	  pynodeshout_topic.publish(roslibpy.Message({'data': pynode_payload}))
	  attempts += 1
	  


print("ending test")
time.sleep(1)
	  
f.close()



# exit program
val = input("\n** press any key to exit **\n")

exit()


