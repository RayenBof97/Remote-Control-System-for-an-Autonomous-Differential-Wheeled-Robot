import queue
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import threading

global Distance, Rotation
DATASIZE = 3
DATA_LABELS = [
    "Consigne",
    "Right speed",
    "Left speed",
]

fig, ax = plt.subplots()
data = []
lines = []
ReadingInput = queue.Queue()

def update_plot(new_value):
    Values = str(new_value).split(',')
    for j in range(0, DATASIZE):
        data[j].append(float(Values[j]))
        lines[j].set_data(range(len(data[j])), data[j])
    plt.pause(0.001)

def publish(topic, message):
    MQClient.publish(topic, message)

def on_connect(mqttc, obj, flags, reason_code, properties):
    print("reason_code: " + str(reason_code))

def on_message(mqttc, obj, msg):
    if msg.topic == "Readings":
        ReadingInput.put(bytes(msg.payload).decode())

def on_subscribe(mqttc, obj, mid, reason_code_list, properties):
    print("Subscribed: " + str(mid) + " " + str(reason_code_list))
for i in range(0, DATASIZE):
    data.append([])
    line, = ax.plot(data[i], label=DATA_LABELS[i])
    lines.append(line)
ax.legend()
MQClient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
MQClient.on_message = on_message
MQClient.on_connect = on_connect
MQClient.on_subscribe = on_subscribe
MQClient.connect("192.168.1.85", 1883, 60)
MQClient.subscribe("Readings")
MQClient.loop_start()

while True:
    if ReadingInput.qsize() > 0:
        firstIndata = ReadingInput.get()
        if firstIndata is not None:
            update_plot(firstIndata)
    fig.canvas.draw()
    fig.canvas.flush_events()

MQClient.loop_stop()

