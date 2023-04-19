import RPi.GPIO as GPIO
import sys
import argparse
import time
import struct
from RF24 import RF24, RF24_PA_MAX, RF24_2MBPS, RF24_1MBPS

Data_Size_From_TX = 12
Data_Size_Out = 12
Data_Received = bytearray(Data_Size_From_TX)
Data_To_Send = bytearray(Data_Size_Out)
New_Data_Received = False

Analog_Channels_Ammount = 6
Digital_Channels_Bytes_Count = 1
Digital_Channels_Ammount = 6
Digital_Channels_Inputs = [0,0,0,0,0,0,0,0]

Connection_Timeout = 5
Connection_Timeout_Timer = time.monotonic()
Connected_To_Transmitter = False

Total_Telemetry_Outputs = 4
Telemetry_Outputs = [[0,-1],[1,-1],[2,-1],[12,-1]]

Telemetry_Elements_Alarms = [False, False, False, False]

M1_LEFT = 4
M1_RIGHT = 17
M2_LEFT = 27
M2_RIGHT = 22
forward_buf = 23
backward_buf = 24

def setup(*ports):
  GPIO.setwarnings(False)
  GPIO.cleanup()
  GPIO.setmode(GPIO.BCM)
  for port in ports:
      GPIO.setup(port, GPIO.OUT)

def stop_all():
  F_M1.ChangeDutyCycle(0)
  F_M2.ChangeDutyCycle(0)
  B_M1.ChangeDutyCycle(0)
  B_M2.ChangeDutyCycle(0)

def forward():
  B_M1.ChangeDutyCycle(0)
  B_M2.ChangeDutyCycle(0)
  F_M1.ChangeDutyCycle(PWM_L*100/256)
  F_M2.ChangeDutyCycle(PWM_R*100/256)

def backward():
  F_M1.ChangeDutyCycle(0)
  F_M2.ChangeDutyCycle(0)
  B_M1.ChangeDutyCycle(PWM_L*100/256*-1)
  B_M2.ChangeDutyCycle(PWM_R*100/256*-1)

def left():
  F_M2.ChangeDutyCycle(0)
  B_M1.ChangeDutyCycle(0)
  B_M2.ChangeDutyCycle(PWM_R*100/256*-1)
  F_M1.ChangeDutyCycle(PWM_L*100/256)

def right():
  F_M1.ChangeDutyCycle(0)
  B_M2.ChangeDutyCycle(0)
  B_M1.ChangeDutyCycle(PWM_L*100/256*-1)
  F_M2.ChangeDutyCycle(PWM_R*100/256)
	
def movement(PWM_L, PWM_R):
	if PWM_L > 0 and PWM_R > 0:
		if GPIO.input(forward_buf) == True:
			stop_all()
		else:
			forward()
	if PWM_L < 0 and PWM_R < 0:
		if GPIO.input(backward_buf) == True:
			stop_all()
		else:
			backward()
	if PWM_L > 0 and PWM_R < 0:
		if GPIO.input(forward_buf) == True and abs(PWM_L) > abs(PWM_R):
			stop_all()
		elif GPIO.input(backward_buf) == True and abs(PWM_R) > abs(PWM_L):
			stop_all()
		else:
			left()
	if PWM_L < 0 and PWM_R > 0:
		if GPIO.input(backward_buf) == True and abs(PWM_L) > abs(PWM_R):
			stop_all()
		elif GPIO.input(forward_buf) == True and abs(PWM_R) > abs(PWM_L):
			stop_all()
		else:
			right()
	if PWM_L == 0 or PWM_R == 0:
		stop_all()

def bitRead(data, number):
  return list(str(bin(data))[2:].zfill(8))[7-number]

def GetChannelInputs():
  global Data_Received
  for j in range(Digital_Channels_Ammount):
    Digital_Channels_Inputs[j] = bitRead(Data_Received[Data_Size_From_TX-2], j)



radio = RF24(25, 0)


def getData():
  global New_Data_Received, Connection_Timeout_Timer, \
      Connected_To_Transmitter, Connection_Timeout, Data_Received
  if radio.available():
    New_Data_Received = True
    buf = radio.read(Data_Size_From_TX)
    Connected_To_Transmitter = True
    Data_Received = []
    for el in struct.unpack("12c", buf):
      Data_Received.append(int.from_bytes(el, "big"))
    print("Data_Received: {}".format(Data_Received))
    GetChannelInputs()
    Connection_Timeout_Timer = time.monotonic()

  if Connected_To_Transmitter == True:
    if (time.monotonic() - Connection_Timeout_Timer) > Connection_Timeout:
      Connected_To_Transmitter = False
      print("\nTimeoutReached: disconnect")
    

# def send() {

  # bool rslt;

  # if(Data_Received[(Data_Size_From_TX-1)] == 1){//Telemetry report has been requested
  
    # FormatTelemetryData();

    # if(Telemetry_Now_Sending_Alarms == true){
      # Data_To_Send[(Data_Size_Out-1)] = 2;
    # }else{
      # Data_To_Send[(Data_Size_Out-1)] = 1;
    # }  
    # rslt = radio.write(&Data_To_Send, sizeof(Data_To_Send));
 
  # }else if(Data_Received[(Data_Size_From_TX-1)] == 2){//Telemetry id list has been requested

    # FormatTelemetryList();
    # Data_To_Send[(Data_Size_Out-1)] = 3;
    # rslt = radio.write(&Data_To_Send, sizeof(Data_To_Send));
  # }
# }

def setBounds(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == "__main__":

  # initialize the nRF24L01 on the spi bus

  #####################################
  #####################################
  #####################################
  # Начало настройки nrf
  if not radio.begin():
      raise RuntimeError("radio hardware is not responding")


  slaveAddress = bytearray(b'ABCAAC')
  thisSlaveAddress = bytearray(b'OvbTX1')

  radio.setPALevel(RF24_PA_MAX)  # RF24_PA_MAX is default
  radio.setChannel(0x6f)
  radio.setRetries(2,15)         # Установите количество и задержку повторных попыток при неудачной отправке.
  radio.setDataRate(RF24_2MBPS)

  radio.openWritingPipe(thisSlaveAddress)  # always uses pipe 0
  radio.openReadingPipe(1, slaveAddress)  # using pipe 1

  radio.startListening()  # put radio in RX mode
  radio.printPrettyDetails()
  # Конец настройки nrf
  #####################################
  #####################################
  #####################################


  setup(M1_RIGHT, M1_LEFT, M2_RIGHT, M2_LEFT)
  GPIO.setup(forward_buf, GPIO.IN)
  GPIO.setup(backward_buf, GPIO.IN)

  F_M1 = GPIO.PWM(M1_LEFT, 100)
  B_M1 = GPIO.PWM(M1_RIGHT, 100)
  B_M2 = GPIO.PWM(M2_RIGHT, 100)
  F_M2 = GPIO.PWM(M2_LEFT, 100)

  F_M1.start(0)
  F_M2.start(0)
  B_M1.start(0)
  B_M2.start(0)

  while True:
    getData()
    if Digital_Channels_Inputs[2] == '1' and Connected_To_Transmitter == True:
        PWM_L = setBounds(Data_Received[1], 0, 255, -125, 125) + setBounds(Data_Received[0], 0, 255, -125, 125)
        PWM_R = setBounds(Data_Received[1], 0, 255, -125, 125) - setBounds(Data_Received[0], 0, 255, -125, 125)
        print("PWM_L", PWM_L)
        print("PWM_R", PWM_R)
        movement(PWM_L,PWM_R)
    else:
        stop_all()

