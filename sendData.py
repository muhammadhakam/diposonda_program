
from digi.xbee.devices import XBeeDevice
PORT = "/dev/ttyACM0"
BAUD_RATE = 9600





Command = ""

REMOTE_NODE_ID = "DIPO_A"



def main():
    print(" +----------------------------------------+")
    print(" ...|Send Data Command|....................")
    print(" +--------------------------------------+\n")

    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()

        # Obtain the remote XBee device from the XBee network.
        xbee_network = device.get_network()
        remote_device = xbee_network.discover_device(REMOTE_NODE_ID)
        if remote_device is None:
            print("Could not find the remote device")
            exit(1)

        print("Sending data to %s >> %s..." % (remote_device.get_64bit_addr(), Command))

        device.send_data(remote_device, Command)

        print("Success")

    finally:
        if device is not None and device.is_open():
            device.close()

def SetCommand():
        start = input("""Choose command for sendin to System :
                      1. SetTimeManual 
                      2. SetTimetoGPS
                      3. SimulationMode
                      4. FlightMode
                      5. PressureSet
                      6. TelemetryOn
                      7. TelemetryOff
                      Enter Command : """)
        inputCMD = int(start)
        global Command

        SetTimetoGPS = "<CMD,Diposonda,ST,GPS>"
        SimulationMode = "<CMD,Diposonda,SIM,ENA>"
        FlightMode = "<CMD,Diposonda,SIM,DIS>"
        TelemetryOn = "<CMD,Diposonda,CX,ON>"
        TelemetryOff = "<CMD,Diposonda,CX,OFF>"

        if (inputCMD == 1) :
            waktu= input(str(""" SetTimeManual format [1:2:30]:  """))
            setTime = (waktu)
            Command = "<CMD,Diposonda,ST,|{}|>".format(setTime)
            print("Set Command :", Command)

        elif (inputCMD == 2) :
            Command = SetTimetoGPS
            print("Set Command :", Command)

        elif (inputCMD == 3) :
            Command = SimulationMode
            print("Set Command :", Command)

        elif (inputCMD == 4) :
            Command = FlightMode
            print("Set Command :", Command)

        elif (inputCMD == 5) :
            pressure = input(str("""SetPressure: """, SetCommand()))
            setPressure = pressure
            Command = "<CMD,Diposonda,SIM,{}>".format(setPressure)
            print("Set Command :", Command)

        elif (inputCMD == 6) :
            Command = TelemetryOn
            print("Set Command :", Command)

        elif (inputCMD == 7) :
            Command =TelemetryOff
            print("Set Command :", Command)

        else :
            Command = "Asu Salah Cok"
            print (Command)

if __name__ == '__main__':
    while True :
        SetCommand()
        if (Command == "Asu Salah Cok") :
            print ("Baleni woii")
        else:
            main()
   