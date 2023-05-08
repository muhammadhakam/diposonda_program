import csv
from digi.xbee.exception import XBeeDeviceException
from digi.xbee.devices import XBeeDevice
from digi.xbee.models.status import NetworkDiscoveryStatus
import time

PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600


class backend:
    def __init__(self) -> None:
        pass
    


    def Recovery():
        print (""" 
        +---------------------------------------------------+
        |||||||||||||||| Recover Devices ||||||||||||||||||||
        +---------------------------------------------------+
        """)

        print("Golek device sik yo sabar!!!")
        device = XBeeDevice(PORT, BAUD_RATE)
        try:
            device.open(force_settings=True)
            print("Device opened and set to operate at %d bauds" %BAUD_RATE)

        except XBeeDeviceException as e:
            print("ERROR: %" % str(e))
            return
        
        finally:
            if device is not None and device.is_open():
                device.close()

    def DiscoverRemote():
        print("+----------------------------------+")
        print("+---------DISCOVER NETWORK+---------")
        print("+----------------------------------+\n")

        device = XBeeDevice(PORT, BAUD_RATE)
        try :
            device.open()
            xbee_network = device.get_network()
            xbee_network.set_discovery_timeout(15)
            xbee_network.clear()
            
            def callback_device_discovered(remote):
                print("Device discovered: %s" % remote)
            
            def callback_discovery_finished(status):
                if status == NetworkDiscoveryStatus.SUCCESS:
                    print("Discovery process finished successfully.")
                else:
                    print("There was an error discovering devices: %s" % status.description)
            xbee_network.add_device_discovered_callback(callback_device_discovered)
            xbee_network.add_discovery_process_finished_callback(callback_discovery_finished)
            xbee_network.start_discovery_process()
            print("Discovering remote xbee devices...")
            while xbee_network.is_discovery_running():
                time.sleep(0.1)

        finally:
            if device is not None and device.is_open():
                device.close()
            

    def Receive():
        print(" +-------------------------------------------------+")
        print("............ | DIPOSONDA RECEIVE DATA |.............")
        print(" +-----------------------------------------------+\n")

        field = ['test']
        rows = []
        filename = "test.csv"
        device = XBeeDevice(PORT, BAUD_RATE)

        try:
            device.open()
            device.flush_queues()

            print("Waiting for data...\n")

            while True:
                xbee_message = device.read_data()
                if xbee_message is not None:
                    print("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                            xbee_message.data.decode()))
                    dataStream = ["From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                            xbee_message.data.decode())]
                    rows.append(dataStream)
                    
                    with open(filename) as csvfile:
                        csvwriter = csv.writer(csvfile)
                        csvwriter.writerow(field)
                        csvwriter.writerows(rows)            

        finally:
            if device is not None and device.is_open():
                device.close()


