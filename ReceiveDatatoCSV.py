
import csv
from digi.xbee.exception import XBeeDeviceException
from digi.xbee.devices import XBeeDevice
from digi.xbee.models.status import NetworkDiscoveryStatus
import time
import pandas as pd
PORT = "/dev/ttyACM0"
BAUD_RATE = 9600


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

    field = '<Diposonda,TeamCode,Time,csc,data_count,flt_md,alt_rel,probe_stat,selfStand_stat,temp,alt_gps,lat_val,lng_val,sat_val,y_ori,z_ori,gasCO,gasCH4,gasEthanol,gasH2,gasNH3,gasNO2>'
    rows = []
    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()
        device.flush_queues()

        print("Waiting for data...\n")

        while True:
            xbee_message = device.read_data()
            if xbee_message is not None:


                dataStream = (xbee_message.data.decode())
                print(dataStream)
                rows.append(dataStream)
                data = {field : [dataStream]}

                df = pd.DataFrame(data)

                with open ('test.csv', 'a') as f:
                    variable1 = 0
                    variable2 = 1
                    df.to_csv(f, header=f.tell()==0, index=False)
                    reader = csv.reader(f)
                    rows = list(reader)
                    
                    for i in range(rows):
                        variable1 += 2
                        variable2 += 2
                        merged_row = rows[variable1] + rows[variable2]
                        with open('output.csv', 'w', newline='') as output_file:
                            writer = csv.writer(output_file)
                            writer.writerow(merged_row)
                time.sleep(1)
    finally:
        if device is not None and device.is_open():
            device.close()


if __name__ == '__main__':
    
    Recovery()
    Receive()
    

