import asyncio
import logging
import copy

from construct import Array, Const, Int8ul, Int16sl, Int16ul, Int32ul, Struct, Float64l, Float32l
from construct.core import ConstError

from bleak import BleakScanner
from bleak import BleakClient

import pandas as pd
import matplotlib.pyplot as plt

devices_dict = {}
devices_list = []
sample_list = []
last_block = None

BLE_SAMPLES_PER_BLOCK = 8
BLE_BLOCKS_PER_PACKET = 2

packet_format = Struct(
    "version" / Const(b"\x01"),
    "blocks" / Int8ul,
    "temperature_index" / Int8ul,
    "reserved" / Int8ul,
    "temperature" / Float32l,
    "block" / Array(BLE_BLOCKS_PER_PACKET, 
                Struct(
                    "timestamp" / Float64l,
                    "counter" / Int32ul,
                    "G_fullScale" / Int16ul,
                    "XL_fullScale" / Int8ul,
                    "samples" / Int8ul,
                    "sample" / Array(BLE_SAMPLES_PER_BLOCK, 
                        Struct(
                            "gyr" / Struct(
                                "x" / Int16sl,
                                "y" / Int16sl,
                                "z" / Int16sl
                            ),
                            "acc" / Struct(
                                "x" / Int16sl,
                                "y" / Int16sl,
                                "z" / Int16sl
                            )
                        )
                    )
                )
    )
)

DEVICE_NAME = "Arduino IMU"
CHARACTERISTIC_UUID = "6f18bfca-8787-4516-89a7-cb272e36c653"

stop_event = asyncio.Event()

#To discover BLE devices nearby 
async def scan():
    dev = await BleakScanner.discover()
    print(dev)
    for i in range(0,len(dev)):
        #print("[" + str(i) + "]" + dev[i].address,dev[i].name,dev[i].metadata["uuids"])
        devices_dict[dev[i].address] = []
        devices_dict[dev[i].address].append(dev[i].name)
        devices_list.append(dev[i].address)

#An easy notify function, just print the recieve data
def notification_handler(sender, data):
    global last_block
    try:
        packet = packet_format.parse(data)
    except ConstError:
        # Wrong version number - leave that for now
        pass
    time_block_0 = packet['block'][0]['timestamp']
    time_block_1 = packet['block'][1]['timestamp']
    dt_0 = (time_block_1 - time_block_0) / packet['block'][0]['samples']
    if last_block is not None:
        time_lastblock = last_block['timestamp']
        dt_lastblock = (time_block_0 - time_lastblock) / last_block['samples']
        samples = extract_samples(last_block, dt_lastblock)
        sample_list.extend(samples)
    samples = extract_samples(packet['block'][0], dt_0)
    sample_list.extend(samples)
    last_block = copy.deepcopy(packet['block'][1])

async def wait_for_esc():
    running = True
    while running:
        await asyncio.sleep(0.1)

        import os
        ch = ''
        if os.name == 'nt': # how it works on windows
            import msvcrt
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in ['\000', '\xe0']:
                    ch = msvcrt.getch()
        else:
            import tty
            import sys
            import termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if len(ch) > 0 and ord(ch) == 27:
            print("ESC pressed")
            loop.call_soon_threadsafe(stop_event.set)
            running = False

async def run(address, debug=False):
    log = logging.getLogger(__name__)
    if debug:
        import sys

        log.setLevel(logging.DEBUG)
        h = logging.StreamHandler(sys.stdout)
        h.setLevel(logging.DEBUG)
        log.addHandler(h)

    def disconnect_callback(client):
        print("Disconnected from client {0}".format(client.address))
        loop.call_soon_threadsafe(stop_event.set)

    client = BleakClient(address, disconnected_callback=disconnect_callback)
    try:
        await client.connect()
        log.info("Connected: {0}".format(client.address))
        log.info("MTU size: {0}".format(client.mtu_size))

        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        print("Processing notifications until ESC pressed or device disconnects...")
        await stop_event.wait()
    except Exception as e:
        print(e)
    finally:
        if client.is_connected:
            print("Stopping notifications and disconnecting")
            await client.stop_notify(CHARACTERISTIC_UUID)
            await asyncio.sleep(0.5)  # Sleep a bit longer to allow _cleanup to remove all notifications nicely...
            await client.disconnect()
        print("Client disconnected")

def extract_samples(block, dt):
    samples = []
    t = block['timestamp']
    c = block['counter']
    acc_factor = block['XL_fullScale'] / 32768.0
    gyr_factor = block['G_fullScale'] / 32768.0
    for s_idx in range(block['samples']):
        s = block['sample'][s_idx]
        samples.append({
            'timestamp': t + s_idx*dt,
            'counter': c + s_idx,
            'acc_x': acc_factor * s['acc']['x'],
            'acc_y': acc_factor * s['acc']['y'],
            'acc_z': acc_factor * s['acc']['z'],
            'gyr_x': gyr_factor * s['gyr']['x'],
            'gyr_y': gyr_factor * s['gyr']['y'],
            'gyr_z': gyr_factor * s['gyr']['z']
        })
    return samples

if __name__ == "__main__":
    print("Scanning for peripherals...")

    #Build an event loop
    loop = asyncio.get_event_loop()
     #Run the discover event
    loop.run_until_complete(scan())

    #Find device with DEVICE_NAME
    imu_address = None
    for k,v in devices_dict.items():
        if DEVICE_NAME in v:
            imu_address = k

    if imu_address is None:
        print("No device found with name " + DEVICE_NAME)
    else:
        print("Address for device '{0}' is ".format(DEVICE_NAME) + imu_address)

        #Run notify event
        loop = asyncio.get_event_loop()
        loop.set_debug(True)
        try:
        #Run wait for ESC task
            loop.create_task(wait_for_esc())
            loop.run_until_complete(run(imu_address, True))
        except KeyboardInterrupt:
            print('\nReceived Keyboard Interrupt')
        finally:
            df = pd.DataFrame(sample_list)
            print (f"Sample rate {1.0/df['timestamp'].diff().median():.2f} Hz")
            df["acc_norm"] = df[["acc_x","acc_y","acc_z"]].pow(2).sum(axis=1).pow(0.5)
            fig, axes = plt.subplots(ncols=2)
            df.plot(ax=axes[0], x="timestamp", y=["acc_x","acc_y","acc_z","acc_norm"])
            df.plot(ax=axes[1], x="timestamp", y=["gyr_x","gyr_y","gyr_z"])
            plt.show()
    print('Program finished')
