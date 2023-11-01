import machine
import network
import socket
import time
import _thread
import max7219
import ntptime
import rp2
import struct
import utime
import uasyncio as asyncio
from PicoDHT22 import PicoDHT22
from machine import Pin, SPI
from machine import ADC, RTC
from machine import Timer
from umqtt.simple import MQTTClient

# variables
sensor_name = 'NK Clock'
ssid = 'KanariNET1'
ssid2 = 'KorrLAN'
psw = 'Ka11p**!bio210373abc'
psw2 = "K0rrL@nW1f1"
UTC_OFFSET = 3 * 60 * 60
NTP_DELTA = 2208988800
host = "pool.ntp.org"
displaying_temp = False
mqtt_broker = "192.168.1.55"
mqtt_username = "<mqtt_username>"
mqtt_password = "<mqtt_password>"
mqtt_topic = "<mqtt_topic>"
x = 0
sync_successful = False
sync_in_progress = False

# Setup
timer = Timer()
rtc = machine.RTC()
adc = machine.ADC(4)
led = machine.Pin("LED", machine.Pin.OUT)
spi = SPI(0, baudrate=10000000, polarity=1, phase=0, sck=Pin(2), mosi=Pin(3))
ss = Pin(5, Pin.OUT)
wlan = network.WLAN(network.STA_IF)
display = max7219.Matrix8x8(spi, ss, 4)
display.brightness(1)
lock = _thread.allocate_lock()

def connect():
    display_corner()
    for _ in range(10):
        if not wlan.isconnected():
            wlan.active(True)
            wlan.connect(ssid, psw)
            time.sleep(1)
        else:
            break
    reset_display()

def reset_display():
    display.fill(0)
    display.show()
    time.sleep(0.1)

def get_temperature_ext():
    global lock
    global displaying_temp
    with lock:
        displaying_temp = True
        d = PicoDHT22(Pin(16, Pin.IN, Pin.PULL_UP))
        T, H = d.read()
        display_text(f'{T}C  {H}%')
        reset_display()
        displaying_temp = False

def display_corner():
    display.pixel(0,0,1)
    display.pixel(0,1,1)
    display.pixel(1,0,1)
    display.show()

def display_value(a_msg, pos = 0):
    display.fill(0)
    display.text(a_msg ,pos,0,1)
    display.show()

def display_text(text):
    length = len(text)
    column = (length * 8)
    for x in range(32, -column, -1):     
       display.fill(0)
       display.text(text ,x,0,1)
       display.show()
       time.sleep(0.05)

def get_temperature_int():
    ADC_voltage = adc.read_u16() * (3.3 / (65535))
    temperature_celcius = round(27 - (ADC_voltage - 0.706)/0.001721,1)
    display_value(f'{temperature_celcius}')

def sync_time():
    global lock
    global sync_successful
    retries = 5
    for i in range(retries):
        got_lock = lock.acquire(False)
        if got_lock:
            try:
                display_synch()
                ntptime.settime()
                set_time()
                sync_successful = True
                break  # time synchronization successful, break the loop
            except Exception as e:
                sync_successful = False
                if i < retries - 1:  # No delay on last attempt
                    time.sleep(5)  # delay between retries
            finally:
                lock.release()
                reset_display()

def display_synch():
    display.pixel(31,7,1)
    display.show()
    
def display_mqtt():
    display.pixel(0,7,1)
    display.show()

def set_time():
    NTP_QUERY = bytearray(48)
    NTP_QUERY[0] = 0x1B
    try:
        addr = socket.getaddrinfo(host, 123)[0][-1]
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1)
        res = s.sendto(NTP_QUERY, addr)
        msg = s.recv(48)
    finally:
        s.close()
    val = struct.unpack("!I", msg[40:44])[0]
    t = val - NTP_DELTA + 3 * 60 * 60  
    tm = time.gmtime(t)
    machine.RTC().datetime((tm[0], tm[1], tm[2], tm[6] + 1, tm[3], tm[4], tm[5], 0))

def init():
    reset_display()
    rp2.country('GR')
    connect()
    time.sleep(2)
    sync_time()  # Perform the initial time synchronization

def timer_callback(timer):
    asyncio.create_task(sync_time_async())

async def sync_time_async():
    global lock
    global sync_successful
    global sync_in_progress

    if sync_in_progress:
        return  # Synchronization is already in progress, skip this attempt

    sync_in_progress = True
    retries = 5

    try:
        for i in range(retries):
            got_lock = lock.acquire(False)
            if got_lock:
                try:
                    display_synch()
                    ntptime.settime()
                    set_time()
                    sync_successful = True
                    break  # time synchronization successful, break the loop
                except Exception as e:
                    sync_successful = False
                    if i < retries - 1:  # No delay on last attempt
                        await asyncio.sleep(5)  # delay between retries
                finally:
                    lock.release()
                    reset_display()

            await asyncio.sleep(0)  # Allow other tasks to run

    finally:
        sync_in_progress = False

async def background_task():
    while True:
        await asyncio.sleep(60)  # Wait for 60 seconds
        asyncio.create_task(sync_time_async())  # Start the time synchronization task

init()

loop = asyncio.get_event_loop()
loop.create_task(background_task())

timer.init(period=24*60*60*1000, mode=Timer.PERIODIC, callback=timer_callback)

while True:
    if not displaying_temp:
        t = machine.RTC().datetime()
        x += 1
        HH = "{:02}".format(t[4])
        mm = "{:02}".format(t[5])
        if t[0] >= 2022 or sync_successful:
            display_value(f'{HH}{mm}')
        else:
            display_value(f'----')
        if x % 2 == 0:
            display.pixel(15, 2, 0)
            display.pixel(15, 5, 0)
        else:
            display.pixel(15, 2, 1)
            display.pixel(15, 5, 1)
        display.show()
    time.sleep(1)
    if x == 20:
        _thread.start_new_thread(get_temperature_ext, ())
        x = 0
