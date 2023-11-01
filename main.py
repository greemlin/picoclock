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
ssid_ = 'KanariNET1'
ssid = 'KorrLAN'
psw_ = 'Ka11p**!bio210373abc'
psw = "K0rrL@nW1f1"
UTC_OFFSET = 2 * 3600  # UTC+2 hours for Greek Time
DST_OFFSET = 1 * 3600  # Additional 1 hour for DST
NTP_DELTA = 2208988800  # NTP time delta
host = "gr.pool.ntp.org"  # NTP server
displaying_temp = False
mqtt_broker = "YourMQTTBrokerIP"
mqtt_username = "YourMQTTUsername"
mqtt_password = "YourMQTTPassword"
mqtt_topic = "YourMQTTTopic"
sync_successful = False
sync_in_progress = False
displaying_temp = False

# Setup
timer = Timer()
rtc = machine.RTC()
adc = machine.ADC(4)
led = machine.Pin("LED", machine.Pin.OUT)
spi = SPI(0, baudrate=10000000, polarity=1, phase=0, sck=Pin(2), mosi=Pin(3))
ss = Pin(5, Pin.OUT)
wlan = network.WLAN(network.STA_IF)
display = max7219.Matrix8x8(spi, ss, 4)
display.brightness(0)
lock = _thread.allocate_lock()

def adjust_display_brightness_by_time():
    current_time = rtc.datetime()  # Get the current time
    hour = current_time[4]  # Hour is at index 4

    # Set brightness based on the hour
    if 6 <= hour <= 18:  # Day time
        display.brightness(1)  # Brighter for daytime
    else:  # Night time
        display.brightness(0)  # Dimmer for nighttime

def is_dst(now=None):
    if now is None:
        now = utime.localtime()
    year, month, day, hour, _, _, wday, yday = now
    # Start and end dates of DST
    dst_start = utime.mktime((year, 3, (31 - (int(5 * year / 4 + 4) % 7)), 2, 0, 0, 0, 0, 0))
    dst_end = utime.mktime((year, 10, (31 - (int(5 * year / 4 + 1) % 7)), 3, 0, 0, 0, 0, 0))
    # Check if current date is within the DST period
    if utime.mktime(now) >= dst_start and utime.mktime(now) < dst_end:
        return True
    return False


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
                adjust_display_brightness_by_time()  # Adjust brightness based on time
                sync_successful = True
                break
            except Exception as e:
                sync_successful = False
                if i < retries - 1:
                    time.sleep(5)
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
    global sync_successful
    try:
        ntptime.host = host
        ntptime.settime()
        t = utime.time()
        # Adjust time for UTC offset for Greek time
        t += UTC_OFFSET
        # Check if DST is in effect and adjust time if needed
        if is_dst(utime.localtime(t)):
            t += DST_OFFSET
        tm = utime.localtime(t)
        rtc.datetime((tm[0], tm[1], tm[2], 0, tm[3], tm[4], tm[5], 0))
        sync_successful = True
    except:
        sync_successful = False
        
def init():
    reset_display()
    rp2.country('GR')
    connect()
    time.sleep(2)
    sync_time()

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

# Start the asyncio loop
loop = asyncio.get_event_loop()
loop.create_task(background_task())
timer.init(period=24*60*60*1000, mode=Timer.PERIODIC, callback=timer_callback)

x=0

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
    if x == 30:
        _thread.start_new_thread(get_temperature_ext, ())
        x = 0

