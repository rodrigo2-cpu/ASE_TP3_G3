import time
import smbus
import random
import tweepy


# authentication credentials
API_KEY = 'asdsadsa'
API_KEY_SECRET = 'asdasdas'
ACCESS_KEY = '-'
ACCESS_SECRET = 'asdsadazazaa'
BEARER_TOKEN = 'xxxxxxxxxxxx'

client = tweepy.Client(consumer_key=API_KEY,
                        consumer_secret=API_KEY_SECRET,
                        access_token=ACCESS_KEY,
                        access_token_secret=ACCESS_SECRET)

# Select the channel
i2c_ch = 1

# Select the sensor address in the bus
i2c_addr = 0x4d

# Select a temperature register
reg_temp = 0x00

old_temp = 0

# write a tweet
def write_tweet(textT):
    response = client.create_tweet(text=textT)

    print(response)

# Calculate the 2's complement of a number
def twos_comp(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

# Read temperature registers and calculate Celsius
def read_temp():

    # Read temperature registers
    val = bus.read_i2c_block_data(i2c_addr, reg_temp, 2)
    # NOTE: val[0] = MSB byte 1, val [1] = LSB byte 2

    temp_c = (val[0] << 4) | (val[1] >> 4)

    # Convert to 2s complement (temperatures can be negative)
    temp_c = twos_comp(temp_c, 12)

    # Convert registers value to temperature (C)
    temp_c = temp_c * 0.0625

    return temp_c

# Initialize I2C (SMBus)
bus = smbus.SMBus(i2c_ch)

# Print out temperature every second
while True:
    temperature = read_temp()
    print(str(temperature)+"||||"+str(old_temp))
    print(abs(temperature - old_temp))
    if(abs(temperature - old_temp) >= 3):
        text = "Currently "+str(round(temperature,2))+"ºC in this room."
        old_temp = temperature
        
        # Publish a tweet
        write_tweet(text)
    print(round(temperature, 2), "ºC")
    time.sleep(1)

