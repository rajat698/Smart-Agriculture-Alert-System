#!/usr/bin/env python3

import requests
import sys
import getopt
import serial


ser = serial.Serial('/dev/cu.usbserial-AR0JVW4V', baudrate=9600, timeout=1)         #Set serial input

#Function to send messages

def send_slack_message(message):
    payload = '{"text":"%s"}' % message
    response = requests.post('https://hooks.slack.com/services/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',
                             data=payload)                      #Defining the address of our target slack channel

    print(response.text)                                        #Print message

#Main function

def main(argv):
    message = ' '

    try:
        opts, args = getopt.getopt(argv, "hm:", ["message="])

    except getopt.GetoptError:
        print('SlackMessage.py -m <message>')
        sys.exit(2)

    if len(opts) == 0:
        message = "Hello, World!"

    for opt, arg in opts:
        if opt == '-h':
            print('SlackMessage.py -m <message>')
            sys.exit()
        elif opt in ("-m", "--message"):
            message = arg

#Infinite Loop to store values from our serial device and sending alerts accordingly
while 1:

    data = ser.readline(5).decode('ascii')              #Read serial values

    if int(data) < 1000:
        send_slack_message("ALERT: TOO LOW BRIGHTNESS") #Send alerts in case of too low brightness

#Main function calling

if __name__ == "__main__":
        main(sys.argv[1:])
