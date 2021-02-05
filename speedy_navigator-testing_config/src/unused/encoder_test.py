import RPi.GPIO as GPIO
import time
import sys

count = 0

try:
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)	# for channel A
  GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)	# for channel B

  while not GPIO.input(27):
    pass
  tm_st = time.time()

  while True:
    if GPIO.input(27):
      #print "A tick"
      count += 1
      print count
      while GPIO.input(27):
        pass
    #if GPIO.input(22):
      ##print "B tick"
      #count += 1
      #while GPIO.input(22):
        #pass
    #tm = time.time() - tm_st
    #print count
    #if tm > 5:
      #print count
      #print tm
      #sys.exit()

finally:
  GPIO.cleanup()
