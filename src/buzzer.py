import machine
import time

class buzzer:
    def __init__(self, buzzer_pin=14, buzzer_freq=3000):
        self.buzzer = machine.PWM(machine.Pin(buzzer_pin), freq=buzzer_freq, duty=0)
        self.duty = 32000
        self.beep_dur = 300 # milliseconds
        self.alarm_interval = 500 # milliseconds
        self.starttime = 0
        self.alarm = False
        self.firstflag = True
        self.highflag = False
        self.duties = [0, self.duty]
        
    def beeper(self):
        self.buzzer.duty(32000)
        time.sleep(1)
        self.buzzer.duty(0)
        
    def alarm_check(self):
        '''
        By running this function on the main loop of the program, an alarm will sound (ON/OFF 500ms)
        '''
        
        if self.alarm:
            self.buzzer.duty(self.duty)
        else:
            self.buzzer.duty(0)
            
            
            '''
            if self.firstflag:
                self.buzzer.duty(self.duty) # ON
                self.highflag = True
                self.starttime = time.time()
                self.firstflag = False
        
            if time.time() - self.starttime >= 1:
                if self.highflag:
                    self.buzzer.duty(0)
                    self.highflag = False
                else:
                    self.buzzer.duty(self.duty)
                    self.highflag = True        
                self.starttime = time.time()         
            '''