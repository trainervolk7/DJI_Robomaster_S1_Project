import time
import sys
sys.path.append('/home/volk/Desktop/DJI_Robomaster/env/lib/python3.7/site-packages')
from evdev import InputDevice, categorize, ecodes

class ps5controller:

    def __init__(self, event_path,blind = 15):
        '''Initializes the PS5 controller object
        @param event_path: The path to the controller event file (e.g. /dev/input/event2)
        @param blind: The range of values around the center that are considered as the center (default is 15)
        '''
        self.controller = InputDevice(event_path)

        self.L2_value = 0
        self.R2_value = 0

        self.button_presses = {                          # ecodes.EV_KEY
            304: 'x',
            305: 'circle',
            307: 'triangle',
            308: 'square',
            310: 'L1',                           
            311: 'R1',
            312: 'L2',                           
            313: 'R2',                           
            314: 'share',                        # 3 vertical lines, top left side of touchpad      
            315: 'menu',                         # 3 horizontal lines, top right of touchpad
            316: 'playstation',
            317: 'left joystick pressed',        # left joystick press down vertically
            318: 'right joystick pressed',       # right joystick press down vertically
            272: 'touchpad'
        }
        self.reverse_button_presses = {v: k for k, v in self.button_presses.items()}
        
        self.button_values = {                           # ecodes.EV_KEY button press values
            0: 'up',
            1: 'down'
        }
        self.reverse_button_values = {v: k for k, v in self.button_values.items()}
        
        self.absolutes = {                               # ecodes.EV_ABS
            0: 'left joystick left/right',          # 0 = left, 255 = right
            1: 'left joystick up/down',             # 0 = up, 255 = down
            2: 'L2 analog',                         # 0 = no press, 255 = full press
            5: 'R2 analog',                         # 0 = no press, 255 = full press
            3: 'right joystick left/right',         # 0 = left, 255 = right
            4: 'right joystick up/down',            # 0 = up, 255 = down
            16: 'leftpad left/right',               # -1 = left, 0 = stop pressing, 1 = right
            17: 'leftpad up/down',                  # -1 = up, 0 = stop pressing, 1 = down
        }
        self.reverse_absolutes = {v: k for k, v in self.absolutes.items()}
        
        self.leftpad_left_right_values = {
            -1: 'left',
            0: 'left-right stop',                   # stop means that the button was no longer pressed
            1: 'right'
        }
        self.reverse_leftpad_left_right_values = {v: k for k, v in self.leftpad_left_right_values.items()}
        
        self.leftpad_up_down_values = {
            -1: 'up',
            0: 'up-down stop',
            1: 'down'
        }
        self.reverse_leftpad_up_down_values = {v: k for k, v in self.leftpad_up_down_values.items()}
        
        self.CENTER = 128
        self.BLIND = blind                                  # there's a lot of drift at 128, so don't report changes within (128 - this value)
        self.MAX_EMERGENCY_DELAY = 1000                  # max number of milliseconds between taps to qualify as an emergency double-tap

        self.emergency_tap_time = 0                      # track when the last time the emergency button (triangle) was pressed
        self.left_joystick, self.right_joystick = [self.CENTER, self.CENTER], [self.CENTER, self.CENTER]

    def __str__(self):
        '''Returns the string representation of the PS5 controller object'''
        return f"PS5 Controller: {self.controller}"
    
    def read_loop(self):
        '''Reads the controller input and yields the events
        Use: In a for loop to read the controller input
        @return: the controller event
        '''
        while True:
            try:
                events = self.controller.read()
                for event in events:
                    yield event
            except BlockingIOError:
                time.sleep(0.01)

    def __update_left_joystick_position(self,event):
        '''Updates the left joystick position
        Use: Class internal function
        See: Update_joystick_position
        @param event: The event from the controller
        '''
        global left_joystick
        if event.code == 0:                     # left joystick, x-axis (left/right)
            self.left_joystick[0] = event.value
        elif event.code == 1:                   # left joystick, y-axis (up/down)
            self.left_joystick[1] = event.value

    def __update_right_joystick_position(self,event):
        '''Updates the right joystick position
        Use: Class internal function
        See: Update_joystick_position
        @param event: the event from the controller
        '''
        global right_joystick
        if event.code == 3:                     # right joystick, x-axis (left/right)
            self.right_joystick[0] = event.value
        elif event.code == 4:                   # right joystick, y-axis (up/down)
            self.right_joystick[1] = event.value
        
    def __decode_leftpad(self,event):
        '''Decodes the leftpad event 
        Use: Class internal function
        See: test_button_press
        @param event: The event from the controller
        @return: The action from the leftpad event
        '''
        action  = ''
        if event.code == 16:                    # leftpad, either a left or right action
            action = self.leftpad_left_right_values[event.value]
        elif event.code == 17:                  # leftpad, either an up or down action
            action = self.leftpad_up_down_values[event.value]
        else:                                   # unhandled event
            return ''
        return f'leftpad: {action}'
    
    def test_input(self,event):
        '''For testing: Prints the output of the butons pressed
        Use: Within the read_loop for the controller
        @param event: the event from the controller
        '''
        if event.type == ecodes.EV_KEY and event.code in self.button_presses:       # any button press other than leftpad
            button, direction = self.button_presses[event.code], self.button_values[event.value]
            print(f'{button} {direction}')

        if event.type == ecodes.EV_ABS and event.code in self.absolutes:                     # leftpad, joystick motion, or L2/R2 triggers
            action, value = self.absolutes[event.code], event.value
            
            if event.code in [0, 1, 3, 4]:                                              # joystick motion
                if event.code in [0, 1]:                                                # left joystick moving
                    self.__update_left_joystick_position(event)
                elif event.code in [3, 4]:                                              # right joystick moving
                    self.__update_right_joystick_position(event)

                if event.value < (self.CENTER - self.BLIND) or event.value > (self.CENTER + self.BLIND):   # skip printing the jittery center for the joysticks     
                    print(f'{self.left_joystick}, {self.right_joystick}')

            if event.code in [2, 5]:                                                    # L2/R2 triggers
                print(f'{action} {value}')
            elif event.code in [16, 17]:                                                # leftpad (d-pad) action
                action = self.__decode_leftpad(event)
                print(action)


    def button_press(self, event, button, direction = 'down'):
        '''Checks if a button was pressed
        Use: Within the read_loop for the controller
        @param event: The event from the controller
        @param button: The desired button to check (e.g. 'triangle')
        @param direction: The direction of the button press (default is down)
        @return: True if the button was pressed, False otherwise
        '''
        if button in self.reverse_absolutes:
            if direction in self.reverse_leftpad_left_right_values: # leftpad left/right
                button_value = self.reverse_leftpad_left_right_values[direction]
                if event.value == button_value:
                    return True
            if direction in self.reverse_leftpad_up_down_values: # leftpad up/down
                button_value = self.reverse_leftpad_up_down_values[direction]
                if event.value == button_value:
                    return True
        elif button in self.reverse_button_presses:
                button_code = self.reverse_button_presses[button]
                button_value = self.reverse_button_values[direction]
                if event.code == button_code and event.value == button_value:
                    return True
        return False
    
    def get_any_button_press(self,event):
        '''Returns the button pressed
        Use: Within the read_loop for the controller
        @param event: The event from the controller
        @return: The button pressed
        '''
        if event.type == ecodes.EV_KEY and event.code in self.button_presses:       # any button press other than leftpad
            button, direction = self.button_presses[event.code], self.button_values[event.value]
            return f'{button}', f'{direction}'
        elif event.code == 16:                    # leftpad, either a left or right action
            direction = self.leftpad_left_right_values[event.value]
            return 'leftpad', direction
        elif event.code == 17:                  # leftpad, either an up or down action
            direction = self.leftpad_up_down_values[event.value]
            return 'leftpad', direction
        else:                                   # unhandled event
            return '',''
        
    def update_joystick_position(self,event):
        '''Updates left and right joystick position
        Use: Within the read_loop for the controller before retrieving the joystick position
        @param event: The event from the controller
        '''
        if event.type == ecodes.EV_ABS and event.code in self.absolutes:
            if event.code in [0, 1, 3, 4]:                                              # joystick motion
                if event.code in [0, 1]:                                                # left joystick moving
                    self.__update_left_joystick_position(event)
                elif event.code in [3, 4]:                                              # right joystick moving
                    self.__update_right_joystick_position(event)
    
    def get_joystick_position(self):
        '''Returns the raw joystick position based on the blind value
        Use: After updating the joystick position
        @return: The raw left and right joystick positions
        '''
        corrected_left_joystick, corrected_right_joystick = self.left_joystick, self.right_joystick
        if self.left_joystick[1] > (self.CENTER - self.BLIND) and self.left_joystick[1] < (self.CENTER + self.BLIND):
            corrected_left_joystick[1] = self.CENTER
        if self.right_joystick[1] > (self.CENTER - self.BLIND) and self.right_joystick[1] < (self.CENTER + self.BLIND):
            corrected_right_joystick[1] = self.CENTER

        if self.left_joystick[0] > (self.CENTER - self.BLIND) and self.left_joystick[0] < (self.CENTER + self.BLIND):
            corrected_left_joystick[0] = self.CENTER
        if self.right_joystick[0] > (self.CENTER - self.BLIND) and self.right_joystick[0] < (self.CENTER + self.BLIND):
            corrected_right_joystick[0] = self.CENTER

        return corrected_left_joystick, corrected_right_joystick

    def linear_scaled_joystick(self,joystick,min,max):
        '''Returns the joystick value based on current position within desired adjusted range
        Use: After updating the joystick position
        @param joystick: The desired joystick to scale (e.g. 'left' or 'right')
        @param min: The minimum value of the new desired output range
        @param max: The maximum value of the new desired output range
        @return: The scaled joystick position
        '''
        corrected_left_joystick, corrected_right_joystick = self.get_joystick_position()
        scaled_left_joystick, scaled_right_joystick = [0,0],[0,0]
        if joystick == 'left' and corrected_left_joystick != [self.CENTER, self.CENTER]:
            if corrected_left_joystick[0] < self.CENTER: #x-axis
                old_min = 0
                old_max = self.CENTER -self.BLIND
                new_min = -min
                new_max = -max
                scaled_left_joystick[0] = new_max - ((corrected_left_joystick[0] - old_min) * (new_max - new_min)) / (old_max - old_min)
            else:
                old_min = self.CENTER + self.BLIND
                old_max = 255
                new_min = min
                new_max = max

                scaled_left_joystick[0] = new_min + ((corrected_left_joystick[0] - old_min) * (new_max - new_min)) / (old_max - old_min)

            if corrected_left_joystick[1] < self.CENTER:  # y-axis, below center
                old_min = 0
                old_max = self.CENTER - self.BLIND
                new_min = max  # Adjusted to start from max (negative direction)
                new_max = min  # Adjusted to end at min (negative direction)
                scaled_left_joystick[1] = new_min - ((corrected_left_joystick[1] - old_min) * (new_min - new_max)) / (old_max - old_min)
            else:  # y-axis, above center
                old_min = self.CENTER + self.BLIND
                old_max = 255
                new_min = -min
                new_max = -max
                scaled_left_joystick[1] = new_min + ((corrected_left_joystick[1] - old_min) * (new_max - new_min)) / (old_max - old_min)

            if corrected_left_joystick[0] == 128:
                scaled_left_joystick[0] = 0
            if corrected_left_joystick[1] == 128:
                scaled_left_joystick[1] = 0

            return scaled_left_joystick
        elif joystick == 'left' and corrected_left_joystick == [self.CENTER, self.CENTER]:
            return [0,0]

        elif joystick == 'right' and corrected_right_joystick != [self.CENTER, self.CENTER]:
            if corrected_right_joystick[0] < self.CENTER: #x-axis
                old_min = 0
                old_max = self.CENTER -self.BLIND
                new_min = -min
                new_max = -max
                scaled_right_joystick[0] = new_max - ((corrected_right_joystick[0] - old_min) * (new_max - new_min)) / (old_max - old_min)
            else:
                old_min = self.CENTER + self.BLIND
                old_max = 255
                new_min = min
                new_max = max

                scaled_right_joystick[0] = new_min + ((corrected_right_joystick[0] - old_min) * (new_max - new_min)) / (old_max - old_min)

            if corrected_right_joystick[1] < self.CENTER:  # y-axis, below center
                old_min = 0
                old_max = self.CENTER - self.BLIND
                new_min = max  # Adjusted to start from max (negative direction)
                new_max = min

                scaled_right_joystick[1] = new_min - ((corrected_right_joystick[1] - old_min) * (new_min - new_max)) / (old_max - old_min)
            else:  # y-axis, above center
                old_min = self.CENTER + self.BLIND
                old_max = 255
                new_min = -min
                new_max = -max
                scaled_right_joystick[1] = new_min + ((corrected_right_joystick[1] - old_min) * (new_max - new_min)) / (old_max - old_min)
                
            if corrected_right_joystick[0] == 128:
                scaled_right_joystick[0] = 0 
            if corrected_right_joystick[1] == 128:
                scaled_right_joystick[1] = 0

            return scaled_right_joystick
        else:
            return [0,0]
        
    def adaptive_trigger_press(self,event,trigger):
        '''Returns the adaptive trigger value (L2 or R2)
        Use: Within the read_loop for the controller
        @param event: The event from the controller
        @param trigger: The desired adaptive trigger to check (e.g. 'L2' or 'R2')
        @return: The adaptive trigger value
        '''
        if trigger == 'L2':
            if event.code == 2:
                self.L2_value = event.value
                return self.L2_value
            else:
                return self.L2_value
        if trigger == 'R2':
            if event.code == 5:
                self.R2_value = event.value
                return self.R2_value
            else:
                return self.R2_value

    #fix so does not need to use trigger_press    
    def scaled_adaptive_trigger(self,trigger,min,max,invert = False):
        '''Returns the scaled adaptive trigger value (L2 or R2)
        Use: After updating the adaptive trigger value
        @param trigger: The desired adaptive trigger to scale (e.g. 'L2' or 'R2')
        @param min: The minimum value of the new desired output range
        @param max: The maximum value of the new desired output range
        @param invert: The direction of the trigger press (default is False)
        @return: The scaled adaptive trigger value'''
        if invert == False:
            if trigger == 'L2':
                if self.L2_value == 0:
                    return 0
                else:
                    old_min = 0
                    old_max = 255
                    new_min = min
                    new_max = max
                    return new_min + ((self.L2_value - old_min) * (new_max - new_min)) / (old_max - old_min)
            if trigger == 'R2':
                if self.R2_value == 0:
                    return 0
                else:
                    old_min = 0
                    old_max = 255
                    new_min = min
                    new_max = max
                    return new_min + ((self.R2_value - old_min) * (new_max - new_min)) / (old_max - old_min)
        else:
            if trigger == 'L2':
                if self.L2_value == 0:
                    return 0
                else:
                    old_min = 0
                    old_max = 255
                    new_min = max
                    new_max = min
                    return new_min + ((self.L2_value - old_min) * (new_max - new_min)) / (old_max - old_min)
            if trigger == 'R2':
                if self.R2_value == 0:
                    return 0
                else:
                    old_min = 0
                    old_max = 255
                    new_min = max
                    new_max = min
                    return new_min + ((self.R2_value - old_min) * (new_max - new_min)) / (old_max - old_min)
            