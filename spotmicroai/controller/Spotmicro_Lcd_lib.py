import signal
import sys
import time


#from spotmicroai.utilities import LCD_16x2_I2C_driver
from spotmicroai.utilities.system import System
#from spotmicroai.utilities.log import Logger
#from spotmicroai.utilities.config import Config

class LCDScreenController:
    log                             = None
    is_alive                        = False
    address                         = 0
    screen                          = None
    clock_tick                      = 0
    status                          = '---'
    step_mode                       = ''
    remote_controller_controller    = None
    lcd_screen_controller           = None
    
    def __init__(self, _address, _log, joystick):
        try:
            self.address                        = _address
            self.log                            = _log
            self.lcd_screen_controller          = 'ON'
            if joystick > 0:
                self.remote_controller_controller   = 'ON'

            self.log.debug('Starting LCD controller...')
            signal.signal(signal.SIGINT, self.exit_gracefully)
            signal.signal(signal.SIGTERM, self.exit_gracefully)

            # i2c_address = int(Config().get(Config.LCD_SCREEN_CONTROLLER_I2C_ADDRESS), 0)
            if (self.address > 0):
                self.screen                 = LCD_16x2_I2C_driver.lcd(address=self.address)

                self.screen.lcd_clear()
                self.update_lcd_screen()
                self.turn_on()
                self.is_alive = True

        except Exception as e:
            self.is_alive = False
            self.log.error('LCD Screen controller initialization problem, module not critical, skipping', e)

    def exit_gracefully(self, signum, frame):
        try:
            self.turn_off()
        finally:
            self.log.info('Terminated')
            sys.exit(0)

    def turn_off(self):
        self.screen.lcd_clear()
        time.sleep(0.1)
        self.screen.backlight(0)

    def turn_on(self):
        self.screen.backlight(1)

    def update_lcd_status(self, status):
        self.status = status
        self.update_lcd_screen()
        
    def update_lcd_screen(self):  # https://www.quinapalus.com/hd44780udg.html

        if self.is_alive == False:
            return

        if self.lcd_screen_controller == 'ON':
            self.turn_on()
        elif self.lcd_screen_controller == 'OFF':
            self.turn_off()
            
        temperature = System().temperature()

        custom_icons = []

        icon_empty = [0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]
        icon_success = [0x0, 0x1, 0x3, 0x16, 0x1c, 0x8, 0x0, 0x0]
        icon_pca9685 = [0x1f, 0x11, 0x15, 0x15, 0x15, 0x15, 0x11, 0x1f]
        icon_gpio = [0x4, 0x4, 0x1f, 0x0, 0x0, 0xe, 0x4, 0x4]
        icon_remote_controller = [0x11, 0xa, 0xe, 0xa, 0xa, 0xe, 0xa, 0x11]
        icon_temperature = [0x18, 0x18, 0x3, 0x4, 0x4, 0x4, 0x3, 0x0]
        icon_problem = [0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0, 0x0]
        icon_success_reverse = [0x1f, 0x1e, 0x1c, 0x9, 0x3, 0x17, 0x1f]

        # There is only memory for 7 in the lcd screen controller
        custom_icons.insert(0, icon_empty)
        custom_icons.insert(1, icon_success)
        custom_icons.insert(2, icon_pca9685)
        custom_icons.insert(3, icon_gpio)
        custom_icons.insert(4, icon_remote_controller)
        custom_icons.insert(5, icon_temperature)
        custom_icons.insert(6, icon_problem)
        custom_icons.insert(7, icon_success_reverse)

        self.screen.lcd_load_custom_chars(custom_icons)

        self.screen.lcd_write(0x80)  # First line

        for char in 'SpotMicro':
            self.screen.lcd_write(ord(char), 0b00000001)

        self.screen.lcd_write_char(0)
        self.screen.lcd_write_char(4)
        self.screen.lcd_write_char(0)
        self.screen.lcd_write_char(3)
        self.screen.lcd_write_char(0)
        self.screen.lcd_write_char(2)
        self.screen.lcd_write_char(2)

        # Write next three chars to row 2 directly
        self.screen.lcd_write(0xC0)  # Second line
        self.screen.lcd_write_char(0)
        self.screen.lcd_write_char(0)
        self.screen.lcd_write_char(0)
        self.screen.lcd_write_char(0)
        self.screen.lcd_write_char(0)

        if temperature:
            for char in temperature.rjust(3, ' '):
                self.screen.lcd_write(ord(char), 0b00000001)
            self.screen.lcd_write_char(5)
        else:
            self.screen.lcd_write_char(0)
            self.screen.lcd_write_char(0)
            self.screen.lcd_write_char(0)
            self.screen.lcd_write_char(0)

        self.screen.lcd_write_char(0)

        if self.remote_controller_controller == 'OK':
            self.screen.lcd_write_char(1)
        elif self.remote_controller_controller == 'SEARCHING':
            self.screen.lcd_write_char(7)
        else:
            self.screen.lcd_write_char(6)

        Rs = 0b00000001 # Register select bit
        self.screen.lcd_write_char(0)
        for char in self.status:
            self.lcd_write(ord(char), Rs)

        #self.screen.lcd_write_char(1)
        self.screen.lcd_write_char(0)

        self.log.info('LCD Screen', self.status)

        #if self.status == 'OK ON':
        #    self.screen.lcd_write_char(1)
        #elif self.abort_controller == 'OK OFF':
        #    self.screen.lcd_write_char(7)
        #else:
        #    self.screen.lcd_write_char(6)
        #self.screen.lcd_write_char(0)
        #if self.motion_controller_1 == 'OK':
        #    self.screen.lcd_write_char(1)
        #else:
        #    self.screen.lcd_write_char(6)
        #if self.motion_controller_2 == 'OK':
        #    self.screen.lcd_write_char(1)
        #else:
        #    self.screen.lcd_write_char(6)