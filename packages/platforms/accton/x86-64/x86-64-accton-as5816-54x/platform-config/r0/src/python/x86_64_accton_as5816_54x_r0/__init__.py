from onl.platform.base import *
from onl.platform.accton import *

class OnlPlatform_x86_64_accton_as5816_54x_r0(OnlPlatformAccton,
                                              OnlPlatformPortConfig_48x10_6x40):

    PLATFORM='x86-64-accton-as5816-54x-r0'
    MODEL="AS5816-54X"
    SYS_OBJECT_ID=".5816.54.1"

    def baseconfig(self):
        self.insmod('cpr_4011_4mxx')
        self.insmod("ym2651y")
        for m in [ 'cpld', 'fan', 'psu', 'leds', 'sfp' ]:
            self.insmod("x86-64-accton-as5816-54x-%s.ko" % m)
        
        ########### initialize pca9548 at cpu board###########
        self.new_i2c_devices(
            [
                # initiate multiplexer (PCA9548)
                ('pca9548', 0x77, 0),
                ]
            )
        ########### initialize I2C bus 1 ###########

        # initialize CPLDs
        self.new_i2c_devices(
            [
                ('as5816_54x_cpld1', 0x60, 1),
                ('as5816_54x_cpld2', 0x61, 1),
                ('as5816_54x_cpld3', 0x62, 1),
                ]
            )
        # initialize SFP devices
        for port in range(1, 49):
            self.new_i2c_device('as5816_54x_sfp%d' % port, 0x50, port+8)
            self.new_i2c_device('as5816_54x_sfp%d' % port, 0x51, port+8)

        # Initialize QSFP devices
        self.new_i2c_device('as5816_54x_sfp49', 0x50, 57)
        self.new_i2c_device('as5816_54x_sfp52', 0x50, 58)
        self.new_i2c_device('as5816_54x_sfp50', 0x50, 59)
        self.new_i2c_device('as5816_54x_sfp53', 0x50, 60)
        self.new_i2c_device('as5816_54x_sfp51', 0x50, 61)
        self.new_i2c_device('as5816_54x_sfp54', 0x50, 62)

        ########### initialize I2C bus 2 ###########
        self.new_i2c_devices(
            [
                # initiate multiplexer (PCA9548)
                ('pca9548', 0x70, 2),

                # initiate PSU-1 AC Power
                ('as5816_54x_psu1', 0x38, 64),
                ('cpr_4011_4mxx',   0x3c, 64),
                ('as5816_54x_psu1', 0x50, 64),
                ('ym2401',  0x58, 64),

                # initiate PSU-2 AC Power
                ('as5816_54x_psu2', 0x3b, 65),
                ('cpr_4011_4mxx',  0x3f, 65),
                ('as5816_54x_psu2', 0x53, 65),
                ('ym2401',  0x5b, 65),

                # initiate lm75
                ('lm75', 0x48, 68),
                ('lm75', 0x49, 69),
                ('lm75', 0x4a, 70),

                # System EEPROM
                ('24c02', 0x54, 0),
                ]
            )
        return True
