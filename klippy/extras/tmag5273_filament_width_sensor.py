#
# Forked from hall_filament_width_sensor.py from Klipper3d/klipper
# Originally Written By - Mustafa YILDIZ <mydiz@hotmail.com>
#

import logging
from . import filament_switch_sensor
from . import bus
import statistics
import time

TMAG5273_DEVICE_ID = 0x5449

# TMAG REGISTER ADDRESSES
TMAG5273_REG = {
    'CONFIG': {
        'DEVICE_1': 0x00,
        'DEVICE_2': 0x01,
        'SENSOR_1': 0x02,
        'SENSOR_2': 0x03,
        'TEMP_EN': 0x07,
        'MAG_GAIN': 0x09
    },
    'RANGE': {
        '40_MT': 0x0,
        '80_MT': 0x1,
    }
}

# TMAG COMMAND DICTIONARY
TMAG5273_CMD = {
    'GET_DATA': {
        'T': {
            'MSB': 0x10,
            'LSB': 0x11
        },
        'X': {
            'MSB': 0x12,
            'LSB': 0x13
        },
        'Y': {
            'MSB': 0x14,
            'LSB': 0x15
        },
        'Z': {
            'MSB': 0x16,
            'LSB': 0x17
        },
    },
    'AVERAGING': {
        '1X' : 0x0,
        '2X' : 0x1,
        '4X' : 0x2,
        '8X' : 0x3,
        '16X': 0x4,
        '32X': 0x5
    },
    'RANGE': {
        '40_MT': 0x0,
        '80_MT': 0x1
    },
    'DEVICE_ID': {
        'MSB': 0x0F,
        'LSB': 0x0E
    }
}


class TMAGHallFilamentWidthSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        # --- Pull Parameters from configuration file ---
        self.dual_sensor = config.getboolean('dual_sensor', False)
        self.TMAG5273_ADDR_1 = config.getint('sens_1_i2c_addr', 0x22)
        self.TMAG5273_ADDR_2 = config.getint('sens_2_i2c_addr', 0x78)
        self.i2c_1 = bus.MCU_I2C_from_config(config, default_addr=self.TMAG5273_ADDR_1, default_speed=100000)
        if self.dual_sensor == True:
            self.i2c_2 = bus.MCU_I2C_from_config(config, default_addr=self.TMAG5273_ADDR_2, default_speed=100000)
        self.extruder_update_time = config.getfloat('extruder_update_time', 1)
        self.sample_time = config.getfloat('sample_time', 0.01)
        #   Calibration Data
        self.cal_slope_1 = config.getfloat('cal_slope_1', -70000)
        self.cal_slope_2 = config.getfloat('cal_slope_2', -70000)
        self.cal_intercept_1 = config.getfloat('cal_intercept_1', 2.03)
        self.cal_intercept_2 = config.getfloat('cal_intercept_2', 2.03)
        #   Filament Diameter Add-On Settings
        self.MEASUREMENT_INTERVAL_MM=config.getint('measurement_interval',2)
        self.nominal_filament_dia = config.getfloat('default_nominal_filament_diameter', above=1)
        self.measurement_delay_1 = config.getfloat('measurement_delay_1', 1, above=0)
        self.measurement_delay_2 = config.getfloat('measurement_delay_2', 1, above=1)
        self.measurement_max_difference = config.getfloat('max_difference', .25)
        self.max_diameter = (self.nominal_filament_dia + self.measurement_max_difference)
        self.min_diameter = (self.nominal_filament_dia - self.measurement_max_difference)
        self.is_active = config.getboolean('enable', False)
        self.runout_dia_min=config.getfloat('min_diameter', 1.0)
        self.runout_dia_max=config.getfloat('max_diameter', self.max_diameter)
        self.is_log =config.getboolean('logging', False)
        self.use_current_dia_while_delay = config.getboolean('use_current_dia_while_delay', False)
        #   TMAG 5273 Settings
        self.opMode = config.getint('operating_mode', 2)
        self.avgMode = config.getint('average_mode', 5, minval=0, maxval=5)
        self.super_sampling = config.getint('super_sampling', 75, minval=1, maxval=100) # Additional Super Sampling
        self.glitchFilter = config.getboolean('glitch_filter', True) # Inverted
        self.magTempMode = config.getint('mag_temp_mode', 1)
        self.powerMode = config.getboolean('low_noise_mode', True)
        self.magGain = config.getint('magnetic_gain', 0)
        self.tempEn = config.getboolean('temperature_enable', True)
        self.xyAxisRange = config.getboolean('xy_axis_range_80mT', True)
        self.zAxisRange = config.getboolean('z_axis_range_80mT', True)
        self.magneticChannel = config.getint('magnetic_channel', 4)

        # --- Runtime Variables ---
        self.diameter_1 = self.nominal_filament_dia
        self.diameter_2 = self.nominal_filament_dia
        self.t_out_1 = 0.
        self.t_out_2 = 0.
        self.filament_array_1 = [] # filament array [e_position, filamentWidth]
        self.filament_array_2 = [] # filament array [e_position, filamentWidth]
        self.lastFilamentWidthReading_1 = 0
        self.lastFilamentWidthReading_2 = 0
        self.firstExtruderUpdatePosition_1 = 0
        self.firstExtruderUpdatePosition_2 = 0
        self.filament_width_1 = self.nominal_filament_dia
        self.filament_width_2 = self.nominal_filament_dia
        self.last_filament_flow_rate = 1.0
        self.last_epos_status = 0

        # --- Load Printer Objects ---
        self.toolhead = self.gcode_move = self.ppins = self.mcu_adc = None
        self.sample_timer = self.reactor.register_timer(self.sample_tmag5273)
        self._load_commands_init()

        # --- Initialize Schedulers/Timers ---
        self.extrude_factor_update_timer = self.reactor.register_timer(self.extrude_factor_update_event)
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_connect(self):
        self._init_tmag()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def _handle_ready(self):
        # Load printer objects
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move')
        # Start extrude factor update timer
        self.reactor.update_timer(self.extrude_factor_update_timer,
                                  self.reactor.NOW)
    
    def _init_tmag(self):
        """Initialize the Diameter Sensor, configures sensor registers based on config file
        """
        # Ensure Connected Device is Correct
        device_id_msb = self.read_reg(self.i2c_1, TMAG5273_CMD['DEVICE_ID']['MSB'])
        device_id_lsb = self.read_reg(self.i2c_1, TMAG5273_CMD['DEVICE_ID']['LSB'])
        self.device_id = (device_id_msb << 8) | device_id_lsb
        if self.device_id != TMAG5273_DEVICE_ID:
            raise self.printer.command_error(
                f"Invalid TMAG5273 Device id (got {self.device_id}).\n"
                f"Expected {TMAG5273_DEVICE_ID} for sensor 1 .\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty chip.")
        
        # Sensor 1 Configuration
        self.setMagneticChannel(self.i2c_1, self.magneticChannel)
        self.setTemperatureEn(self.i2c_1, self.tempEn)
        self.setOperatingMode(self.i2c_1, self.opMode)
        self.setLowPower(self.i2c_1, self.powerMode)
        self.setXYAxisRange(self.i2c_1, self.xyAxisRange)
        self.setZAxisRange(self.i2c_1, self.zAxisRange)
        self.setGlitchFilter(self.i2c_1, self.glitchFilter)
        self.setConvAvg(self.i2c_1, self.avgMode)
        self.setMagTemp(self.i2c_1, self.magTempMode)

        # Sensor 2 Configuration
        if self.dual_sensor == True:
            device_id_msb = self.read_reg(self.i2c_2, TMAG5273_CMD['DEVICE_ID']['MSB'])
            device_id_lsb = self.read_reg(self.i2c_2, TMAG5273_CMD['DEVICE_ID']['LSB'])
            self.device_id = (device_id_msb << 8) | device_id_lsb
            if self.device_id != TMAG5273_DEVICE_ID:
                raise self.printer.command_error(
                    f"Invalid TMAG5273 Device id (got {self.device_id}).\n"
                    f"Expected {TMAG5273_DEVICE_ID} for sensor 2 .\n"
                    "This is generally indicative of connection problems\n"
                    "(e.g. faulty wiring) or a faulty chip.")
            self.setMagneticChannel(self.i2c_2, self.magneticChannel)
            self.setTemperatureEn(self.i2c_2, self.tempEn)
            self.setOperatingMode(self.i2c_2, self.opMode)
            self.setLowPower(self.i2c_2, self.powerMode)
            self.setXYAxisRange(self.i2c_2, self.xyAxisRange)
            self.setZAxisRange(self.i2c_2, self.zAxisRange)
            self.setGlitchFilter(self.i2c_2, self.glitchFilter)
            self.setConvAvg(self.i2c_2, self.avgMode)
            self.setMagTemp(self.i2c_2, self.magTempMode)

    def _TMAG_M221_BYPASS(self, extrude_factor):
        """Bypass lookahead to apply extrusion multiplier
        """
        new_extrude_factor = extrude_factor
        try:
            gcode_move = self.printer.lookup_object('gcode_move')
            gcode_move.M221_BYPASS(new_extrude_factor)
        except:
            self.gcode.respond_info("M221 Bypass Error")      
  
    def sample_tmag5273(self, eventtime):
        """Query sensors for diameter and temperature data.
        """
        try:
            sen_1_buffer = []
            for i in range(0,self.super_sampling):
                self.z_msb_1 = self.read_reg(self.i2c_1, TMAG5273_CMD['GET_DATA']['Z']['MSB'])
                self.z_lsb_1 = self.read_reg(self.i2c_1, TMAG5273_CMD['GET_DATA']['Z']['LSB'])
                self.z_raw_1 = self.z_lsb_1 + (self.z_msb_1 << 8)
                if self.z_raw_1 >= 32768:  # If the value exceeds 16-bit signed max
                    self.z_raw_1 -= 65536  # Convert to a negative number
                sen_1_buffer.append(self.z_raw_1)
            self.z_raw_1 = statistics.mean(sen_1_buffer)
            self.z_out_1 = self.z_raw_1
            self.lastFilamentWidthReading_1 = self.z_out_1
            diameter_new_1 = round((self.lastFilamentWidthReading_1*self.cal_slope_1)+self.cal_intercept_1,5)
            self.diameter_1 = diameter_new_1

            # Grab Temp
            self.t_msb_1 = self.read_reg(self.i2c_1, TMAG5273_CMD['GET_DATA']['T']['MSB'])
            self.t_lsb_1 = self.read_reg(self.i2c_1, TMAG5273_CMD['GET_DATA']['T']['LSB'])
            self.t_raw_1 = self.t_lsb_1 + (self.t_msb_1 << 8)
            self.t_out_1 = 25 + ((self.t_raw_1 - 17508) / 60.1)
                
            if self.dual_sensor:
                sen_2_buffer = []
                for i in range(0,self.super_sampling):
                    self.z_msb_2 = self.read_reg(self.i2c_2, TMAG5273_CMD['GET_DATA']['Z']['MSB'])
                    self.z_lsb_2 = self.read_reg(self.i2c_2, TMAG5273_CMD['GET_DATA']['Z']['LSB'])
                    self.z_raw_2 = self.z_lsb_2 + (self.z_msb_2 << 8)
                    if self.z_raw_2 >= 32768:  # If the value exceeds 16-bit signed max
                        self.z_raw_2 -= 65536  # Convert to a negative number
                    sen_2_buffer.append(self.z_raw_2)
                self.z_raw_2 = statistics.mean(sen_2_buffer)
                self.z_out_2 = self.z_raw_2
                # Grab Temp
                self.t_msb_2 = self.read_reg(self.i2c_2, TMAG5273_CMD['GET_DATA']['T']['MSB'])
                self.t_lsb_2 = self.read_reg(self.i2c_2, TMAG5273_CMD['GET_DATA']['T']['LSB'])
                self.t_raw_2 = self.t_lsb_2 + (self.t_msb_2 << 8)
                self.t_out_2 = 25 + ((self.t_raw_2 - 17508) / 60.1)
                self.lastFilamentWidthReading_2 = self.z_out_2
                diameter_new_2 = round((self.lastFilamentWidthReading_2*self.cal_slope_2)+self.cal_intercept_2,5)
                self.diameter_2 = diameter_new_2
            
        except Exception:
            logging.exception("TMAG5273: Error reading data")
            return self.reactor.NEVER

        measured_time = self.reactor.monotonic()
        return measured_time + self.sample_time


    def update_filament_buffer(self, last_epos):
        """Updates filament diameter buffer
        """
        if len(self.filament_array_1) > 0:
            # Get last reading position in array & calculate next
            # reading position
            next_reading_position_1 = (self.filament_array_1[-1][0] + self.MEASUREMENT_INTERVAL_MM)

            if next_reading_position_1 <= (last_epos + self.measurement_delay_1):
                    self.filament_array_1.append([last_epos + self.measurement_delay_1, self.diameter_1])
                    if self.is_log:
                        self.gcode.respond_info("Filament Width Sensor 1: %.3f mm" % ( self.diameter_1 ))

            if self.dual_sensor:
                if len(self.filament_array_2) > 0:
                    next_reading_position_2 = (self.filament_array_2[-1][0] + self.MEASUREMENT_INTERVAL_MM)
                    if next_reading_position_2 <= (last_epos + self.measurement_delay_2):
                        self.filament_array_2.append([last_epos + self.measurement_delay_2, self.diameter_2])
                        if self.is_log:
                            self.gcode.respond_info("Filament Width Sensor 2: %.3f mm" % ( self.diameter_2 ))

        else:
            self.filament_array_1.append([self.measurement_delay_1 + last_epos, self.diameter_1])
            self.firstExtruderUpdatePosition_1 = (self.measurement_delay_1 + last_epos)
            if self.dual_sensor:
                self.filament_array_2.append([self.measurement_delay_2 + last_epos, self.diameter_2])
                self.firstExtruderUpdatePosition_2 = (self.measurement_delay_2 + last_epos)


    def extrude_factor_update_event(self, eventtime):
        """Primary function that manipulates the flow rate in real time
        """
        pos = self.toolhead.get_position() # Live Toolhead Position
        # pos = self.gcode_move._get_gcode_position() # GCODE Commanded Position
        last_epos = pos[3]
        self.last_epos_status = last_epos
        self.update_filament_buffer(last_epos)
        if self.diameter_1 > 0.5:
            
            # --- Get next filament width from buffer if needed ---
            if len(self.filament_array_1) > 0:
                pending_position = self.filament_array_1[0][0]
                if pending_position <= last_epos:
                    item = self.filament_array_1.pop(0)
                    self.filament_width_1 = item[1]
                else:
                    if ((self.use_current_dia_while_delay) and (self.firstExtruderUpdatePosition_1 == pending_position)):
                        self.filament_width_1 = self.diameter_1
                    elif  self.firstExtruderUpdatePosition_1 == pending_position:
                        self.filament_width_1 = self.nominal_filament_dia

            if self.dual_sensor:
                if self.diameter_2 > 0.5:
                    if len(self.filament_array_2) > 0:
                        pending_position_2 = self.filament_array_2[0][0]
                        if pending_position_2 <= last_epos:
                            # Get first item in filament_array queue
                            item = self.filament_array_2.pop(0)
                            self.filament_width_2 = item[1]
                        else:
                            if ((self.use_current_dia_while_delay) and (self.firstExtruderUpdatePosition_2 == pending_position_2)):
                                self.filament_width_2 = self.diameter_2
                            elif  self.firstExtruderUpdatePosition_2 == pending_position_2:
                                self.filament_width_2 = self.nominal_filament_dia
            # -----------------------------------------------------

            # --- Calculate out flow rate from diameter ---
            if not self.dual_sensor:
                # Only runs if single sensor
                if ((self.filament_width_1 <= self.max_diameter)
                    and (self.filament_width_1 >= self.min_diameter)):
                    percentage = self.nominal_filament_dia**2 / self.filament_width_1**2 * 100
                    self.last_filament_flow_rate = percentage
                    # percentage = round(percentage)
                    # self.gcode.run_script("M221 S" + str(percentage))
                    self._TMAG_M221_BYPASS(percentage / 100)

                else:
                    self.gcode.run_script("M221 S100")
            else:
                # Runs if dual sensor
                mean_width = (self.filament_width_1 + self.filament_width_2)/2
                if ((mean_width <= self.max_diameter) and (mean_width >= self.min_diameter)):
                        percentage = self.nominal_filament_dia**2 / (self.filament_width_1 * self.filament_width_2) * 100
                        self.last_filament_flow_rate = percentage
                        # percentage = round(percentage)
                        # self.gcode.run_script("M221 S" + str(percentage))
                        self._TMAG_M221_BYPASS(percentage / 100)
                else:
                    self.gcode.run_script("M221 S100")
                    self.last_filament_flow_rate = 100
            # ---------------------------------------------


        else:
            # If diameter less than 0.5 set flow rate to 100%
            self.gcode.run_script("M221 S100")
            self.filament_array_1 = []
            self.filament_array_2 = []

        if self.is_active:
            return eventtime + self.extruder_update_time
        else:
            return self.reactor.NEVER
        
    # --- This section is for interfacing with the sensor registers ---
    def bitWrite(self, register, index, val):
        """Writes value to register by index position.
        
        Args:
            register:
            index:
            val:

        Returns:
            Returns the complete register value
        """
        mask = 1 << index   # Compute mask, an integer with just bit 'index' set.
        register &= ~mask          # Clear the bit indicated by the mask (if x is False)
        if val:
            register |= mask         # If x was True, set the bit indicated by the mask.
        return register            # Return the result, we're done.

    def set_reg(self, i2c_dev, reg, val, minclock=0):
        i2c_dev.i2c_write([reg, val & 0xFF], minclock=minclock)

    def read_reg(self, i2c_dev, reg):
        params = i2c_dev.i2c_read([reg], 1)
        return bytearray(params['response'])[0]


    def setConvAvg(self, i2c_dev, avgMode):
        """Sets the super sampling average mode

        Args:
            i2c_dev: The target I2C device for the command
            avgMode: The super sampling rate
        """
        mode = self.read_reg(i2c_dev, TMAG5273_REG['CONFIG']['DEVICE_1'])

        if avgMode == 0x0: # 0b0000
            mode &= ~(1 << 2)  # Clear bit 2
            mode &= ~(1 << 3)  # Clear bit 3
            mode &= ~(1 << 4)  # Clear bit 4
        if avgMode == 0x1: # 0b0001
            mode |=  (1 << 2)  # Set bit 2
            mode &= ~(1 << 3)  # Clear bit 3
            mode &= ~(1 << 4)  # Clear bit 4
        if avgMode == 0x2: # 0b0010
            mode &= ~(1 << 2)  # Clear bit 2
            mode |=  (1 << 3)  # Set bit 3
            mode &= ~(1 << 4)  # Clear bit 4
        if avgMode == 0x3: # 0b0011
            mode |=  (1 << 2)  # Set bit 2
            mode |=  (1 << 3)  # Set bit 3
            mode &= ~(1 << 4)  # Clear bit 4
        if avgMode == 0x4: # 0b0100
            mode &= ~(1 << 2)  # Clear bit 2
            mode &= ~(1 << 3)  # Clear bit 3
            mode |=  (1 << 4)  # Set bit 4
        if avgMode == 0x5: # 0b0101
            mode |=  (1 << 2)  # Set bit 2
            mode &= ~(1 << 3)  # Clear bit 3
            mode |=  (1 << 4)  # Set bit 4
        self.set_reg(i2c_dev, TMAG5273_REG['CONFIG']['DEVICE_1'], mode)

    def setMagneticChannel(self, i2c_dev, channelMode):
        """Set the magnetic channels, should only ever be Z but implemented all options.

        Args:
            i2c_dev: The target I2C device for the command
            channelMode: The channel setting options
        """
        mode = self.read_reg(i2c_dev, TMAG5273_REG['CONFIG']['SENSOR_1'])
        
        if channelMode == 0x0: # 0b0000
            mode &= ~(1 << 4)  # Clear bit 4
            mode &= ~(1 << 5)  # Clear bit 5
            mode &= ~(1 << 6)  # Clear bit 6
            mode &= ~(1 << 7)  # Clear bit 7
        if channelMode == 0x1: # 0b0001
            mode |=  (1 << 4)  # Set bit 4
            mode &= ~(1 << 5)  # Clear bit 5
            mode &= ~(1 << 6)  # Clear bit 6
            mode &= ~(1 << 7)  # Clear bit 7
        if channelMode == 0x2: # 0b0010
            mode &= ~(1 << 4)  # Clear bit 4
            mode |=  (1 << 5)  # Set bit 5
            mode &= ~(1 << 6)  # Clear bit 6
            mode &= ~(1 << 7)  # Clear bit 7
        if channelMode == 0x3: # 0b0011
            mode |=  (1 << 4)  # Set bit 4
            mode |=  (1 << 5)  # Set bit 5
            mode &= ~(1 << 6)  # Clear bit 6
            mode &= ~(1 << 7)  # Clear bit 7
        if channelMode == 0x4: # 0b0100
            mode &= ~(1 << 4)  # Clear bit 4
            mode &= ~(1 << 5)  # Clear bit 5
            mode |=  (1 << 6)  # Set bit 6
            mode &= ~(1 << 7)  # Clear bit 7
        if channelMode == 0x5: # 0b0101
            mode |=  (1 << 4)  # Set bit 4
            mode &= ~(1 << 5)  # Clear bit 5
            mode |=  (1 << 6)  # Set bit 6
            mode &= ~(1 << 7)  # Clear bit 7
        self.set_reg(i2c_dev, TMAG5273_REG['CONFIG']['SENSOR_1'], mode)

    def setTemperatureEn(self, i2c_dev, tempEnable):
        mode = self.read_reg(i2c_dev, TMAG5273_REG['CONFIG']['TEMP_EN'])
        
        if tempEnable == False:  # 0b0
            mode &= ~(1 << 7)  # Clear bit 7
        elif tempEnable == True:  # 0b1
            mode |=  (1 << 7)  # Set bit 7
        self.set_reg(i2c_dev, TMAG5273_REG['CONFIG']['TEMP_EN'], mode)

    def setXYAxisRange(self, i2c_dev, mag_range):
        mode = self.read_reg(i2c_dev, TMAG5273_REG['CONFIG']['SENSOR_2'])

        if mag_range == False:  # 0b0
            mode &= ~(1 << 1)  # Clear bit 1
        elif mag_range == True:  # 0b1
            mode |=  (1 << 1)  # Set bit 1
        self.set_reg(i2c_dev, TMAG5273_REG['CONFIG']['SENSOR_2'], mode)

    def setZAxisRange(self, i2c_dev, mag_range):
        mode = self.read_reg(i2c_dev, TMAG5273_REG['CONFIG']['SENSOR_2'])

        if mag_range == False:  # 0b0
            mode &= ~(1 << 0)  # Clear bit 
        elif mag_range == True:  # 0b1
            mode |=  (1 << 0)  # Set bit 0
        self.set_reg(i2c_dev, TMAG5273_REG['CONFIG']['SENSOR_2'], mode)

    def setMagTemp(self, i2c_dev, magTempMode):
        mode = self.read_reg(i2c_dev, TMAG5273_REG['CONFIG']['DEVICE_1'])

        # Set bits 6 and 5 based on mag_temp_mode using bitwise operations
        if magTempMode == 0x0:  # 0b00
            mode &= ~(1 << 5)  # Clear bit 5
            mode &= ~(1 << 6)  # Clear bit 6
        elif magTempMode == 0x1:  # 0b01
            mode |= (1 << 5)   # Set bit 5
            mode &= ~(1 << 6)  # Clear bit 6
        elif magTempMode == 0x2:  # 0b10
            mode &= ~(1 << 5)  # Clear bit 5
            mode |= (1 << 6)   # Set bit 6
        elif magTempMode == 0x3:  # 0b11
            mode |= (1 << 5)   # Set bit 5
            mode |= (1 << 6)   # Set bit 6
        self.set_reg(i2c_dev, TMAG5273_REG['CONFIG']['DEVICE_1'], mode)

    def setOperatingMode(self, i2c_dev, oPmode):
        mode = self.read_reg(i2c_dev, TMAG5273_REG['CONFIG']['DEVICE_2'])

        # Set bits 6 and 5 based on mag_temp_mode using bitwise operations
        if oPmode == 0:  # 0b00
            mode &= ~(1 << 0)  # Clear bit 0
            mode &= ~(1 << 1)  # Clear bit 1
        elif oPmode == 1:  # 0b01
            mode |= (1 << 0)   # Set bit 0
            mode &= ~(1 << 1)  # Clear bit 1
        elif oPmode == 2:  # 0b10
            mode &= ~(1 << 0)  # Clear bit 0
            mode |= (1 << 1)   # Set bit 1
        elif oPmode == 3:  # 0b11
            mode |= (1 << 0)   # Set bit 0
            mode |= (1 << 1)   # Set bit 1
        self.set_reg(i2c_dev, TMAG5273_REG['CONFIG']['DEVICE_2'], mode)

    def setLowPower(self, i2c_dev, powerMode):
        mode = self.read_reg(i2c_dev, TMAG5273_REG['CONFIG']['DEVICE_2'])

        if powerMode == False:  # 0b0
            mode &= ~(1 << 4)  # Clear bit 4
        elif powerMode == True:  # 0b1
            mode |=  (1 << 4)  # Set bit 4
        self.set_reg(i2c_dev, TMAG5273_REG['CONFIG']['DEVICE_2'], mode)

    def setGlitchFilter(self, i2c_dev, glitchMode):
        mode = self.read_reg(i2c_dev, TMAG5273_REG['CONFIG']['DEVICE_2'])

        glitchMode = not glitchMode # Invert as the register defaults to enabled
        if glitchMode == False:  # 0b0
            mode &= ~(1 << 3)  # Clear bit 3
        elif glitchMode == True:  # 0b1
            mode |=  (1 << 3)  # Set bit 3
        self.set_reg(i2c_dev, TMAG5273_REG['CONFIG']['DEVICE_2'], mode)
    
    # --- End of section for interfacing with sensor registers ---

    
    # Custom GCODE Commands for Diameter Sensor
    def _load_commands_init(self):
        """Register Custom GCODE Commands with Klipper
        """
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('QUERY_FILAMENT_WIDTH', self.cmd_M407)
        self.gcode.register_command('RESET_FILAMENT_WIDTH_SENSOR',
                                    self.cmd_ClearFilamentArray)
        self.gcode.register_command('DISABLE_FILAMENT_WIDTH_SENSOR',
                                    self.cmd_M406)
        self.gcode.register_command('ENABLE_FILAMENT_WIDTH_SENSOR',
                                    self.cmd_M405)
        self.gcode.register_command('QUERY_RAW_FILAMENT_WIDTH',
                                    self.cmd_Get_Raw_Values)
        self.gcode.register_command('ENABLE_FILAMENT_WIDTH_LOG',
                                    self.cmd_log_enable)
        self.gcode.register_command('DISABLE_FILAMENT_WIDTH_LOG',
                                    self.cmd_log_disable)
        self.gcode.register_command('GET_FILAMENT_WIDTH_SENSOR_DEVICE_ID',
                                    self.cmd_get_device_id)
        self.gcode.register_command('GET_FILAMENT_WIDTH_SENSOR_REGISTERS',
                                    self.cmd_get_device_regs)
        self.gcode.register_command('GET_FILAMENT_WIDTH_SENSOR_TEMPS',
                                    self.cmd_get_sensor_temp)
        self.gcode.register_command('GET_FILAMENT_WIDTH_SENSOR_DEBUG',
                                    self.cmd_Get_Debug_Data)
        self.gcode.register_command('GET_GCODE_MOVE_CONNECTION',
                                    self.cmd_gcode_move_connection_test)
        self.gcode.register_command('DIRECT_M221',
                                    self.cmd_direct_m221)

    def cmd_M407(self, gcmd):
        response = ""
        if self.diameter_1 > 0:
            response += ("Filament dia (measured mm): "
                         + str(self.diameter_1))
        else:
            response += "Filament NOT present"
        gcmd.respond_info(response)

    def cmd_ClearFilamentArray(self, gcmd):
        self.filament_array_1 = []
        self.filament_array_2 = []
        gcmd.respond_info("Filament width measurements cleared!")
        # Set extrude multiplier to 100%
        self.gcode.run_script_from_command("M221 S100")

    def cmd_M405(self, gcmd):
        response = "Filament width sensor Turned On"
        if self.is_active:
            response = "Filament width sensor is already On"
        else:
            self.is_active = True
            # Start extrude factor update timer
            self.reactor.update_timer(self.extrude_factor_update_timer,
                                      self.reactor.NOW)
        gcmd.respond_info(response)

    def cmd_M406(self, gcmd):
        response = "Filament width sensor Turned Off"
        if not self.is_active:
            response = "Filament width sensor is already Off"
        else:
            self.is_active = False
            # Stop extrude factor update timer
            self.reactor.update_timer(self.extrude_factor_update_timer,
                                      self.reactor.NEVER)
            # Clear filament array
            self.filament_array_1 = []
            self.filament_array_2 = []
            # Set extrude multiplier to 100%
            self.gcode.run_script_from_command("M221 S100")
        gcmd.respond_info(response)

    def cmd_Get_Raw_Values(self, gcmd):
        if self.dual_sensor == False:
            response = "ADC1="
            response +=  (" "+str(self.lastFilamentWidthReading_1))
            gcmd.respond_info(response)
        elif self.dual_sensor == True:
            response = "ADC1="
            response +=  (" "+str(self.lastFilamentWidthReading_1))
            response +=  (" ADC2="+str(self.lastFilamentWidthReading_2))
            response +=  (" DIAM_1="+str(self.diameter_1))
            response +=  (" DIAM_2="+str(self.diameter_2))
            response +=  (" FLOW="+str(self.last_filament_flow_rate))
            gcmd.respond_info(response)

    def cmd_Get_Debug_Data(self, gcmd):
        if self.dual_sensor == False:
            response =   ("ADC1: "+str(self.lastFilamentWidthReading_1))
            response +=  (" DIAM_1: "+str(self.diameter_1))
            response +=  (" TEMP_1: "+str(round(self.t_out_1,5)))
            response +=  (" FLOW: "+str(self.last_filament_flow_rate)+"\n")
            response +=  (" E_POS: "+str(self.last_epos_status)+"\n")
            response +=  (" SENS_1_BUFFER: "+str(self.filament_array_1)+"\n")
            gcmd.respond_info(response)
        elif self.dual_sensor == True:
            response =   ("ADC1: "+str(self.lastFilamentWidthReading_1))
            response +=  (" ADC2: "+str(self.lastFilamentWidthReading_2)+"\n")
            response +=  (" DIAM_1: "+str(self.diameter_1))
            response +=  (" DIAM_2: "+str(self.diameter_2)+"\n")
            response +=  (" TEMP_1: "+str(round(self.t_out_1,5)))
            response +=  (" TEMP_2: "+str(round(self.t_out_2,5))+"\n")
            response +=  (" FLOW: "+str(self.last_filament_flow_rate)+"\n")
            response +=  (" E_POS: "+str(self.last_epos_status)+"\n")
            response +=  (" SENS_1_BUFFER: "+str(self.filament_array_1)+"\n")
            response +=  (" SENS_2_BUFFER: "+str(self.filament_array_2)+"\n")
            gcmd.respond_info(response)
                    
    def cmd_get_device_id(self, gcmd):
        gcmd.respond_info(f"Diameter Sensor Device ID-> {self.device_id}")

    def cmd_get_device_regs(self, gcmd):
        dev_1  = self.read_reg(self.i2c_1, TMAG5273_REG['CONFIG']['DEVICE_1'])
        dev_2  = self.read_reg(self.i2c_1, TMAG5273_REG['CONFIG']['DEVICE_2'])
        sens_1 = self.read_reg(self.i2c_1, TMAG5273_REG['CONFIG']['SENSOR_1'])
        sens_2 = self.read_reg(self.i2c_1, TMAG5273_REG['CONFIG']['SENSOR_2'])
        gcmd.respond_info(f"Sensor 1 Registers:\n DEV_1: {dev_1:08b}\n DEV_2: {dev_2:08b}\n SEN_1: {sens_1:08b}\n SEN_2: {sens_2:08b}")
        
        if self.dual_sensor:
            time.sleep(.5) # Added as calling this without delay with dual sensors caused a klipper crash
            dev_1  = self.read_reg(self.i2c_2, TMAG5273_REG['CONFIG']['DEVICE_1'])
            dev_2  = self.read_reg(self.i2c_2, TMAG5273_REG['CONFIG']['DEVICE_2'])
            sens_1 = self.read_reg(self.i2c_2, TMAG5273_REG['CONFIG']['SENSOR_1'])
            sens_2 = self.read_reg(self.i2c_2, TMAG5273_REG['CONFIG']['SENSOR_2'])
            gcmd.respond_info(f"Sensor 2 Registers:\n DEV_1: {dev_1:08b}\n DEV_2: {dev_2:08b}\n SEN_1: {sens_1:08b}\n SEN_2: {sens_2:08b}")

    def cmd_get_sensor_temp(self, gcmd):
        gcmd.respond_info(f"Sensor Temperature SEN_1: {self.t_out_1}")
        if self.dual_sensor:
            gcmd.respond_info(f"Sensor Temperature SEN_2: {self.t_out_2}")
    
    def cmd_log_enable(self, gcmd):
        self.is_log = True
        gcmd.respond_info("Filament width logging Turned On")

    def cmd_log_disable(self, gcmd):
        self.is_log = False
        gcmd.respond_info("Filament width logging Turned Off")

    def cmd_gcode_move_connection_test(self, gcmd):
        """ Example Command: GET_GCODE_MOVE_CONNECTION
        """
        try:
            extrude_factor = self.gcode_move.extrude_factor
            gcmd.respond_info(f"connection_succesful : {extrude_factor}")
        except:
            gcmd.respond_info("Error")

    def cmd_direct_m221(self, gcmd):
        """ Example Command: M221_DIRECT S95.25
        """
        new_extrude_factor = gcmd.get_float('S', 100., above=0.) / 100.
        try:
            self.gcode_move.M221_BYPASS(new_extrude_factor)
        except:
            gcmd.respond_info("Error")

    def get_status(self, eventtime):
        if self.dual_sensor == False:
            return {'sen_1_diameter': self.diameter_1,
                    'sen_1_raw_adc':(self.lastFilamentWidthReading_1),
                    'sen_1_temp': round(self.t_out_1,5),
                    'sen_1_buffer': str(self.filament_array_1),
                    'last_flow_rate': round(self.last_filament_flow_rate,5),
                    'is_active':self.is_active}
        elif self.dual_sensor == True:
            return {'sen_1_diameter': self.diameter_1,
                    'sen_1_temp': round(self.t_out_1,5),
                    'sen_1_raw_adc': (self.lastFilamentWidthReading_1),
                    'sen_1_buffer': str(self.filament_array_1),
                    'sen_2_diameter': self.diameter_2,
                    'sen_2_temp': round(self.t_out_2,5),
                    'sen_2_raw_adc': (self.lastFilamentWidthReading_2),
                    'sen_2_buffer': str(self.filament_array_2),
                    'last_flow_rate': round(self.last_filament_flow_rate,5),
                    'last_ext_pos': (self.last_epos_status),
                    'is_active':self.is_active}
        else:
            raise self.printer.command_error(f"TMAG5273 Error \n")



def load_config(config):
    return TMAGHallFilamentWidthSensor(config)

