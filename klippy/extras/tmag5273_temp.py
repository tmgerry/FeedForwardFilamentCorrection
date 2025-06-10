import logging
from . import bus

class TMAG5273_TEMP:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.report_time = config.getint('tmag_poll_time', 1, minval=1)
        self.temp = self.min_temp = self.max_temp = 0.
        self.deviceId = config.get('sensor_type')
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=0x44, default_speed=100000)
        self.sensor_number = config.getint('sensor_number', 1, minval=1, maxval=2)
        self.sample_timer = self.reactor.register_timer(self._sample_tmag5273_temp)
        self.printer.add_object("tmag5273_temp " + self.name, self)
        # self.gcode = self.printer.lookup_object('gcode')
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        
    def handle_connect(self):
        # self.gcode.respond_info("Temp Sensor Connecting")
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def _sample_tmag5273_temp(self, eventtime):
        try:
            self.tmag5273 = self.printer.lookup_object('tmag5273_filament_width_sensor')
            # self.gcode.respond_info("Temp Sensor Sampled")
            if self.sensor_number == 1:
                self.temp = self.tmag5273.t_out_1
            if self.sensor_number == 2:
                self.temp = self.tmag5273.t_out_2
        except Exception:
            logging.exception("sht3x: Error reading data")
            # self.gcode.respond_info("Temp Sensor Error")
            self.temp = .69
            return self.reactor.NEVER

        measured_time = self.reactor.monotonic()
        print_time = self.i2c.get_mcu().estimated_print_time(measured_time)
        self._callback(print_time, self.temp)
        return measured_time + self.report_time

    def get_status(self, eventtime):
        return {
            'temperature': round(self.temp, 2),
        }

def load_config(config):
    # Register sensor
    pheater = config.get_printer().lookup_object("heaters")
    pheater.add_sensor_factory("TMAG5273_TEMP", TMAG5273_TEMP)