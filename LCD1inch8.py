import sys
import time
import spidev
import numpy as np
from gpiozero import DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice


L2R_U2D = 1
L2R_D2U = 2
R2L_U2D = 3
R2L_D2U = 4
U2D_L2R = 5
U2D_R2L = 6
D2U_L2R = 7
D2U_R2L = 8
SCAN_DIRECTION = U2D_R2L

LCD_WIDTH = 160
LCD_HEIGHT = 128

LCD_X_OFFSET = 2
LCD_Y_OFFSET = 1


class LCD1Inch8():
    def __init__(self, spi=spidev.SpiDev(0, 0), spi_freq=40000000, reset_pin=27, dc_pin=25, bl_pin=18, bl_freq=1000):
        self.INPUT = False
        self.OUTPUT = True
        self.SPI_FREQ = spi_freq
        self.BL_FREQ = bl_freq

        self.RESET_PIN = self.gpio_mode(reset_pin, self.OUTPUT)
        self.DC_PIN = self.gpio_mode(dc_pin, self.OUTPUT)
        self.BL_PIN = self.gpio_pwm(bl_pin)
        self.LCD_SCAN_DIRECTION = SCAN_DIRECTION
        self.LCD_X_OFFSET = LCD_X_OFFSET
        self.LCD_Y_OFFSET = LCD_Y_OFFSET
        self.width = LCD_WIDTH
        self.height = LCD_HEIGHT

        self.SPI = spi
        self.SPI.max_speed_hz = self.SPI_FREQ
        self.SPI.mode = 0b00

        self.set_backlight(0)
        self.reset()
        self.init_registers()
        self.set_scan_direction(self.LCD_SCAN_DIRECTION)
        self.delay(200)
        self.write_command(0x11)
        self.delay(120)
        self.write_command(0x29)
        self.clear()

    def gpio_mode(self, pin, mode, pull_up=None, active_state=True):
        if mode:
            return DigitalOutputDevice(pin, active_high=True, initial_value=False)
        else:
            return DigitalInputDevice(pin, pull_up=pull_up, active_state=active_state)

    def digital_write(self, pin, value):
        if value:
            pin.on()
        else:
            pin.off()

    def digital_read(self, pin):
        return pin.value

    def delay(self, ms):
        time.sleep(ms / 1000.0)

    def gpio_pwm(self, pin):
        return PWMOutputDevice(pin, frequency=self.BL_FREQ)

    def spi_write_byte(self, data):
        if self.SPI != None:
            self.SPI.writebytes(data)

    def set_backlight(self, duty_cycle):
        self.BL_PIN.value = duty_cycle / 100

    def set_backlight_frquency(self, freq):
        self.BL_PIN.frequency = freq

    def deinit(self):
        if self.SPI != None:
            self.SPI.close()
        self.digital_write(self.RESET_PIN, 1)
        self.digital_write(self.DC_PIN, 0)
        self.BL_PIN.close()
        time.sleep(0.001)

    def write_command(self, cmd):
        self.digital_write(self.DC_PIN, False)
        self.spi_write_byte([cmd])

    def write_data(self, data):
        self.digital_write(self.DC_PIN, True)
        self.spi_write_byte([data])

    def reset(self):
        self.digital_write(self.RESET_PIN, True)
        self.delay(10)
        self.digital_write(self.RESET_PIN, False)
        self.delay(10)
        self.digital_write(self.RESET_PIN, True)
        self.delay(10)

    def set_scan_direction(self, scan_direction):
        self.LCD_SCAN_DIRECTION = scan_direction
        if scan_direction == L2R_U2D or scan_direction == L2R_D2U or scan_direction == R2L_U2D or scan_direction == R2L_D2U:
            self.width, self.height = self.height, self.width
            if scan_direction == L2R_U2D:
                scan_direction_data = 0X00 | 0x00
            elif scan_direction == L2R_D2U:
                scan_direction_data = 0X00 | 0x80
            elif scan_direction == R2L_U2D:
                scan_direction_data = 0x40 | 0x00
            else:
                scan_direction_data = 0x40 | 0x80
        else:
            self.LCD_X_OFFSET, self.LCD_Y_OFFSET = self.LCD_Y_OFFSET, self.LCD_X_OFFSET
            if scan_direction == U2D_L2R:
                scan_direction_data = 0X00 | 0x00 | 0x20
            elif scan_direction == U2D_R2L:
                scan_direction_data = 0X00 | 0x40 | 0x20
            elif scan_direction == D2U_L2R:
                scan_direction_data = 0x80 | 0x00 | 0x20
            else:
                scan_direction_data = 0x40 | 0x80 | 0x20
        self.write_command(0x36) 
        self.write_data(scan_direction_data & 0xf7)

    def init_registers(self):
        self.write_command(0xB1)
        self.write_data(0x01)
        self.write_data(0x2C)
        self.write_data(0x2D)

        self.write_command(0xB2)
        self.write_data(0x01)
        self.write_data(0x2C)
        self.write_data(0x2D)

        self.write_command(0xB3)
        self.write_data(0x01)
        self.write_data(0x2C)
        self.write_data(0x2D)
        self.write_data(0x01)
        self.write_data(0x2C)
        self.write_data(0x2D)

        self.write_command(0xB4)
        self.write_data(0x07)

        self.write_command(0xC0)
        self.write_data(0xA2)
        self.write_data(0x02)
        self.write_data(0x84)
        self.write_command(0xC1)
        self.write_data(0xC5)

        self.write_command(0xC2)
        self.write_data(0x0A)
        self.write_data(0x00)

        self.write_command(0xC3)
        self.write_data(0x8A)
        self.write_data(0x2A)
        self.write_command(0xC4)
        self.write_data(0x8A)
        self.write_data(0xEE)

        self.write_command(0xC5)
        self.write_data(0x0E)

        self.write_command(0xe0)
        self.write_data(0x0f)
        self.write_data(0x1a)
        self.write_data(0x0f)
        self.write_data(0x18)
        self.write_data(0x2f)
        self.write_data(0x28)
        self.write_data(0x20)
        self.write_data(0x22)
        self.write_data(0x1f)
        self.write_data(0x1b)
        self.write_data(0x23)
        self.write_data(0x37)
        self.write_data(0x00)
        self.write_data(0x07)
        self.write_data(0x02)
        self.write_data(0x10)

        self.write_command(0xe1)
        self.write_data(0x0f)
        self.write_data(0x1b)
        self.write_data(0x0f)
        self.write_data(0x17)
        self.write_data(0x33)
        self.write_data(0x2c)
        self.write_data(0x29)
        self.write_data(0x2e)
        self.write_data(0x30)
        self.write_data(0x30)
        self.write_data(0x39)
        self.write_data(0x3f)
        self.write_data(0x00)
        self.write_data(0x07)
        self.write_data(0x03)
        self.write_data(0x10)

        self.write_command(0xF0)
        self.write_data(0x01)

        self.write_command(0xF6)
        self.write_data(0x00)

        self.write_command(0x3A)
        self.write_data(0x05)

    def set_window(self, x_start, y_start, x_end, y_end):
        self.write_command(0x2A)
        self.write_data(0x00)
        self.write_data((x_start & 0xff) + self.LCD_X_OFFSET)
        self.write_data(0x00)
        self.write_data(((x_end - 1) & 0xff) + self.LCD_X_OFFSET)

        self.write_command(0x2B)
        self.write_data(0x00)
        self.write_data((y_start & 0xff) + self.LCD_Y_OFFSET)
        self.write_data(0x00)
        self.write_data(((y_end - 1) & 0xff) + self.LCD_Y_OFFSET)

        self.write_command(0x2C)

    def clear(self, color=0XFFFF):
        buffer = [color]*(self.width * self.height * 2)
        if (self.LCD_SCAN_DIRECTION == L2R_U2D) or (self.LCD_SCAN_DIRECTION == L2R_D2U) or (self.LCD_SCAN_DIRECTION == R2L_U2D) or (self.LCD_SCAN_DIRECTION == R2L_D2U):
            self.set_window(0, 0, self.width, self.height)
            self.digital_write(self.DC_PIN, True)
            for i in range(0, len(buffer), 4096):
                self.spi_write_byte(buffer[i:i+4096])
        else:
            self.set_window(0, 0, self.height, self.width)
            self.digital_write(self.DC_PIN, True)
            for i in range(0, len(buffer), 4096):
                self.spi_write_byte(buffer[i:i+4096])

    def display(self, image):
        if (image is None):
            return
        image_w, image_h = image.size
        if image_w != self.width or image_h != self.height:
            raise ValueError(f"Image size must equal display size. Image was {image_w}x{image_h}, while display is {self.width}x{self.height}.")
        img = np.asarray(image)
        pixels = np.zeros((self.height, self.width, 2), dtype=np.uint8)
        pixels[..., [0]] = np.add(np.bitwise_and(
            img[..., [0]], 0xF8), np.right_shift(img[..., [1]], 5))
        pixels[..., [1]] = np.add(np.bitwise_and(np.left_shift(
            img[..., [1]], 3), 0xE0), np.right_shift(img[..., [2]], 3))
        pixels = pixels.flatten().tolist()
        self.set_window(0, 0, self.width, self.height)
        self.digital_write(self.DC_PIN, True)
        for i in range(0, len(pixels), 4096):
            self.spi_write_byte(pixels[i:i+4096])
