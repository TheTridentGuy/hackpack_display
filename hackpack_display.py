import LCD1inch8
import time
from PIL import Image, ImageDraw, ImageFont
import psutil
import ipaddress
import subprocess


FONT_HEIGHT = 12
FONT_LG_HEIGHT = 28
PADDING = 2

lcd = LCD1inch8.LCD1Inch8()
font = ImageFont.load_default()
font_sm = ImageFont.load_default(8)
font_lg = ImageFont.load_default(25)

critical_animate = False


def cut_string(string, length):
    if len(string) <= length:
        return string
    else:
        return string[:(length-3)]+"..."


def draw_info(canvas):
    global critical_animate
    y = 0

    temps = psutil.sensors_temperatures()
    cpu = temps["cpu_thermal"][0]
    color = "green"
    if cpu.high:
        color = "yellow"
    elif cpu.critical:
        if critical_animate:
            canvas.rectangle((PADDING, y, 160, PADDING+FONT_LG_HEIGHT), "red")
            color = "black"
        else:
            color = "red"
        critical_animate = not critical_animate
    canvas.text((PADDING+8, y), f"{cpu.current:2.1f}°C", color, font_lg)
    canvas.text((PADDING, y+4), f"C", color, font_sm)
    canvas.text((PADDING, y+11), f"P", color, font_sm)
    canvas.text((PADDING, y+18), f"U", color, font_sm)
    y += FONT_LG_HEIGHT

    interfaces = psutil.net_if_addrs()
    interface_stats = psutil.net_if_stats()
    for ifname, addr in interfaces.items():
        if "wlan" in ifname:
            if interface_stats[ifname].isup:
                formatted_address = str(ipaddress.ip_interface(addr[0].address + "/" + addr[0].netmask))
            else:
                formatted_address = "DOWN"
            line_1 = f"{ifname}: {formatted_address}"
            # iw output isn't stable, expect bugs here
            iw_info = subprocess.check_output(f"iw dev {ifname} info".split(" ")).decode("utf-8")
            if "type monitor" in iw_info:
                line_1 += " (mon)"
            elif "type AP" in iw_info:
                line_1 += " (ap)"
            canvas.text((PADDING, y+PADDING), line_1, "white", font)
            y += FONT_HEIGHT

    
try:
    lcd.set_backlight(60)
    while True:
        image = Image.new("RGB", (lcd.width, lcd.height), "black")
        canvas = ImageDraw.Draw(image)
        draw_info(canvas)
        lcd.display(image)
        time.sleep(1)

except KeyboardInterrupt:
    lcd.deinit()