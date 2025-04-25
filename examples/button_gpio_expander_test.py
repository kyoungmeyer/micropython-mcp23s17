from Pico_MCP23S17 import MCP23S17
from machine import Pin

try:
    mcp = MCP23S17(address=0x00, sck=2, mosi=3, miso=4, cs=5)
    mcp.open()
    led = Pin(25, Pin.OUT, value=0)



    mcp.setDirPORTA(mcp.DIR_INPUT)
    mcp.setDirPORTB(mcp.DIR_INPUT)

    mcp.setPullupPORTA(mcp.PULLUP_ENABLED)
    mcp.setPullupPORTB(mcp.PULLUP_ENABLED)

    """
    Alternatively, can also be done by:
    
    for x in range(0, 16):
        mcp.setDirection(x, mcp.DIR_INPUT)
        mcp.setPullupMode(x, mcp.PULLUP_ENABLED)

    or:

    mcp.setDirByPort(mcp.MCP23S17_PORTA, mcp.DIR_OUTPUT)
    mcp.setDirByPort(mcp.MCP23S17_PORTB, mcp.DIR_OUTPUT)
    
    mcp.setPullupByPort(mcp.MCP23S17_PORTA, mcp.PULLUP_ENABLED)
    mcp.setPullupByPort(mcp.MCP23S17_PORTB, mcp.PULLUP_ENABLED)

    """

    old_btn = 1

    print("Starting reading pin A7 (CTRL+C to quit)")
    while True:
        new_btn = mcp.digitalRead(7)
        if new_btn != old_btn:
            if new_btn == 0:
                print("Button pressed!")
                led.value(1)
            else:
                print("Button released!")
                led.value(0)

        old_btn = new_btn

finally:
    mcp.close()