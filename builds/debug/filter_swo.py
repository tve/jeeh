from platformio.public import DeviceMonitorFilterBase

# ITM SWO data format is described in ARMv7-M Architecture Reference Manual, Appendix D "Debug ITM and DWT Packet Protocol"
class SWO(DeviceMonitorFilterBase):
    NAME = "swo"

    def __init__(self, *args, **kwargs):
        # Construct parent
        super().__init__(*args, **kwargs)

        # Reset current payload length
        self.payload_len = 0

    def rx(self, text):
        """Process inbound data"""

        # Init output
        output = ""

        # Process input one character at a time
        for c in text:
            # Process character
            if self.payload_len > 0:
                # Payload bytes remain, pass through
                output += c

                # Decrement payload length
                self.payload_len -= 1

            else:
                # No payload remaining, read next header
                c = ord(c)
                self.payload_len = (c & 0x3) >> 0
                self.payload_src = (c & 0x4) >> 2
                self.itm_port = (c & 0xf8) >> 3

                # fix header 1/2/3 => 1/2/4
                if self.payload_len == 3:
                    self.payload_len = 4

        # Provide processed output
        return output

    def tx(self, text):
        """Process outbound data"""

        # Do nothing to transmitted data
        return text
