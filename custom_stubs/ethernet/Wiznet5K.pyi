import microcontroller

class Wiznet5K:
    """Wiznet5K Ethernet Controller Interface
    Initializes a Wiznet5K ethernet device on the SPI bus for use with a UDP
    message protocol. This class is written specifically for this purpose, and cannot
    do much else."""
    def __init__(self, clock: microcontroller.Pin, MOSI: microcontroller.Pin, MISO: microcontroller.Pin, CS: microcontroller.Pin, mac_address: bytes, frequency: int=14000000) -> None:
        """Construct an SPI object on the given pins.
        :param ~microcontroller.Pin clock: the pin to use for the clock.
        :param ~microcontroller.Pin MOSI: the Main Out Selected In pin.
        :param ~microcontroller.Pin MISO: the Main In Selected Out pin.
        :param ~microcontroller.Pin CS: the Chip Select pin.
        :param bytes mac_address: the MAC Address to use for the Wiznet5K device
        :param int frequency: the SPI bus frequency to use for communication
        .. note:: The spi frequency is limited to between 100kHz and 30MHz. 14MHz is the default"""
        ...
    
    def deinit(self) -> None:
        """Deinitializes the Wiznet5K and releases any hardware resources for reuse."""
        ...
    
    def __enter__() -> Wiznet5K:
        """No-op used by Context Managers.
        Provided by context manager helper."""
        ...
    
    def __exit__(self) -> None:
        """Automatically deinitializes the hardware when exiting a context. See
        :ref:`lifetime-and-contextmanagers` for more info."""
        ...
    
    def ifconfig(self, ip: bytes, gateway: bytes, subnet: bytes) -> None:
        """Configures the static IP address of the Wiznet5K device.
        :param bytes ip: The IP Address to use for the device as a 4 byte array
        :param bytes gateway: The IP Address of the gateway to use as a 4 byte array
        :param bytes subnet: The subnet mask of the local network as a 4 byte array"""
        ...

    chip_id: int
    """The chip id of the Wiznet5K Chip"""