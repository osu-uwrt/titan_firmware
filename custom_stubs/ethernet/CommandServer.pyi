from . import Wiznet5K

class CommandServer:
    """Wiznet5K Command Server Interface
    Initializes a UDP socket for receiving commands from the network."""
    def __init__(self, device: Wiznet5K, port: int) -> None:
        """Construct an SPI object on the given pins.
        :param ~Wiznet5K device: the device to host the port on
        :param int port: the port to host the udp command server on"""
        ...
    
    def deinit(self) -> None:
        """Deinitializes the CommandServer and releases any hardware resources for reuse."""
        ...
    
    def __enter__() -> CommandServer:
        """No-op used by Context Managers.
        Provided by context manager helper."""
        ...
    
    def __exit__(self) -> None:
        """Automatically deinitializes the hardware when exiting a context. See
        :ref:`lifetime-and-contextmanagers` for more info."""
        ...
    
    def next_command(self) -> tuple[int, bytes, bytes]:
        """Gets the next command from the command server
        :returns: Tuple containing either three None elements if there is no next command, or
        a tuple of the command id, the data bytes, and the packet data bytes to be sent with the reply"""
        ...
    
    def reply(self, response_data: bytes, packet_data: bytes) -> None:
        """Sends a reply to a received command with the specified data
        :param bytes response_data: The data to respond with
        :param bytes packet_data: The packet data from the incoming command packet"""
        ...

    open: bool
    """If the given socket is open on the network device.
    This should normally return true, unless deinited, but can return false
    if the network device is deinited."""

    rx_buffer_count: int
    """The number of bytes in the rx buffer of the UDP socket"""

    rx_buffer_size: int
    """The size of the rx buffer for the UDP socket"""