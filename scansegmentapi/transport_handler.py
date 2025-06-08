#
# Copyright (c) 2024 SICK AG
# SPDX-License-Identifier: MIT
#

from abc import ABCMeta, abstractmethod


class TransportHandler(metaclass=ABCMeta):
    """This is an abstract base class for all transport handlers.
    This class provides the error handling and the interface for receiving new scan segments.
    """
    def __init__(self):
        self.no_error_flag = False
        self.last_error_code = None
        self.last_error_message = ""
        self.counter = 0

    @abstractmethod
    def receive_new_scan_segment(self) -> tuple[bytes, str]:
        """Waits on a new scan segment.

        In the return value the sender ip is returned as well because for some transport protocols
        this might be needed to identify the sender.

        Returns:
            tuple[bytes, str]: Tuple of the received data and the ip address of the sender
        """
        pass

    def has_no_error(self) -> bool:
        """Check whether the TransportHandler is in an error state

        Returns:
            boolean: True if there was no error, false otherwise
        """
        return self.no_error_flag

    def get_data_counter(self) -> int:
        """Get the number of received scan segments

        Returns:
            int: The number of received scan segments
        """
        return self.counter

    def get_last_error_code(self) -> int:
        """Get the last error code.

        Returns:
            int: The error code of the error that last occurred
        """
        return self.last_error_code
