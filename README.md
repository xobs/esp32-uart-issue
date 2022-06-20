# ESP32 Uart / Wifi Breakage Demo

This repository demonstrates an issue that was discovered where the ESP32
stops responding to wifi traffic when it gets a lot of data via the UART,
even if it's not actually doing anything with that UART data.

## Usage

1. Modify `main/config.h` and update the following variables:
   
   - `CONFIG_ESP_WIFI_SSID` and `CONFIG_ESP_WIFI_PASSWORD`
   - `CONFIG_UART_TX_GPIO` and `CONFIG_UART_RX_GPIO`
   - `CONFIG_UART_BAUD`
   - `CONFIG_LED_GPIO`, or remove the definition to disable LED support
3. Monitor UART0 to determine what the device's IP address is.
4. Begin pinging the IP address (`ping -t DEVICE_ADDRESS`)
5. Transfer data to the device via serial port
6. Observe that the network traffic, and that the ping has stopped
7. Stop transferring data
8. Observe that the network traffic has resumed

## Example Output

For a source text, Project Gutenberg can be relied upon for plain text files.
Pride and Prejudice is their most popular book, and is about 780 kB. It
may be freely downloaded from https://www.gutenberg.org/files/1342/1342-0.txt

An example of the network traffic observed when sending this file to an ESP32
device may be seen below:

```
Reply from 10.0.237.105: bytes=32 time=1ms TTL=255
Reply from 10.0.237.105: bytes=32 time=1ms TTL=255
Request timed out.
Request timed out.
Request timed out.
Request timed out.
Reply from 10.0.237.105: bytes=32 time=15ms TTL=255
Reply from 10.0.237.105: bytes=32 time=1ms TTL=255
```

Four packets were missed due to the fact that the 780 kB file takes about 18
seconds to transfer, and Windows pings time out after five seconds.

## About the LED

The example uses an LED to determine if the device is in "bulk mode" or "byte mode". In
normal, interactive environments, the device will be in "byte" mode and will return
immediately if data is received. This presents a responsive environment.

In "bulk mode", the device attempts to read 126 bytes at a time. This mode limits the
number of interrupts that the device will receive.

The LED reflects which mode the device is in.

## TFTP Updates

This device presents a TFTP server. You can install the `tftp-hpa` package and
update this device via TFTP:

```
$ tftp -v -m octet 10.0.237.105 -c put build/uart_test.bin firmware.bin
Connected to 10.0.237.105 (10.0.237.105), port 69
putting build/uart_test.bin to 10.0.237.105:firmware.bin [octet]
Sent 656384 bytes in 13.5 seconds [389314 bit/s]
$
```