#!/usr/bin/env python3

import configparser
import logging
import socket
import serial
import threading
import time
import signal
import sys

running = True
need_gps = True

# Конфигурация
LOG_FILE = 'udp_log.txt'

config = configparser.ConfigParser()
config.read("config.ini")

HOST_IP = str(config["Settings"]["host_ip"])
TCP_PORT = int(config["Settings"]["tcp_port"])
UDP_PORT = int(config["Settings"]["udp_port"])
STATION_ADDRESS = int(config["Settings"]["station_address"])
NUMBER_OF_BIG_GEAR = int(config["Settings"]["number_of_big_gear"])
NUMBER_OF_LITTLE_GEAR = int(config["Settings"]["number_of_little_gear"])
MAX_COORDINATE = int(((NUMBER_OF_BIG_GEAR / NUMBER_OF_LITTLE_GEAR) * 16384) // 1)

nmea_now = ""

uart_send_buffer = [0, 0, 0, 0, 0]
need_send = False
uart_receive_buffer = [0, 0, 0, 0, 0]
data_received = False

abstract_coordinate = 0

logging.basicConfig(filename=LOG_FILE, level=logging.INFO, format='%(asctime)s - %(message)s')

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_client_addr = None

tilt_answer = 0


def setup_serial():
    try:
        ser = serial.Serial(
            port='/dev/serial0',
            baudrate=115200,
            timeout=1
        )
        if not ser.is_open:
            ser.open()
        return ser
    except serial.SerialException as e:
        print(f"Error port opening: {e}")
        return None


def setup_gps_serial():
    try:
        ser_gps = serial.Serial(
            port='/dev/serial1',
            baudrate=9600,
            timeout=1
        )
        if not ser_gps.is_open:
            ser_gps.open()
        return ser_gps
    except serial.SerialException as e:
        print(f"Error port opening: {e}")
        return None


def uart_gps_receiver():
    global nmea_now
    ser_gps = setup_gps_serial()
    if ser_gps is None:
        return

    log_file = open("uart_log.txt", "a")

    try:
        while need_gps:
            try:
                if ser_gps.in_waiting > 0:
                    data = ser_gps.readline().decode('latin1').strip()
                    nmea_now = data
                    log_entry = f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {data}\n"
                    log_file.write(log_entry)
            except Exception as e:
                print(f"Error of input/output: {e}")
                ser_gps.close()
                ser_gps = setup_gps_serial()
                if ser_gps is None:
                    break
    finally:
        if ser_gps:
            ser_gps.close()
        log_file.close()


# Функция для приема данных по UART
def uart_sender_receiver():
    global uart_send_buffer, uart_receive_buffer, abstract_coordinate, need_send, data_received
    ser = setup_serial()
    if ser is None:
        return

    try:
        while running:
            try:
                if need_send:
                    ser.write(bytearray(uart_send_buffer))
                    need_send = False
                if ser.in_waiting > 4:
                    data = ser.read(5)
                    if len(data) == 5 and data[0] == 0xFF and data[1] == 0x24 and data[4] == (
                            data[1] + data[2] + data[3]) & 0xFF:
                        # print("uart reading", list(data)) # для тестов
                        abstract_coordinate = data[2] << 8 | data[3]
                        data_received = True
                    else:
                        print("incorrect uart data: ", list(data))
            except OSError as e:
                print(f"Error of input/output: {e}")
                ser.close()
                ser = setup_serial()
                if ser is None:
                    break
    finally:
        if ser:
            ser.close()


# Функция для отправки данных по TCP
def tcp_sender():
    global nmea_now
    while need_gps:
        tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            tcp_sock.bind((HOST_IP, TCP_PORT))
            tcp_sock.listen()
            conn, addr = tcp_sock.accept()
            buffer = ""
            while need_gps:
                try:
                    if nmea_now.startswith('$') and nmea_now != buffer:
                        conn.sendall(nmea_now.encode('latin1'))
                        buffer = nmea_now
                except BrokenPipeError:
                    print("TCP client is disconnected, waiting...")
                    break
                except Exception as e:
                    print(f"TCP Sender Error: {e}")
                    break
        finally:
            tcp_sock.close()


# Функция для приема данных по UDP
def udp_receiver():
    global udp_sock, udp_client_addr, need_gps

    udp_sock.bind((HOST_IP, UDP_PORT))
    while running:
        try:
            data, addr = udp_sock.recvfrom(1024)
            if data != 0:
                if need_gps:
                    need_gps = False
                # print(data)     # для  тестов
                udp_client_addr = addr
                command = check_command(data)
                if command:
                    logging.info(f"{time.strftime('%Y-%m-%d %H:%M:%S')} Received command from {addr}: {command}")
        except Exception as e:
            print(f"UDP Receiver Error: {e}")
    udp_sock.close()


# Функция для отправки данных по UDP
def udp_sender(command, data):
    global udp_sock, udp_client_addr

    if not udp_client_addr:
        print("Client is not connected")
        return False

    data_to_send = bytearray(7)
    data_to_send[0] = 0xFF
    data_to_send[1] = STATION_ADDRESS
    data_to_send[2] = 0x00

    if command == 0x71:
        data_to_send[3] = 0x7c
        data_to_send[4] = 0x00
        data_to_send[5] = 0x00
    else:
        data_to_send[3] = command + 16
        data_to_send[4] = (data >> 8) & 0xFF
        data_to_send[5] = data & 0xFF

    data_to_send[6] = (data_to_send[1] + data_to_send[2] + data_to_send[3] + data_to_send[4] + data_to_send[5]) & 0xFF

    try:
        udp_sock.sendto(data_to_send, udp_client_addr)
        # print(data_to_send)    # для  тестов
        return True
    except BrokenPipeError:
        print("UDP client is disconnected, waiting...")
        return False
    except Exception as e:
        print(f"UDP Sender Error: {e}")
        return False


# Парсер команд
def check_command(data):
    if len(data) != 7 or data[0] != 0xFF:
        return None

    address = data[1]
    command1 = data[2]
    command2 = data[3]
    data1 = data[4]
    data2 = data[5]
    checksum = data[6]

    if checksum != (address + command1 + command2 + data1 + data2) & 0xFF:
        return None

    command_dict = {
        'address': address,
        'command1': command1,
        'command2': command2,
        'data1': data1,
        'data2': data2,
        'checksum': checksum
    }

    if command2 & 0b1 == 1:
        parse_pelco_de_command(command_dict)
    else:
        parse_pelco_d_command(command_dict)

    return command_dict


def parse_pelco_d_command(command_dict):
    if command_dict["address"] == STATION_ADDRESS:
        if command_dict["command1"] == 0 or command_dict["command1"] == 1:
            if command_dict["command2"] & 0b10 == 0b10:
                x = command_dict["data1"] << 8 | 0x01
                uart_send_request(0x07, x)
            elif command_dict["command2"] & 0b100 == 0b100:
                x = command_dict["data1"] << 8 | 0x02
                uart_send_request(0x07, x)
            elif command_dict["command2"] == 0:
                x = command_dict["data1"] << 8 | 0x00
                uart_send_request(0x07, x)
            else:
                print("Unknown command (2)")
        else:
            print("Unknown command (1)")
    else:
        print("Not correct address")


def parse_pelco_de_command(command_dict):
    global abstract_coordinate, data_received, tilt_answer
    if command_dict["address"] == STATION_ADDRESS:
        if command_dict["command1"] == 0:
            if command_dict["command2"] == 0x71:
                need_coord = command_dict["data1"] << 8 | command_dict["data2"]
                uart_send_request(0x12, need_coord)
                udp_sender(0x71, 0)
                print("set pan to ", need_coord)
            elif command_dict["command2"] == 0x51 or command_dict["command2"] == 0x79:
                uart_send_request(0x14, 0)
                while not data_received:
                    pass
                data_received = False
                udp_sender(0x51, abstract_coordinate)
                print("send pan coordinate", abstract_coordinate)
            elif command_dict["command2"] == 0x55:
                udp_sender(0x55, MAX_COORDINATE)
            elif command_dict["command2"] == 0x43:
                print("step right")
                uart_send_request(0x12, abstract_coordinate + 50)
            elif command_dict["command2"] == 0x41:
                print("step left")
                uart_send_request(0x12, abstract_coordinate - 50)
            elif command_dict["command2"] == 0x57:
                udp_sender(0x57, 1)
            elif command_dict["command2"] == 0x53:
                udp_sender(0x53, tilt_answer)
            elif command_dict["command2"] == 0x73:
                if command_dict["data2"] == 0x00:
                    tilt_answer = 0
                    udp_sender(0x71, 0)
                elif command_dict["data2"] == 0x01:
                    tilt_answer = 1
                    udp_sender(0x71, 0)
                else:
                    print("Incorrect tilt data")
            else:
                print("Unknown command (2)")
        else:
            print("Unknown command (1)")
    else:
        print("Not correct address")


def uart_send_request(command, x):
    global need_send
    while need_send:
        pass
    uart_send_buffer[0] = 0xFF
    uart_send_buffer[1] = command
    uart_send_buffer[2] = x >> 8
    uart_send_buffer[3] = x & 0xFF
    uart_send_buffer[4] = (uart_send_buffer[1] + uart_send_buffer[2] + uart_send_buffer[3]) & 0xFF
    need_send = True


# Обработчик сигнала для завершения потоков
def signal_handler(sig, frame):
    global running
    print()
    print('The program was been stopped by user, stopping...')
    running = False
    sys.exit(0)


if __name__ == "__main__":
    print("start")

    signal.signal(signal.SIGINT, signal_handler)

    uart_thread = threading.Thread(target=uart_sender_receiver, daemon=True)
    udp_thread = threading.Thread(target=udp_receiver, daemon=True)
    uart_gps_thread = threading.Thread(target=uart_gps_receiver, daemon=True)

    udp_thread.start()
    uart_thread.start()
    uart_gps_thread.start()

    udp_thread.join()
    uart_thread.join()
    uart_gps_thread.join()

    while need_gps:
        tcp_sender()
