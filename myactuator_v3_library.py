import can
from time import sleep

#pip install candle-bus #THIS IS IMPORTANT
#pip install can

bus = can.interface.Bus(bitrate=1000000, bustype='candle', channel = '00000000') #the bustype // channel depends on your device

###ALL COMMANDS###
'''
read_pid_parameter()
write_pid_parameter_to_ram(CurrKP, CurrKI, SpdKP, SpdKI, PosKP, PosKI)
write_pid_parameter_to_rom(CurrKP, CurrKI, SpdKP, SpdKI, PosKP, PosKI)
read_acceleration(function_index)
write_acceleration(function_index, accel)
read_multi_turn_encoder_position()
read_multi_turn_encoder_original_position()
read_multi_turn_encoder_zero_offset()
write_encoder_multi_turn_value_to_rom(encoder_offset)
write_encoder_position_to_rom()
read_single_turn_encoder()
read_multi_turn_angle()
read_single_turn_angle()
read_motor_status_and_error()
read_motor_status_2()
read_motor_status_3()

##MOTION
motor_shutdown()
motor_stop()
torque_closed_loop_control(iq_control)
speed_closed_loop_control(speed_control)
absolute_position_closed_loop_control(max_speed, angle_control)
single_turn_position_control(spin_direction, max_speed, angle_control)
incremental_position_control(max_speed, angle_control)

get_system_operating_mode()
get_motor_power()
system_reset()
system_brake_release()
system_brake_lock()
system_runtime_read()
system_software_version_date_read()
set_communication_interruption_protection_time(can_recv_time_ms)
set_communication_baud_rate(baud_rate, bus_type)
read_motor_model()
activate_reply_command(command, enable_bit, interval_ms)
function_control_command(index, parameter)
multi_motor_command(command, data)
canid_setting_command(read_write_flag, can_id)
motion_mode_control_command(id_number, p_des, v_des, t_ff, kp, kd)


'''

###commands created by chatGPT

def read_pid_parameter(unitID:int):
    # Send the command to read PID parameters
    msg = can.Message(arbitration_id=0x140+unitID, data=[0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
    bus.send(msg)
    
    # Wait for the response message
    message = bus.recv(timeout=1)
    
    if message is None:
        raise TimeoutError(f"No CAN response received from motor {unitID}")

    if message.arbitration_id != 0x240+unitID or message.data[0] != 0x30:
        raise ValueError(f"Unexpected CAN response from motor {unitID}: ID={hex(message.arbitration_id)}, data={message.data.hex()}")

    # Extracting the parameters from the message data
    CurrKP = message.data[2]
    CurrKI = message.data[3]
    SpdKP = message.data[4]
    SpdKI = message.data[5]
    PosKP = message.data[6]
    PosKI = message.data[7]

    # Converting the extracted uint8_t values to the actual parameter values
    CurrKP_value = CurrKP * 3 / 256
    CurrKI_value = CurrKI * 0.1 / 256
    SpdKP_value = SpdKP * 0.1 / 256
    SpdKI_value = SpdKI * 0.01 / 256
    PosKP_value = PosKP * 0.1 / 256
    PosKI_value = PosKI * 0.01 / 256

    # Returning the results in a readable format
    return {
        "Current loop KP": CurrKP_value,
        "Current loop KI": CurrKI_value,
        "Speed loop KP": SpdKP_value,
        "Speed loop KI": SpdKI_value,
        "Position loop KP": PosKP_value,
        "Position loop KI": PosKI_value
    }
def write_pid_parameter_to_ram(unitID:int, CurrKP, CurrKI, SpdKP, SpdKI, PosKP, PosKI):
    # Ensure parameter values are within the allowable range
    CurrKP = min(max(CurrKP, 0), 256)
    CurrKI = min(max(CurrKI, 0), 256)
    SpdKP = min(max(SpdKP, 0), 256)
    SpdKI = min(max(SpdKI, 0), 256)
    PosKP = min(max(PosKP, 0), 256)
    PosKI = min(max(PosKI, 0), 256)

    # Construct the message to write PID parameters to RAM
    data = [0x31, 0x00, CurrKP, CurrKI, SpdKP, SpdKI, PosKP, PosKI]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for acknowledgment (echo) message
    ack_message = bus.recv(timeout=1)

    if ack_message is None:
        return "No acknowledgment received"

    if ack_message.arbitration_id != 0x240+unitID or ack_message.data[0] != 0x31:
        return "Unexpected acknowledgment received"

    # Check if the echoed data matches what was sent
    if ack_message.data[2:] != data[2:]:
        return "Mismatch in echoed data"

    return "PID parameters written to RAM successfully"

def write_pid_parameter_to_rom(unitID:int, CurrKP, CurrKI, SpdKP, SpdKI, PosKP, PosKI):
    # Ensure parameter values are within the allowable range
    CurrKP = min(max(CurrKP, 0), 256)
    CurrKI = min(max(CurrKI, 0), 256)
    SpdKP = min(max(SpdKP, 0), 256)
    SpdKI = min(max(SpdKI, 0), 256)
    PosKP = min(max(PosKP, 0), 256)
    PosKI = min(max(PosKI, 0), 256)

    # Construct the message to write PID parameters to ROM
    data = [0x32, 0x00, CurrKP, CurrKI, SpdKP, SpdKI, PosKP, PosKI]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for acknowledgment (echo) message
    ack_message = bus.recv(timeout=1)

    if ack_message is None:
        return "No acknowledgment received"

    if ack_message.arbitration_id != 0x240+unitID or ack_message.data[0] != 0x32:
        return "Unexpected acknowledgment received"

    # Check if the echoed data matches what was sent
    if ack_message.data[2:] != data[2:]:
        return "Mismatch in echoed data"

    return "PID parameters written to ROM successfully"

def read_acceleration(unitID:int, function_index):
    # Ensure function index is within the allowable range
    function_index = min(max(function_index, 0), 3)

    # Construct the message to read acceleration
    data = [0x42, function_index, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        raise TimeoutError(f"No CAN response from motor {unitID} (read_acceleration)")

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x42:
        raise ValueError(f"Unexpected CAN response from motor {unitID}: ID={hex(response_message.arbitration_id)}, cmd={hex(response_message.data[0])}")

    # Extract the acceleration value from the response data
    accel = response_message.data[4] + (response_message.data[5] << 8) + (response_message.data[6] << 16) + (response_message.data[7] << 24)

    return accel

'''
def read_acceleration(function_index):
    function_names = {
        0x00: "Position Planning Acceleration",
        0x01: "Position Planning Deceleration",
        0x02: "Speed Planning Acceleration",
        0x03: "Speed Planning Deceleration"
    }
    
    if function_index in function_names:
        return f"Function Index {function_index}: {function_names[function_index]}"
    else:
        return "Invalid function index. Please provide a valid function index."

# When user types "read_acceleration("
popup_description = "Available function indexes for read_acceleration():\n"
for index, name in function_names.items():
    popup_description += f"- {hex(index)}: {name}\n"
popup_description = popup_description.strip()

print(popup_description)
'''

def write_acceleration(unitID:int, function_index, accel):
    # Ensure function index is within the allowable range
    function_index = min(max(function_index, 0), 3)

    # Ensure acceleration value is within the allowable range
    accel = max(min(accel, 60000), 100)

    # Construct the message to write acceleration
    data = [0x43, function_index, 0x00, 0x00, accel & 0xFF, (accel >> 8) & 0xFF, (accel >> 16) & 0xFF, (accel >> 24) & 0xFF]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        raise TimeoutError(f"No CAN response from motor {unitID} (write_acceleration)")

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x43:
        raise ValueError(f"Unexpected CAN response from motor {unitID}: ID={hex(response_message.arbitration_id)}, cmd={hex(response_message.data[0])}")

    return "Acceleration written successfully"

def read_multi_turn_encoder_position(unitID:int):
    # Construct the message to read multi-turn encoder position
    data = [0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        raise TimeoutError(f"No CAN response from motor {unitID} (read_multi_turn_encoder_position)")

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x60:
        raise ValueError(f"Unexpected CAN response from motor {unitID}: ID={hex(response_message.arbitration_id)}, cmd={hex(response_message.data[0])}")

    # Decode the encoder position from the response
    encoder_position = response_message.data[4] + \
                       (response_message.data[5] << 8) + \
                       (response_message.data[6] << 16) + \
                       (response_message.data[7] << 24)

    return encoder_position

def read_multi_turn_encoder_original_position(unitID:int):
    # Construct the message to read multi-turn encoder original position
    data = [0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        raise TimeoutError(f"No CAN response from motor {unitID} (read_multi_turn_encoder_original_position)")

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x61:
        raise ValueError(f"Unexpected CAN response from motor {unitID}: ID={hex(response_message.arbitration_id)}, cmd={hex(response_message.data[0])}")

    # Decode the encoder original position from the response
    encoder_original_position = response_message.data[4] + \
                                 (response_message.data[5] << 8) + \
                                 (response_message.data[6] << 16) + \
                                 (response_message.data[7] << 24)

    return encoder_original_position


def read_multi_turn_encoder_zero_offset(unitID:int):
    # Construct the message to read multi-turn encoder zero offset
    data = [0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x62:
        return "Unexpected response received"

    # Decode the encoder zero offset from the response
    encoder_zero_offset = response_message.data[4] + \
                           (response_message.data[5] << 8) + \
                           (response_message.data[6] << 16) + \
                           (response_message.data[7] << 24)

    return encoder_zero_offset

def write_encoder_multi_turn_value_to_rom(unitID:int, encoder_offset):
    # Construct the message to write encoder multi-turn value to ROM
    data = [0x63, 0x00, 0x00, 0x00,
            encoder_offset & 0xFF,
            (encoder_offset >> 8) & 0xFF,
            (encoder_offset >> 16) & 0xFF,
            (encoder_offset >> 24) & 0xFF]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x63:
        return "Unexpected response received"

    return "Encoder multi-turn value written to ROM as motor zero"

def write_encoder_position_to_rom(unitID:int):
    # Construct the message to write encoder position to ROM
    data = [0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x64:
        return "Unexpected response received"

    # Extract the encoder offset value from the response message
    encoder_offset = (response_message.data[7] << 24) | (response_message.data[6] << 16) | \
                     (response_message.data[5] << 8) | response_message.data[4]

    return f"Encoder position written to ROM as motor zero with offset: {encoder_offset}"

def read_single_turn_encoder(unitID:int):
    # Construct the message to read single-turn encoder data
    data = [0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x90:
        return "Unexpected response received"

    # Extract encoder position, encoder original position, and encoder zero offset from the response message
    encoder_position = (response_message.data[3] << 8) | response_message.data[2]
    encoder_original_position = (response_message.data[5] << 8) | response_message.data[4]
    encoder_zero_offset = (response_message.data[7] << 8) | response_message.data[6]

    return f"Encoder Position: {encoder_position}, Encoder Original Position: {encoder_original_position}, Encoder Zero Offset: {encoder_zero_offset}"

def read_multi_turn_angle(unitID:int):
    # Construct the message to read multi-turn angle data
    data = [0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x92:
        return "Unexpected response received"

    # Extract motor angle from the response message
    motor_angle = (response_message.data[7] << 24) | (response_message.data[6] << 16) | (response_message.data[5] << 8) | response_message.data[4]

    # Convert the angle to degrees
    motor_angle_degrees = motor_angle * 0.01

    return f"Motor Angle: {motor_angle_degrees} degrees"
def read_single_turn_angle(unitID:int):
    # Construct the message to read single-turn angle data
    data = [0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x94:
        return "Unexpected response received"

    # Extract single-turn angle from the response message
    single_turn_angle = (response_message.data[7] << 8) | response_message.data[6]

    # Convert the angle to degrees
    single_turn_angle_degrees = single_turn_angle * 0.01

    return f"Single-turn Angle: {single_turn_angle_degrees} degrees"

def read_motor_status_and_error(unitID:int):
    # Construct the message to read motor status and error
    data = [0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x9A:
        return "Unexpected response received"

    # Extract motor temperature from the response message
    motor_temperature = response_message.data[1]

    # Extract brake control command from the response message
    brake_control_command = response_message.data[3]

    # Extract voltage from the response message
    voltage = (response_message.data[5] << 8) | response_message.data[4]
    voltage = voltage * 0.1

    # Extract error flags from the response message
    error_state = (response_message.data[7] << 8) | response_message.data[6]

    # Decode error flags based on the provided table
    error_flags = []
    if error_state & 0x0002:
        error_flags.append("Motor stall")
    if error_state & 0x0004:
        error_flags.append("Low pressure")
    if error_state & 0x0008:
        error_flags.append("Overvoltage")
    if error_state & 0x0010:
        error_flags.append("Overcurrent")
    if error_state & 0x0040:
        error_flags.append("Power overrun")
    if error_state & 0x0080:
        error_flags.append("Calibration parameter writing error")
    if error_state & 0x0100:
        error_flags.append("Speeding")
    if error_state & 0x1000:
        error_flags.append("Motor temperature over temperature")
    if error_state & 0x2000:
        error_flags.append("Encoder calibration error")

    return {
        "Motor Temperature": motor_temperature,
        "Brake Control Command": "Release" if brake_control_command == 1 else "Lock",
        "Voltage": voltage,
        "Error Flags": error_flags
    }

def read_motor_status_2(unitID:int):
    # Construct the message to read motor status 2
    data = [0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        raise TimeoutError(f"No CAN response from motor {unitID} (read_motor_status_2)")
    
    if response_message.data is None:
        raise TimeoutError(f"No CAN response data from motor {unitID} (read_motor_status_2)")

    if len(response_message.data) < 8:
        raise TimeoutError(f"No CAN response data length from motor {unitID} (read_motor_status_2)")

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x9C:
        raise ValueError(f"Unexpected CAN response from motor {unitID}: ID={hex(response_message.arbitration_id)}, cmd={hex(response_message.data[0])}")

    # Extract motor temperature from the response message
    motor_temperature = response_message.data[1]

    # Extract torque current value (iq) from the response message
    iq = (response_message.data[3] << 8) | response_message.data[2]

    # Extract motor speed from the response message
    speed = (response_message.data[5] << 8) | response_message.data[4]

    # Extract motor angle from the response message
    degree = (response_message.data[7] << 8) | response_message.data[6]

    return {
        "Motor Temperature": motor_temperature,
        "Torque Current (A)": iq * 0.01,
        "Motor Speed (dps)": speed,
        "Motor Angle (degrees)": degree
    }

def read_motor_status_3(unitID:int):
    # Construct the message to read motor status 3
    data = [0x9D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        raise TimeoutError(f"No CAN response from motor {unitID} (read_motor_status_3)")

    if response_message.arbitration_id != 0x240+unitID or response_message.data[0] != 0x9D:
        raise ValueError(f"Unexpected CAN response from motor {unitID}: ID={hex(response_message.arbitration_id)}, cmd={hex(response_message.data[0])}")

    # Extract motor temperature from the response message
    motor_temperature = response_message.data[1]

    # Extract phase A current data from the response message
    iA = (response_message.data[3] << 8) | response_message.data[2]

    # Extract phase B current data from the response message
    iB = (response_message.data[5] << 8) | response_message.data[4]

    # Extract phase C current data from the response message
    iC = (response_message.data[7] << 8) | response_message.data[6]

    return {
        "Motor Temperature": motor_temperature,
        "Phase A Current (A)": iA * 0.01,
        "Phase B Current (A)": iB * 0.01,
        "Phase C Current (A)": iC * 0.01
    }

def motor_shutdown(unitID:int):
    # Construct the message to send motor shutdown command
    data = [0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    # Check if the response is as expected
    expected_response = [0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    if response_message.data != expected_response:
        return "Unexpected response received"

    return "Motor shutdown command successful"

def motor_stop(unitID:int):
    # Construct the message to send motor stop command
    data = [0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    # Check if the response is as expected
    expected_response = [0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    response_message = [byte for byte in response_message.data]
    if response_message != expected_response:
        return "Unexpected response received"

    return "Motor stop command successful"

def torque_closed_loop_control(unitID:int, iq_control):
    # Construct the message to send torque closed-loop control command
    data = [0xA1, 0x00, 0x00, 0x00, iq_control & 0xFF, (iq_control >> 8) & 0xFF, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    # Parse the response message
    temperature = response_message.data[1]
    iq = response_message.data[2] + (response_message.data[3] << 8)
    speed = response_message.data[4] + (response_message.data[5] << 8)
    angle = response_message.data[6] + (response_message.data[7] << 8)

    return {
        "temperature": temperature,
        "torque_current": iq,
        "motor_speed": speed,
        "motor_angle": angle
    }

def speed_closed_loop_control(unitID:int, speed_control):
    # Construct the message to send speed closed-loop control command
    data = [
        0xA2,
        0x00, 0x00, 0x00,  # NULL
        speed_control & 0xFF,
        (speed_control >> 8) & 0xFF,
        (speed_control >> 16) & 0xFF,
        (speed_control >> 24) & 0xFF
    ]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    # Parse the response message
    temperature = response_message.data[1]
    iq = response_message.data[2] + (response_message.data[3] << 8)
    speed = response_message.data[4] + (response_message.data[5] << 8)
    angle = response_message.data[6] + (response_message.data[7] << 8)

    return {
        "temperature": temperature,
        "torque_current": iq,
        "motor_speed": speed,
        "motor_angle": angle
    }

def absolute_position_closed_loop_control(unitID:int, max_speed, angle_control):
    """
    Function to perform absolute position closed-loop control.

    Args:
        max_speed (int): Maximum speed for closed-loop control.
                         Valid range: Any integer value.
        angle_control (int): Angle for closed-loop control.
                             Valid range: Any integer value.
    
    Returns:
        dict: Dictionary containing the response data.
              - temperature: Temperature data.
              - torque_current: Torque current data.
              - motor_speed: Motor speed data.
              - motor_angle: Motor angle data.
    """
    
    # Construct the message to send absolute position closed-loop control command
    data = [
        0xA4,
        0x00,  # NULL
        max_speed & 0xFF,
        (max_speed >> 8) & 0xFF,
        angle_control & 0xFF,
        (angle_control >> 8) & 0xFF,
        (angle_control >> 16) & 0xFF,
        (angle_control >> 24) & 0xFF
    ]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    # Parse the response message
    temperature = response_message.data[1]
    iq = response_message.data[2] + (response_message.data[3] << 8)
    speed = response_message.data[4] + (response_message.data[5] << 8)
    angle = response_message.data[6] + (response_message.data[7] << 8)

    return {
        "temperature": temperature,
        "torque_current": iq,
        "motor_speed": speed,
        "motor_angle": angle
    }

def single_turn_position_control(unitID:int, spin_direction, max_speed, angle_control):
    # Construct the message to send single-turn position control command
    data = [
        0xA6,
        spin_direction,
        max_speed & 0xFF,
        (max_speed >> 8) & 0xFF,
        angle_control & 0xFF,
        (angle_control >> 8) & 0xFF,
        0x00,  # NULL
        0x00   # NULL
    ]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    # Parse the response message
    temperature = response_message.data[1]
    iq = response_message.data[2] + (response_message.data[3] << 8)
    speed = response_message.data[4] + (response_message.data[5] << 8)
    encoder = response_message.data[6] + (response_message.data[7] << 8)

    return {
        "temperature": temperature,
        "torque_current": iq,
        "motor_speed": speed,
        "encoder_value": encoder
    }

def incremental_position_control(unitID:int, max_speed, angle_control):
    # Construct the message to send incremental position control command
    data = [
        0xA8,
        0x00,  # NULL
        max_speed & 0xFF,
        (max_speed >> 8) & 0xFF,
        angle_control & 0xFF,
        (angle_control >> 8) & 0xFF,
        (angle_control >> 16) & 0xFF,
        (angle_control >> 24) & 0xFF
    ]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    # Parse the response message
    temperature = response_message.data[1]
    iq = response_message.data[2] + (response_message.data[3] << 8)
    speed = response_message.data[4] + (response_message.data[5] << 8)
    angle = response_message.data[6] + (response_message.data[7] << 8)

    return {
        "temperature": temperature,
        "torque_current": iq,
        "motor_speed": speed,
        "motor_angle": angle
    }

def get_system_operating_mode(unitID:int):
    # Construct the message to send system operating mode acquisition command
    data = [0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    # Parse the response message
    runmode = response_message.data[7]

    # Interpret the run mode
    mode = ""
    if runmode == 0x01:
        mode = "Current loop mode"
    elif runmode == 0x02:
        mode = "Speed loop mode"
    elif runmode == 0x03:
        mode = "Position loop mode"

    return mode

def get_motor_power(unitID:int):
    # Construct the message to send motor power acquisition command
    data = [0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the response message
    response_message = bus.recv(timeout=1)

    if response_message is None:
        return "No response received"

    # Parse the response message
    motorpower = response_message.data[6] + (response_message.data[7] << 8)

    # Convert motor power to watts
    motor_power_watts = motorpower * 0.1

    return motor_power_watts

def system_reset(unitID:int):
    # Construct the message to send system reset command
    data = [0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

def system_brake_release(unitID:int):
    # Construct the message to release the system brake
    data = [0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

def system_brake_lock(unitID:int):
    # Construct the message to lock the system brake
    data = [0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

def system_runtime_read(unitID:int):
    # Construct the message to read system runtime
    data = [0xB1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the reply
    reply = bus.recv(timeout=1.0)
    if reply is not None:
        # Extract the system runtime from the reply
        sys_runtime = (reply.data[4] << 0) | (reply.data[5] << 8) | (reply.data[6] << 16) | (reply.data[7] << 24)
        print("System runtime:", sys_runtime, "ms")
    else:
        print("No reply received.")

def system_software_version_date_read(unitID:int):
    # Construct the message to read system software version date
    data = [0xB2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the reply
    reply = bus.recv(timeout=1.0)
    if reply is not None:
        # Extract the version date from the reply
        version_date = (reply.data[4] << 0) | (reply.data[5] << 8) | (reply.data[6] << 16) | (reply.data[7] << 24)
        print("System software version date:", version_date)
    else:
        print("No reply received.")

def set_communication_interruption_protection_time(unitID:int, can_recv_time_ms):
    # Convert time to bytes
    time_bytes = [
        (can_recv_time_ms >> 0) & 0xFF,
        (can_recv_time_ms >> 8) & 0xFF,
        (can_recv_time_ms >> 16) & 0xFF,
        (can_recv_time_ms >> 24) & 0xFF
    ]
    
    # Construct the message to set communication interruption protection time
    data = [0xB3, 0x00, 0x00, 0x00] + time_bytes
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)
    bus.send(msg)

    # Wait for the reply
    reply = bus.recv(timeout=1.0)
    if reply is not None:
        print("Communication interruption protection time set successfully.")
    else:
        print("No reply received.")

def set_communication_baud_rate(unitID:int, baud_rate, bus_type):
    # Check if the bus type is valid
    if bus_type.lower() not in ['can', 'rs485']:
        print("Invalid bus type. Please specify either 'CAN' or 'RS485'.")
        return
    
    # Convert baud rate to bytes
    if bus_type.lower() == 'rs485':
        baud_rate_bytes = baud_rate
    else:
        if baud_rate == 500000:
            baud_rate_bytes = 0
        elif baud_rate == 1000000:
            baud_rate_bytes = 1
        else:
            print("Invalid baud rate for CAN. Supported baud rates are 500Kbps and 1Mbps.")
            return

    # Construct the message to set communication baud rate
    data = [0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, baud_rate_bytes]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)

    # Send the command
    bus.send(msg)

    print("Communication baud rate set successfully.")

def read_motor_model(unitID:int):
    # Construct the message to read motor model
    data = [0xB5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)

    # Send the command
    bus.send(msg)

    # Receive the reply
    reply = bus.recv()
    
    # Extract motor model from reply
    motor_model = ""
    for i in range(1, 8):
        motor_model += chr(reply.data[i])

    print("Motor Model:", motor_model)

def activate_reply_command(unitID:int, command, enable_bit, interval_ms):
    # Ensure CAN version
    if not is_can_version():
        print("Error: 485 version does not support this function.")
        return

    # Construct the message to activate reply command
    data = [0xB6, command, enable_bit, interval_ms & 0xFF, (interval_ms >> 8) & 0xFF, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)

    # Send the command
    bus.send(msg)

    print(f"Active reply command {hex(command)} enabled with interval {interval_ms}ms.")

def function_control_command(unitID:int, index, parameter):
    # Construct the message for function control command
    data = [0x20, index, 0x00, 0x00, parameter & 0xFF, (parameter >> 8) & 0xFF, (parameter >> 16) & 0xFF, (parameter >> 24) & 0xFF]
    msg = can.Message(arbitration_id=0x140+unitID, data=data, is_extended_id=False)

    # Send the command
    bus.send(msg)

    print(f"Function control command {hex(index)} with parameter {parameter} sent.")

def multi_motor_command(command, data):
    # Construct the message for multi-motor command
    message_data = [0x280 + command] + data
    msg = can.Message(arbitration_id=0x280, data=message_data, is_extended_id=False)

    # Send the command
    bus.send(msg)

    print(f"Multi-motor command 0x{280+command:X} with data {data} sent.")

def canid_setting_command(read_write_flag, can_id):
    # Construct the message for CANID setting command
    message_data = [0x79, 0x00, read_write_flag, 0x00, 0x00, 0x00, can_id & 0xFF, (can_id >> 8) & 0xFF]
    msg = can.Message(arbitration_id=0x300, data=message_data, is_extended_id=False)

    # Send the command
    bus.send(msg)

    print(f"CANID setting command sent: Read/Write flag={read_write_flag}, CAN ID={can_id}.")

def motion_mode_control_command(id_number, p_des, v_des, t_ff, kp, kd):
    # Calculate parameter values
    p_des_bytes = [(p_des >> 8) & 0xFF, p_des & 0xFF]
    v_des_bytes = [(v_des >> 4) & 0xFF, (v_des & 0xF) << 4]
    kp_bytes = [(kp >> 8) & 0xFF, kp & 0xFF]
    kd_bytes = [(kd >> 4) & 0xFF, (kd & 0xF) << 4]
    t_ff_bytes = [(t_ff >> 4) & 0xFF, (t_ff & 0xF) << 4]

    # Construct the message for motion mode control command
    message_data = [p_des_bytes[0], p_des_bytes[1], v_des_bytes[0], v_des_bytes[1],
                    kp_bytes[0], kp_bytes[1], kd_bytes[0], kd_bytes[1],
                    t_ff_bytes[0], t_ff_bytes[1]]

    msg = can.Message(arbitration_id=0x400 + id_number, data=message_data, is_extended_id=False)

    # Send the command
    bus.send(msg)

    print(f"Motion Mode Control Command sent for ID {id_number}.")




