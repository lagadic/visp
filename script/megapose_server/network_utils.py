import io
import struct
from typing import Callable, List, Tuple
import numpy as np
import torch
from math import prod
import socket

from megapose_server.server_operations import ServerMessage, SERVER_OPERATION_CODE_LENGTH

def read_string(buffer: io.BytesIO):
    '''
    Read a string from a buffer.
    From the buffer, read an int describing the length of the string, then read the characters (ascii)
    '''
    str_count = struct.unpack('>I', buffer.read(struct.calcsize('I')))[0]
    data = struct.unpack(f'{str_count}s', buffer.read(str_count))[0]
    return data.decode('ascii')
def pack_string(s: str, buffer: bytearray):
    '''
    Pack a string into a buffer
    an int being the length of the string and the characters are appended to the buffer
    '''
    buffer.extend(struct.pack(f'>I{len(s)}s', len(s), s.encode('ascii')))

def read_image(buffer: io.BytesIO):
    '''
    Read an image (an array of uint8 values)  from a buffer.

    First, 3 ints are read (height, width and channels) then height * width * channels bytes
    The elements are consumed from the buffer.

    If the image has an alpha channel, it is discarded

    returns the image as an np.array
    '''
    image_shape = struct.unpack('>3I', buffer.read(struct.calcsize('>3I')))
    elem_count = prod(image_shape)
    img_bytes = buffer.read(elem_count)
    # img = torch.frombuffer(img_bytes, dtype=torch.uint8, count=elem_count).view(image_shape)
    img = np.frombuffer(img_bytes, dtype=np.uint8, count=elem_count).reshape(image_shape)
    if image_shape[-1] == 4: # Image is of type RGBA, discard alpha
        img = img[:, :, :3]

    return img

def read_uint16_image(buffer: io.BytesIO):
    '''
    Read an uint16 image from a buffer
    First, 2 ints are read (height, width) then the endianness symbol and then height * width * 2 bytes.

    The elements are consumed from the buffer.

    returns the image as an np.array
    '''
    image_shape = struct.unpack('>2I', buffer.read(struct.calcsize('>2I')))
    endianness = struct.unpack('c', buffer.read(struct.calcsize('c')))[0].decode('ascii')
    assert endianness in ['>', '<']
    elem_count = prod(image_shape)
    img_bytes = buffer.read(elem_count * 2)
    dt = np.dtype(np.uint16)
    dt = dt.newbyteorder(endianness)
    img = np.frombuffer(img_bytes, dtype=dt, count=elem_count).reshape(image_shape)
    return img

def pack_image(image, buffer: bytearray):
    '''
    Pack an image into a buffer
    '''
    image = image.astype(np.uint8)
    assert len(image.shape) == 3
    buffer.extend(struct.pack('>3I', *image.shape))
    buffer.extend(image.tobytes('C'))

def create_message(message_code: ServerMessage, fn: Callable[[bytearray], None]):
    '''
    Create a message to be sent on the network

    A message has the shape

    MSG_LENGTH | MSG_CODE | DATA
    where MSG_LENGTH is the length of DATA (in bytes), and MSG_CODE is the operation to be performed
    '''
    data = bytearray()
    temp_length = struct.pack('>I', 0)
    data.extend(temp_length)
    data.extend(struct.pack(f'{SERVER_OPERATION_CODE_LENGTH}s', message_code.value.encode('UTF-8')))

    header_length = struct.calcsize(f'>I{SERVER_OPERATION_CODE_LENGTH}s')
    fn(data)
    data[0:struct.calcsize('>I')] = struct.pack('>I', len(data) - header_length)
    return data

def receive_message(s: socket.socket) -> Tuple[str, io.BytesIO]:
    '''
    Read a socket message
    A message has the shape

    MSG_LENGTH | MSG_CODE | DATA

    returns the code as an str value associated to the ServerOperation enum, as well as DATA, as an io.ByteIO

    '''
    msg_length = s.recv(4)
    length = struct.unpack('>I', msg_length)[0]
    code = s.recv(SERVER_OPERATION_CODE_LENGTH).decode('UTF-8')
    # data = bytearray(length)
    data = bytearray()
    iters = 0
    read_count = 0
    while read_count < length:
        packet = s.recv(length - read_count)
        if not packet:
            return None
        data.extend(packet)
        # data[read_count:read_count + len(packet)] = packet
        read_count += len(packet)

    return code, io.BytesIO(data)
