from enum import Enum
SERVER_OPERATION_CODE_LENGTH = 4

class ServerMessage(Enum):
    GET_POSE = 'GETP'
    RET_POSE = 'RETP'
    GET_VIZ = 'GETV'
    RET_VIZ = 'RETV'
    SET_INTR = 'INTR'
    GET_SCORE = 'GSCO'
    RET_SCORE = 'RSCO'
    SET_SO3_GRID_SIZE = 'SO3G'
    ERR = 'RERR'
    OK = 'OKOK'