import ctypes as ct

esmini_home = "/opt/esmini/"

rm = ct.CDLL(esmini_home + "bin/libesminiRMLib.so")  # Linux


"""
typedef struct
{
    float x;
    float y;
    float z;
    float h;
    float p;
    float r;
    float hRelative;
    id_t  roadId;
    id_t  junctionId;  // -1 if not in a junction
    int   laneId;
    float laneOffset;
    float s;
} RM_PositionData;
"""


class RM_PositionData(ct.Structure):
    _fields_ = [
        ("x", ct.c_float),
        ("y", ct.c_float),
        ("z", ct.c_float),
        ("h", ct.c_float),
        ("p", ct.c_float),
        ("r", ct.c_float),
        ("hRelative", ct.c_float),
        ("roadId", ct.c_int),
        ("junctionId", ct.c_int),
        ("laneId", ct.c_int),
        ("laneOffset", ct.c_float),
        ("s", ct.c_float),
    ]


def _setup_function_signatures():
    rm.RM_CreatePosition.argtypes = None
    rm.RM_CreatePosition.restype = ct.c_int

    rm.RM_SetLanePosition.argtypes = [
        ct.c_int,  # handle
        ct.c_int,  # roadId
        ct.c_int,  # laneId
        ct.c_float,  # laneOffset
        ct.c_float,  # s
        ct.c_bool,  # align
    ]
    rm.RM_SetLanePosition.restype = ct.c_int

    rm.RM_SetWorldPosition.argtypes = [
        ct.c_int,  # handle
        ct.c_float,  # x
        ct.c_float,  # y
        ct.c_float,  # z
        ct.c_float,  # h
        ct.c_float,  # p
        ct.c_float,  # r
    ]
    rm.RM_SetWorldPosition.restype = ct.c_int

    rm.RM_GetPositionData.argtypes = [
        ct.c_int,  # handle
        ct.POINTER(RM_PositionData),  # positionData
    ]
    rm.RM_GetPositionData.restype = ct.c_int


def print_info(pos_id):
    pos_data = RM_PositionData()
    rm.RM_GetPositionData(pos_id, ct.byref(pos_data))
    print(
        f"Position Data: x={pos_data.x}, y={pos_data.y}, z={pos_data.z}, h={pos_data.h}, p={pos_data.p}, r={pos_data.r}"
    )


_setup_function_signatures()

if rm.RM_Init("/sbsvf/scenarios/tw/maps/taoyuan_minsheng_fuxing.xodr".encode()) != 0:
    # if rm.RM_Init("/sbsvf/scenarios/mvp/maps/e6mini.xodr".encode()) != 0:
    print("Failed init xodr")
    exit(1)

pos_id = rm.RM_CreatePosition()
print("Created Position ID =", pos_id)
# ret = rm.RM_SetLanePosition(pos_id, 1, -2, 0.0, 21.0, False)
ret = rm.RM_SetLanePosition(pos_id, 0, -1, 0.0, 20.0, True)
print("RM_SetLanePosition ret =", ret)

print_info(pos_id)


# RM_DLL_API int RM_SetLanePosition(int handle, id_t roadId, int laneId, float laneOffset, float s, bool align)

# RM_DLL_API int RM_SetWorldPosition(int handle, float x, float y, float z, float h, float p, float r)
