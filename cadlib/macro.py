import numpy as np

ALL_COMMANDS = ['Line', 'Arc', 'Circle', 'Spline', 'SCP', 'EOS', 'SOL', 'Ext', 'Rev', 'Pocket', 'Groove', 'Shell', 'Chamfer', 'Fillet', 'Draft', 'Mirror', 'Hole', 'Topo', 'Select', 'MirrorStart', 'NoSharedIncluded', 'NoSharedIncludedEnd', 'AllOrientedIncluded1', 'AllOrientedIncluded2', 'AllOrientedIncludedEnd', 'AllPartiallySharedIncluded', 'AllPartiallySharedIncludedEnd']
LINE_IDX = ALL_COMMANDS.index('Line')                                           # 0
ARC_IDX = ALL_COMMANDS.index('Arc')                                             # 1
CIRCLE_IDX = ALL_COMMANDS.index('Circle')                                       # 2
SPLINE_IDX = ALL_COMMANDS.index('Spline')                                       # 3
SCP_IDX = ALL_COMMANDS.index('SCP')                                             # 4     Spline Control Point
EOS_IDX = ALL_COMMANDS.index('EOS')                                             # 5
SOL_IDX = ALL_COMMANDS.index('SOL')                                             # 6
EXT_IDX = ALL_COMMANDS.index('Ext')                                             # 7
REV_IDX = ALL_COMMANDS.index('Rev')                                             # 8
POCKET_IDX = ALL_COMMANDS.index('Pocket')                                       # 9
GROOVE_IDX = ALL_COMMANDS.index('Groove')                                       # 10
SHELL_IDX = ALL_COMMANDS.index('Shell')                                         # 11
CHAMFER_IDX = ALL_COMMANDS.index('Chamfer')                                     # 12
FILLET_IDX = ALL_COMMANDS.index('Fillet')                                       # 13
DRAFT_IDX = ALL_COMMANDS.index('Draft')                                         # 14
MIRROR_IDX = ALL_COMMANDS.index('Mirror')                                       # 15
HOLE_IDX = ALL_COMMANDS.index('Hole')                                           # 16
TOPO_IDX = ALL_COMMANDS.index('Topo')                                           # 17
SELECT_IDX = ALL_COMMANDS.index('Select')                                       # 18
MIRROR_START_IDX = ALL_COMMANDS.index('MirrorStart')                            # 19
NO_SHARED_INCLUDED_IDX = ALL_COMMANDS.index('NoSharedIncluded')                 # 20
NO_SHARED_INCLUDED_END_IDX = ALL_COMMANDS.index('NoSharedIncludedEnd')          # 21
ALL_ORIENTED_INCLUDED_1_IDX = ALL_COMMANDS.index('AllOrientedIncluded1')        # 22
ALL_ORIENTED_INCLUDED_2_IDX = ALL_COMMANDS.index('AllOrientedIncluded2')        # 23
ALL_ORIENTED_INCLUDED_END_IDX = ALL_COMMANDS.index('AllOrientedIncludedEnd')    # 24
ALL_PARTIALLY_INCLUDED_IDX = ALL_COMMANDS.index('AllPartiallySharedIncluded')   # 25
ALL_PARTIALLY_INCLUDED_END_IDX = ALL_COMMANDS.index('AllPartiallySharedIncludedEnd')   # 26

BOOLEAN_OPERATIONS = ["AddFeatureOperation", "CutFeatureOperation", "IntersectFeatureOperation"]
EXTENT_TYPE = ["OffsetLimit", "UpToNextLimit", "UpToLastLimit", "UpToPlaneLimit", "UpToSurfaceLimit", "UpThruNextLimit"]
SELECT_TYPE = ["Wire", "Face", "Edge", "Multiply_Face", "Sub_Face"]
BODY_TYPE = ["None", "OriginElements", "Sketch", "Pad", "Shaft", 'Pocket', "Add", "Remove", "Intersect", "Shell", "Chamfer", "EdgeFillet", "Mirror", "Hole"]
CatChamferMode = ["catTwoLengthChamfer", "catLengthAngleChamfer"]
CatChamferOrientation = ["catNoReverseChamfer", "catReverseChamfer"]
CatChamferPropagation = ["catTangencyChamfer", "catMinimalChamfer"]
CatDraftNeutralPropagationMode = ["catNoneDraftNeutralPropagationMode", "catSmoothDraftNeutralPropagationMode"]
CatDraftMode = ["catStandardDraftMode", "catReflectKeepFaceDraftMode", "catReflectKeepEdgeDraftMode"]
CatDraftMultiselectionMode = ["catNoneDraftMultiselectionMode", "catDraftMultiselectionByNeutralMode"]

PAD_VAL = -1
N_ARGS_SKETCH = 5  # sketch parameters: x, y, alpha, f, r
N_ARGS_PLANE = 3  # sketch plane orientation: theta, phi, gamma
N_ARGS_TRANS = 4  # sketch plane origin + sketch bbox size: p_x, p_y, p_z, s
# extrude, revolve, ... parameters: length1, length2, length1_type, length2_type, angle1, angle2, boolean
N_ARGS_BODY_PARAM = 7
# shell, chamfer, ... parameters: thickness1, thickness2, length1, length2, radius, alpha, hole r, hole depth, hole type
N_ARGS_FINISH_PARAM = 9
N_ARGS_SELECT_PARAM = 4  # select parameters: select_type, body_type, body_no, no
N_ARGS_EXT = N_ARGS_PLANE + N_ARGS_TRANS + N_ARGS_BODY_PARAM
N_ARGS = N_ARGS_SKETCH + N_ARGS_EXT + N_ARGS_FINISH_PARAM + N_ARGS_SELECT_PARAM

SOL_VEC = np.array([SOL_IDX, *([PAD_VAL] * N_ARGS)])
EOS_VEC = np.array([EOS_IDX, *([PAD_VAL] * N_ARGS)])
TOPO_VEC = np.array([TOPO_IDX, *([PAD_VAL] * N_ARGS)])
SPLINE_VEC = np.array([SPLINE_IDX, *([PAD_VAL] * N_ARGS)])
MIRROR_START_VEC = np.array([MIRROR_START_IDX, *([PAD_VAL] * N_ARGS)])
NO_SHARED_INCLUDED_VEC = np.array([NO_SHARED_INCLUDED_IDX, *([PAD_VAL] * N_ARGS)])
NO_SHARED_INCLUDED_END_VEC = np.array([NO_SHARED_INCLUDED_END_IDX, *([PAD_VAL] * N_ARGS)])
ALL_ORIENTED_INCLUDED_1_VEC = np.array([ALL_ORIENTED_INCLUDED_1_IDX, *([PAD_VAL] * N_ARGS)])
ALL_ORIENTED_INCLUDED_2_VEC = np.array([ALL_ORIENTED_INCLUDED_2_IDX, *([PAD_VAL] * N_ARGS)])
ALL_ORIENTED_INCLUDED_END_VEC = np.array([ALL_ORIENTED_INCLUDED_END_IDX, *([PAD_VAL] * N_ARGS)])
ALL_PARTIALLY_INCLUDED_VEC = np.array([ALL_PARTIALLY_INCLUDED_IDX, *([PAD_VAL] * N_ARGS)])
ALL_PARTIALLY_INCLUDED_END_VEC = np.array([ALL_PARTIALLY_INCLUDED_END_IDX, *([PAD_VAL] * N_ARGS)])

# 指令对应参数的掩码，需要扩展到22位
CMD_ARGS_MASK = np.array([[1, 1, 0, 0, 0, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],                     # line
                          [1, 1, 1, 1, 0, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],                     # arc
                          [1, 1, 0, 0, 1, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],                     # circle
                          [0, 0, 0, 0, 0, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],                     # spline
                          [1, 1, 0, 0, 0, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],                     # scp
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],                # EOS
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],                # SOL
                          [*[0]*N_ARGS_SKETCH, *[1]*(N_ARGS_PLANE + N_ARGS_TRANS), 1, 1, 1, 1, 0, 0, 1, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # Extrude
                          [*[0]*N_ARGS_SKETCH, *[1]*(N_ARGS_PLANE + N_ARGS_TRANS), 0, 0, 0, 0, 1, 1, 1, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # Revolve
                          [*[0]*N_ARGS_SKETCH, *[1]*(N_ARGS_PLANE + N_ARGS_TRANS), 1, 1, 1, 1, 0, 0, 0, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # Pocket
                          [*[0]*N_ARGS_SKETCH, *[1]*(N_ARGS_PLANE + N_ARGS_TRANS), 0, 0, 0, 0, 1, 1, 0, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # Groove
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, 1, 1, 0, 0, 0, 0, 0, 0, 0, *[0]*N_ARGS_SELECT_PARAM],  # Shell
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, 0, 0, 1, 1, 0, 0, 0, 0, 0, *[0]*N_ARGS_SELECT_PARAM],  # Chamfer
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, 0, 0, 0, 0, 1, 0, 0, 0, 0, *[0]*N_ARGS_SELECT_PARAM],  # Fillet
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, 0, 0, 0, 0, 0, 1, 0, 0, 0, *[0]*N_ARGS_SELECT_PARAM],  # Draft
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],   # Mirror
                          [1, 1, 0, 0, 0, *[0]*N_ARGS_EXT, 0, 0, 0, 0, 0, 0, 1, 1, 1, *[0]*N_ARGS_SELECT_PARAM],  # Hole
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # TOPO
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[1]*N_ARGS_SELECT_PARAM],  # Select
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # MirrorStart
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # NoSharedIncluded
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # NoSharedIncludedEnd
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # AllOrientedIncluded1
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # AllOrientedIncluded2
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # AllOrientedIncludedEnd
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM],  # AllPartiallySharedIncluded
                          [*[0]*N_ARGS_SKETCH, *[0]*N_ARGS_EXT, *[0]*N_ARGS_FINISH_PARAM, *[0]*N_ARGS_SELECT_PARAM]   # AllPartiallySharedIncludedEnd
                         ])

NORM_FACTOR = 0.75 # scale factor for normalization to prevent overflow during augmentation

MAX_N_EXT = 10 # maximum number of extrusion
MAX_N_LOOPS = 6 # maximum number of loops per sketch
MAX_N_CURVES = 15 # maximum number of curves per loop
MAX_TOTAL_LEN = 100 # maximum cad sequence length
ARGS_DIM = 256

ARGS_N = 256

def Get_integer_and_fraction(num, n=256):
    # fraction = num
    # num = np.trunc(num).clip(min=0, max=n-1).astype(np.int)
    # fraction = ((fraction - num) * n).round().clip(min=0, max=n-1).astype(np.int)
    # return num, fraction
    return np.round(num), 0
