import math

MAX_ANGLES_URDF = 4
MAX_ANGLES_URDF_THUMB = 3
MAX_FINGERS = 6
NUM_FINGERS = 5
EXTRA_MOTORS = 1


PI = 3.141592653589793
HALF_PI = 1.570796326794896
ONE_HALF_PI = 4.71238898038469


POS_NAGATIVE_CDF = -1.466

def METER_TO_MILLIMETER(value):                                                 # Length in urdf is in meter, convert to millimter in calculation.
    return (value * 1000)

def MAP(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def CLAMP(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def ANGLE_COS(L_a, L_b, L_c):
    return math.acos((L_b * L_b + L_c * L_c - L_a * L_a) / (2 * L_b * L_c))         #L_a is the opposite side of the desired angle, L_b and L_c is the near side.

def ANGLE_SIN(angle_a, L_a, L_b):
    return math.asin((L_b * math.sin(angle_a)) / L_a)                                    # L_b is the opposite side of the desired angle.

def LENGTH_COS(angle, L_b, L_c):
    return math.sqrt(L_b *L_b + L_c * L_c - 2 * L_b * L_c * math.cos(angle))             # L_b and L_c is the near side of the desired angle.

def LENGTH_SIN(angle_a, L_a, angle_b):
    return ((math.sin(angle_b) * L_a) / math.sin(angle_a))                               #angle_a is the opposite angle of L_a, angle_b is the near angle of the desired side.

def SQRT(value):
    return (math.sqrt(value))

def ATAN(value):
    return (math.atan(value))

# Parameters in calibration:
ThumbParams = [
    #L_BP0, L_OP,  L_AB,  L_OA,  ∠OAR, L_CT0, L_OT,  ∠EOQ,  L_OE, L_CD,  L_ED,  ∠DEF,  L_EF,    XE, L_DF,   ∠EFD, L_CG
    7.49,   0.48,  11.00, 10.00, 0.5983, 39.79, 2.13, 0.2089, 49.91, 9.01, 10.27, 1.3161, 25.20, 48.83, 24.70, 0.4141, 12.48 # thumb
]

FingerParams = [
    #X_A0, L_AP,  L_AC,  L_OC,  ∠BOQ,  ∠COD,  ∠EDF,  L_OD,  L_CD, L_OB, L_DE,  L_BE,  L_DF
    [ 6.20, 8.93, 16.64, 12.85, 0.5253, 0.7782, 0.9313, 32.65, 25.17, 7.97, 7.69, 31.07, 43.27], # index_finger
    [ 6.49, 8.93, 16.64, 13.06, 0.5253, 0.7781, 0.9342, 37.52, 29.67, 7.97, 7.69, 35.92, 43.25], # middle_finger
    [ 6.19, 8.93, 16.64, 12.84, 0.5253, 0.7771, 0.9313, 32.67, 25.18, 7.97, 7.69, 31.07, 43.26], # ring_finger
    [ 6.37, 8.93, 16.64, 13.08, 0.5253, 0.7482, 0.8216, 29.06, 21.41, 7.97, 7.53, 28.08, 33.11]  # little_finger
]

# Offset angle to the initial state in URDF, the output angle needs to minus it.
OffsetAngleForURDF_Thumb = [
    #proximal,          #connecting,        #distal,           
    0.6709008910354062, 2.7972123559022117, 1.3087610621598222
]

OffsetAngleForURDF = [
    #abpart,            proximal,           distal,             connecting,
    [3.2234745075598523, 3.289937319993764, 3.1162262515620833, 2.626962850209491],
    [3.1833006883299437, 3.2373129742644506, 3.1008555171484615, 2.58818205017669],
    [3.2251391241575877, 3.2909284310550326, 3.1132137783847593, 2.627968560411981],
    [3.1899970337180843, 3.219561574737824, 3.0455334793320183, 2.5281789803624695]
]

#
#Calculate the angle of thumb with the offset to reference.
def THUMB_OffsetToAngle(offset_to_ref):
    pos = METER_TO_MILLIMETER(offset_to_ref)

    # Initial the array. 
    # Thumb has only 3 angles in URDF
    Angle_Thumb = [0] * MAX_ANGLES_URDF_THUMB
    L_BP0 = ThumbParams[0]
    L_OP = ThumbParams[1]
    L_AB = ThumbParams[2]
    L_OA = ThumbParams[3]
    Angle_OAR = ThumbParams[4]
    L_CT0 = ThumbParams[5]
    L_OT = ThumbParams[6]
    Angle_EOQ = ThumbParams[7]
    L_OE = ThumbParams[8]
    L_CD = ThumbParams[9]
    L_ED = ThumbParams[10]
    Angle_DEF = ThumbParams[11]
    L_EF = ThumbParams[12]
    XE = ThumbParams[13]

    L_BP = L_BP0 + pos
    L_OB = SQRT(L_OP * L_OP + L_BP * L_BP)
    Angle_OBP = ATAN(L_OP / L_BP)
    Angle_AOB = ANGLE_COS(L_AB, L_OA, L_OB)
    Angle_AOQ = Angle_AOB - Angle_OBP
    Angle_QOS = Angle_AOQ - Angle_OAR               # Angle of th_proximal link

    L_CT = L_CT0 + pos
    L_OC = SQRT(L_CT * L_CT + L_OT * L_OT)
    Angle_OCT = ATAN(L_OT / L_CT)
    Angle_EOC = Angle_EOQ + Angle_OCT
    L_CE = LENGTH_COS(Angle_EOC, L_OE, L_OC)
    Angle_CED = ANGLE_COS(L_CD, L_CE, L_ED)
    Angle_CEF = Angle_CED + Angle_DEF
    L_CF = LENGTH_COS(Angle_CEF, L_CE, L_EF)
    Angle_ECF = ANGLE_COS(L_EF, L_CE, L_CF)
    Angle_ECU = HALF_PI - ATAN((XE - L_CT) / L_CE)
    Angle_FCU = Angle_ECU - Angle_ECF
    Angle_QCF = HALF_PI - Angle_FCU
    Angle_ECD = ANGLE_COS(L_ED, L_CD, L_CE)
    Angle_ECQ = Angle_QCF - Angle_ECF
    Angle_DCT = Angle_ECD + Angle_ECQ + HALF_PI     # Angle of th_connecting link

    Angle_OCD = Angle_DCT - Angle_OCT
    L_OD = LENGTH_COS(Angle_OCD, L_OC, L_CD)
    Angle_OED = ANGLE_COS(L_OD, L_OE, L_ED)         # Angle of th_distal link
 
 
    Angle_Thumb[0] = Angle_QOS - OffsetAngleForURDF_Thumb[0]
    Angle_Thumb[1] = Angle_DCT - OffsetAngleForURDF_Thumb[1]
    Angle_Thumb[2] = OffsetAngleForURDF_Thumb[2] - Angle_OED
 
    return Angle_Thumb


#
#Calculate the angle of other four fingers with the offset to the reference.
def FINGER_OffsetToAngle(finger_id, offset_to_ref):
    pos = METER_TO_MILLIMETER(offset_to_ref)

    # Modify index to match the array sequence.
    finger_id -= 1 

    # Initial the array. 
    Angle_Finger = [[0] * MAX_ANGLES_URDF for _ in range(MAX_ANGLES_URDF)]

    X_A0 = FingerParams[finger_id][0]
    L_AP = FingerParams[finger_id][1]
    L_AC = FingerParams[finger_id][2]
    L_OC = FingerParams[finger_id][3]
    Angle_BOQ = FingerParams[finger_id][4]
    Angle_COD = FingerParams[finger_id][5]
    Angle_EDF = FingerParams[finger_id][6]
    L_OD = FingerParams[finger_id][7]
    L_OB = FingerParams[finger_id][9]
    L_DE = FingerParams[finger_id][10]
    L_BE = FingerParams[finger_id][11]


    L_OP = X_A0 + pos
    L_OA = SQRT(L_OP * L_OP + L_AP * L_AP)
    Angle_AOP = ANGLE_COS(L_AP, L_OA, L_OP)
    Angle_OAP = HALF_PI - Angle_AOP
    Angle_CAO = ANGLE_COS(L_OC, L_AC, L_OA)
    Angle_CAR = ONE_HALF_PI - Angle_OAP - Angle_CAO # Angle of slider_abpart

    Angle_COA = ANGLE_COS(L_AC, L_OC, L_OA)
    Angle_COP = Angle_AOP + Angle_COA
    Angle_DOP = Angle_COD + Angle_COP               # Angle of proximal link

    Angle_DOQ = PI - Angle_DOP
    Angle_DOB = Angle_BOQ + Angle_DOQ
    L_BD = LENGTH_COS(Angle_DOB, L_OB, L_OD)
    Angle_BDO = ANGLE_COS(L_OB, L_BD, L_OD)
    Angle_BDE = ANGLE_COS(L_BE, L_DE, L_BD)
    Angle_EDO = Angle_BDE - Angle_BDO
    L_OE = LENGTH_COS(Angle_BDE, L_DE, L_OD)
    Angle_EBO = ANGLE_COS(L_OE, L_BE, L_OB)        # Angle of connecting link

    Angle_FDO = Angle_EDF + Angle_EDO              # Angle of distal link

    Angle_Finger[finger_id][0] = -Angle_CAR + OffsetAngleForURDF[finger_id][0]# slider_abpart
    Angle_Finger[finger_id][1] = -Angle_DOP + OffsetAngleForURDF[finger_id][1]# proximal
    Angle_Finger[finger_id][2] = -Angle_FDO + OffsetAngleForURDF[finger_id][2]# distal
    Angle_Finger[finger_id][3] = -Angle_EBO + OffsetAngleForURDF[finger_id][3]# connecting

    return Angle_Finger[finger_id]


#
#Calculate the angle with the position
def HAND_FingerPosToAngle(finger_id, pos):

    # Initial the array.
    angles = [0] * MAX_ANGLES_URDF 

    if finger_id >= NUM_FINGERS + EXTRA_MOTORS:
        # Invalid id
        return
    elif finger_id >= NUM_FINGERS:
        # The input value of thumb root in urdf is the value of slider position in joint_tate_publisher_gui
        angles = pos

    else:

        if finger_id == 0:
            angles = THUMB_OffsetToAngle(pos)
        else:
            angles = FINGER_OffsetToAngle(finger_id, pos)

    return angles

