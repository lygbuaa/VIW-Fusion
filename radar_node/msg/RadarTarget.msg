uint8 FUSION=0
uint8 FRONT=1
uint8 LEFT_FRONT=2
uint8 RIGHT_FRONT=3

int32 id    #object id
float32 px  #position of x, meter
float32 py  #position of y, meter
float32 ix  #normalized x in image, (0~1)*width
float32 iy  #normalized y in image, (0~1)*height
float32 vx  #velocity of x, m/s
float32 vy  #velocity of y, m/s
uint32 type #0=unknow, 1=pedestrian, 2=bike, 3=car, 4=truck
uint32 domain # FUSION/FRONT/LEFT_FRONT/RIGHT_FRONT
