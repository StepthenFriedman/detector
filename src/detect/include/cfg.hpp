#define absolute(x,y) sqrt(((x)*(x))+((y)*(y)))
#define abssub(x,y) (((x)>(y))?((x)-(y)):((y)-(x)))
#define absdiv(x,y) (((x)>(y))?((x)/(y)):((y)/(x)))
#define getvec(a,b) ((b).x-(a).x),((b).y-(a).y)
#define crossproduct(a,b) ((((a).x*(b).y)-((a).y*(b).x))>0)

#define PREPROCESSOR CHANNELSUB
#define IDENTIFY_METHOD MSE
#define COLOR RED
#define DEBUG

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 1024

#define FPX 20
#define MSG_DELAY 50ms  /* should be equal to (1000/FPX)ms */
#define BINARY_THRESH 120

#define COLOR_RATIO 1.2

#define TRANS_WID 215
#define TRANS_HEI 340

#define LIGHT_ARRAY_LENGTH 10000
#define ARMOR_ARRAY_LENGTH (LIGHT_ARRAY_LENGTH/2)

#define LIGHT_MIN_RATIO 1.3
#define LIGHT_MIN_ANGLE 30
#define LIGHT_ANGLE_ERR 30

#define ARMOR_RATIO_MAX 4
#define ARMOR_ANGLE_ERROR 20 
#define ARMOR_HEIGHT_ERROR 1.5
#define ARMOR_NUMBER_WID_RATIO 0.77
#define ARMOR_NUMBER_HEI_RATIO 1.4

#define IDENTIFY_THRESH 20 //if minimal mse overflow this value, identify fail.
#define IDENTIFY_MEAN_DECLINE 2

#define WINDOWS_TITLE "display"
#define RESOURCE_PATH "./resource/"
#define INPUT_SOURCE "test.avi"
#define OUTPUT_SOURCE "opt_1.mp4"

#define INPUT_PATH std::string(RESOURCE_PATH)+"input/"+INPUT_SOURCE
#define OUTPUT_PATH std::string(RESOURCE_PATH)+"output/"+OUTPUT_SOURCE
#define DECIMAL_PATH(i) std::string(RESOURCE_PATH)+"decimal/"+std::to_string(i)+".png"

#define RED 114
#define BLUE 514
#define MSE 1919
#define MLP 810
#define CHANNELSUB 5201314
#define 