CC = g++

OPENCV_FLAGS = -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab
BOOST_FLAGS = -lboost_program_options -lboost_filesystem -lboost_system
STD_FLAG = -std=c++11

BIN_DIR = bin
TARGET_MAIN = $(BIN_DIR)/main
TARGETS = $(TARGET_MAIN)

all: $(TARGETS)

$(TARGET_MAIN): main.cpp
	$(CC) $(STD_FLAG) $< -o $@ $(OPENCV_FLAGS) $(BOOST_FLAGS)

clean:
	-rm $(TARGETS)
