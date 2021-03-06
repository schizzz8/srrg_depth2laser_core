#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "srrg_system_utils/system_utils.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_writer.h"
#include "srrg_txt_io/pinhole_image_message.h"
#include "srrg_txt_io/laser_message.h"
#include "srrg_txt_io/sensor_message_sorter.h"
#include "srrg_scan_extractor/scan_extractor.h"

using namespace std;
using namespace srrg_core;
using namespace srrg_depth2laser_core;

// Help objects to force linking
PinholeImageMessage i;
LaserMessage l;

const char* banner[] = {
    "srrg_depth2laser_app: example on how to convert depth images into laser scans",
    "",
    "usage: srrg_depth2laser_app [options] <dump_file>",
    0
};

int main(int argc, char ** argv) {
    if (argc < 2 || !strcmp(argv[1], "-h")) {
        printBanner(banner);
        return 0;
    }

    string filename = argv[1];
    ScanExtractor extractor(filename);

    string input_filename = argv[2];
    MessageReader reader;
    reader.open(input_filename.c_str());
    BaseMessage* msg = 0;
    MessageWriter writer;
    writer.open(input_filename.substr(0,input_filename.find("."))+"_with_laser.txt");
    while ((msg = reader.readMessage())) {
        msg->untaint();

        PinholeImageMessage* pinhole_image_msg=dynamic_cast<PinholeImageMessage*>(msg);
        if (pinhole_image_msg && !strcmp(pinhole_image_msg->topic().c_str(),"/camera/depth/image_raw")) {
	  LaserMessage laser_msg;
	  extractor.setParameters(pinhole_image_msg->seq(),pinhole_image_msg->timestamp(),laser_msg);
	  extractor.compute(pinhole_image_msg->image(),laser_msg);
	  laser_msg.setOdometry(pinhole_image_msg->odometry());
	  writer.writeMessage(laser_msg);
        }
    }
    cerr << "done" << endl;
}
