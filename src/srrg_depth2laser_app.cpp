#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include "srrg_system_utils/system_utils.h"
#include "srrg_txt_io/message_reader.h"
#include "srrg_txt_io/message_writer.h"
#include "srrg_txt_io/pinhole_image_message.h"
#include "srrg_txt_io/laser_message.h"
#include "srrg_txt_io/sensor_message_sorter.h"
#include "srrg_core_map/pinhole_camera_info.h"

using namespace std;
using namespace srrg_core;
using namespace srrg_core_map;

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
    string laser_topic = "/scan";
    string laser_frame_id = "/laser_frame";
    float angle_min = -M_PI/6;
    float angle_max = M_PI/6;
    int num_ranges = 256;
    float range_min = 0.1;
    float range_max = 5.0;
    float laser_plane_thickness = 0.05;
    float squared_max_norm=range_max*range_max;
    float squared_min_norm=range_min*range_min;
    float angle_increment=(angle_max-angle_min)/num_ranges;
    float inverse_angle_increment = 1./angle_increment;

    bool gotInfo = false;
    Eigen::Matrix3f K;
    Eigen::Matrix3f inv_K;
    Eigen::Isometry3f camera_transform;
    Eigen::Isometry3f laser_transform = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f camera2laser_transform;
    MessageReader reader;
    reader.open(argv[1]);
    BaseMessage* msg = 0;
    MessageWriter writer;
    writer.open("depth2laser.txt");
    while ((msg = reader.readMessage())) {
        msg->untaint();

        PinholeImageMessage* pinhole_image_msg=dynamic_cast<PinholeImageMessage*>(msg);
        if (pinhole_image_msg) {
            if(!gotInfo){
                K = pinhole_image_msg->cameraMatrix();
                inv_K = K.inverse();
                cerr << "Got camera matrix!" << endl;
                camera_transform = pinhole_image_msg->offset();
                cerr << "Got camera transform!" << endl;
                laser_transform.translation() = camera_transform.translation();
                camera2laser_transform = laser_transform.inverse()*camera_transform;
                gotInfo = true;
            }
            if(gotInfo && !strcmp(pinhole_image_msg->topic().c_str(),"/camera/depth/image_raw")){
                //cv::imshow(pinhole_image_msg->topic(), pinhole_image_msg->image());
                //cv::waitKey(30);
                vector<float> ranges(num_ranges,range_max+0.1);
                LaserMessage* laser_msg = new LaserMessage(laser_topic,
                                                           laser_frame_id,
                                                           pinhole_image_msg->seq(),
                                                           pinhole_image_msg->timestamp());

                const cv::Mat image = pinhole_image_msg->image();

                for(int i =0;i<image.rows;i++){
                    const ushort* row_ptr = image.ptr<ushort>(i);
                    for(int j=0;j<image.cols;j++){
                        ushort id=row_ptr[j];
                        if(id!=0){
                            float d=1e-3*id;
                            Eigen::Vector3f image_point(j*d,i*d,d);
                            Eigen::Vector3f camera_point=inv_K*image_point;
                            Eigen::Vector3f laser_point=camera2laser_transform*camera_point;

                            if (fabs(laser_point.z())<laser_plane_thickness){
                                float theta=atan2(laser_point.y(),laser_point.x());
                                float range=laser_point.head<2>().squaredNorm();
                                if (range<squared_min_norm)
                                    continue;
                                if (range>squared_max_norm)
                                    continue;
                                range=sqrt(range);
                                int bin=(int)((theta-angle_min)*inverse_angle_increment);
                                if (bin<0||bin>=ranges.size())
                                    continue;
                                if(ranges[bin]>range)
                                    ranges[bin]=range;
                            }
                        }
                    }
                }
                laser_msg->setRanges(ranges);
                laser_msg->setTopic(laser_topic);
                laser_msg->setFrameId(laser_frame_id);
                laser_msg->setMinRange(range_min);
                laser_msg->setMaxRange(range_max);
                laser_msg->setMinAngle(angle_min);
                laser_msg->setMaxAngle(angle_max);
                laser_msg->setAngleIncrement(angle_increment);
                laser_msg->setOffset(laser_transform);
		laser_msg->setOdometry(pinhole_image_msg->odometry());
                writer.writeMessage(*laser_msg);
            }
        }
        //writer.writeMessage(*msg);
    }
    cerr << "done" << endl;
}
