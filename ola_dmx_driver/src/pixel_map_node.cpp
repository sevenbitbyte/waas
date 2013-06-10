#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iniparser.h>
#include <dictionary.h>

ros::NodeHandlePtr _nhPtr;

ros::Publisher _statusPub;
ros::Subscriber _frameSub;

ros::ServiceServer _refreshParamServ;

using namespace std;

//Subscriber callbacks
void imageCallback(const sensor_msgs::Image::ConstPtr& imageMsg);

//Helper functions
bool parseConfig(std::string configPath);

struct pixel_map_config{
    uint16_t array_size[2];

    struct dmx_map{
        //int universeId;
        int dmxOffset;
        int imageOffset;
        int pixels;
    };

    map<int, vector<dmx_map> > universes;
};

pixel_map_config _pixelMap;

int main(int argc, char** argv){
    ros::init (argc, argv, "point_downsample");
    _nhPtr = ros::NodeHandlePtr(new ros::NodeHandle());

    /*_tfListener = new tf::TransformListener();
    tf::TransformBroadcaster _broadcaster;
    _tfBroadcaster = &_broadcaster;*/


    _frameSub = _nhPtr->subscribe("/animation_host/frame", 1, imageCallback);

    //_statusPub = _nhPtr->advertise<sensor_msgs::PointCloud2> ("/point_downsample/points", 1);


    //Load settings from ini
    if(!parseConfig("/home/sevenbit/.waas/pixel_map.ini")){
        ROS_ERROR("Failed to parse pixel map configuration");
        return -1;
    }


    ros::spin();
}


void imageCallback(const sensor_msgs::Image::ConstPtr& imageMsg){
    //Using _pixelMap select mapped regions of image and generate valid DmxBuffer's
    //Use OlaClient to send all DmxBuffer's to correct universes
    //Should use a double buffering approach to avoid visiable tearing
}


bool parseConfig(std::string configPath){
    ROS_DEBUG("Parsing pixel map[%s]", configPath.c_str());

    dictionary* dict = iniparser_load(configPath.c_str());

    if(dict != NULL){
        int sections = iniparser_getnsec(dict);
        ROS_DEBUG("Pixel map has %i sections", sections);

        for(int i=0; i<sections; i++){
            string nodeName = iniparser_getsecname(dict, i);
            ROS_DEBUG("Section: %s", nodeName.c_str());

            if(nodeName == "encoding"){
                //Create expected keys
                string dmxEncodingKey = string(nodeName).append(":dmx_encoding");
                string dmxChanDirKey = string(nodeName).append(":dmx_chan_dir");
                string arrayShapeKey = string(nodeName).append(":array_shape");
                string arraySizeKey = string(nodeName).append(":array_size");
                string universesKey = string(nodeName).append(":universes");

                string dmxEncoding = iniparser_getstring(dict, dmxEncodingKey.c_str(), (char*)string("rgb").c_str());
                string dmxChanDir = iniparser_getstring(dict, dmxChanDirKey.c_str(), (char*)string("col").c_str());
                string arrayShape = iniparser_getstring(dict, arrayShapeKey.c_str(), (char*)string("square").c_str());
                string arraySize = iniparser_getstring(dict, arraySizeKey.c_str(), (char*)string("").c_str());
                string universeList = iniparser_getstring(dict, universesKey.c_str(), (char*)string("").c_str());

                //Check for required definitions
                if(arraySize.length() <= 0){
                    ROS_ERROR("No array_size specified in pixel map[%s]!", configPath.c_str());
                    iniparser_freedict(dict);
                    return false;
                }
                if(universeList.length() <= 0){
                    ROS_ERROR("No universes specified in pixel map[%s]!", configPath.c_str());
                    iniparser_freedict(dict);
                    return false;
                }

                //Check for unsupported configurations
                if(arrayShape != "square"){
                    ROS_WARN("Array shape[%s] not supported! Defaulting to square.", arrayShape.c_str());
                }

                if(dmxEncoding != "rgb"){
                    ROS_WARN("DMX encoding[%s] not supported! Defaulting to rgb.", arrayShape.c_str());
                }

                string::size_type index = arraySize.find(',');
                if(index == string::npos){
                    ROS_ERROR("Invalid array_size format in pixel map[%s], expected \"x,y\"", configPath.c_str());
                    iniparser_freedict(dict);
                    return false;
                }

                //Read led array dimensions
                string x = arraySize.substr(0, index);
                string y = arraySize.substr(index+1);

                if(x.length() < 1 || y.length() < 1){
                    ROS_ERROR("Invalid array_shape format in pixel map[%s], expected \"x,y\"", configPath.c_str());
                    iniparser_freedict(dict);
                    return false;
                }

                //Parse array dimensions
                _pixelMap.array_size[0] = atoi(x.c_str());
                _pixelMap.array_size[1] = atoi(y.c_str());

                //Read universe listing
                do{
                    index = universeList.find(',');

                    string univIdStr = universeList.substr(0, index);

                    if(univIdStr.length() < 1){
                        ROS_ERROR("In pixel map config[%s], invalid universes format!", configPath.c_str());
                        iniparser_freedict(dict);
                        return false;
                    }
                    universeList.erase(0, index+1);

                    int univId = atoi(univIdStr.c_str());

                    vector<pixel_map_config::dmx_map> emptyVec;
                    _pixelMap.universes.insert(make_pair( univId, emptyVec ));

                } while(index != string::npos);
            }
            else{
                ROS_WARN("DMX overrides not yet supported, ignoring section %s.", nodeName.c_str());
            }
        }

        if(_pixelMap.array_size[0] > 0 &&
           _pixelMap.array_size[1] > 0 &&
           _pixelMap.universes.size() > 0){
            return true;
        }
        else{
            ROS_WARN("No universes specified in pixel map! No output will be generated");
            return true;
        }
    }
    else{
        ROS_ERROR("Failed to open pixel map[%s]", configPath.c_str());
    }

    iniparser_freedict(dict);
    return false;
}
