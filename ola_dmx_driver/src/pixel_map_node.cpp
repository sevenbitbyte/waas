#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iniparser.h>
#include <dictionary.h>

#include <ola/DmxBuffer.h>
#include <ola/OlaClient.h>
#include <ola/StreamingClient.h>

ros::NodeHandlePtr _nhPtr;

ros::Publisher _statusPub;
ros::Subscriber _frameSub;

ros::ServiceServer _refreshParamServ;

using namespace std;

//Subscriber callbacks
void imageCallback(const sensor_msgs::Image::ConstPtr& imageMsg);

//Helper functions
bool parseConfig(std::string configPath);

#define BYTES_PER_PORT      (512/4)
#define PIXELS_PER_PORT     (BYTES_PER_PORT/3)


struct pixel_map_config{
    uint16_t array_size[2];

    struct dmx_map{
        int dmxOffset;
        int imageOffset;
        int pixels;
        int pixelStep;
    };

    map<int, vector<dmx_map> > universes;
};

pixel_map_config _pixelMap;
map<int, ola::DmxBuffer*> _dmxBuffers;
ola::StreamingClient _olaCient(true);

int main(int argc, char** argv){
    ros::init (argc, argv, "point_downsample");
    _nhPtr = ros::NodeHandlePtr(new ros::NodeHandle());


    _frameSub = _nhPtr->subscribe("/animation_host/frame", 1, imageCallback);



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

    map<int, ola::DmxBuffer*>::iterator bufferIter = _dmxBuffers.begin();

    //Render into DmuxBuffers
    for(; bufferIter != _dmxBuffers.end(); bufferIter++){
        ola::DmxBuffer* buffer = bufferIter->second;
        vector<pixel_map_config::dmx_map>::iterator mapIter = _pixelMap.universes[ bufferIter->first ].begin();

        for(; mapIter != _pixelMap.universes[ bufferIter->first ].end(); mapIter++){
            if(mapIter->pixelStep == 3){
                buffer->SetRange(mapIter->dmxOffset, &imageMsg->data[mapIter->imageOffset], mapIter->pixels * 3);
            }
            else{
                for(int i=0; i < mapIter->pixels; i++){
                    int pixelIndex = i * mapIter->pixelStep;
                    buffer->SetRange(mapIter->dmxOffset, &imageMsg->data[mapIter->imageOffset + pixelIndex], 3);
                }
            }
        }
    }

    for(bufferIter = _dmxBuffers.begin(); bufferIter != _dmxBuffers.end(); bufferIter++){
        _olaCient.SendDmx(bufferIter->first, *bufferIter->second);
    }
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

                string dmxEncoding = iniparser_getstring(dict, dmxEncodingKey, "rgb");
                string dmxChanDir = iniparser_getstring(dict, dmxChanDirKey, "col");
                string arrayShape = iniparser_getstring(dict, arrayShapeKey, "square");
                string arraySize = iniparser_getstring(dict, arraySizeKey, "");
                string universeList = iniparser_getstring(dict, universesKey, "");

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

                //Check for unsupported/non-optimal configurations
                if(arrayShape != "square"){
                    ROS_WARN("Array shape[%s] not supported! Defaulting to square.", arrayShape.c_str());
                }

                if(dmxChanDir == "row" ){
                    ROS_WARN("Setting dmx_chan_dir to value other than col is supported but may not be efficient");
                }
                else if(dmxChanDir != "col"){
                    ROS_ERROR("Unsupported dmx_chan_dir[%s] in pixel map[%s]", dmxChanDir.c_str(), configPath.c_str());
                    iniparser_freedict(dict);
                    return false;
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

                int mappedPorts=0;

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

                    vector<pixel_map_config::dmx_map> dmxMapVec;

                    if(arrayShape == "square"){
                        for(int i=0; i<4; i++){
                            pixel_map_config::dmx_map dmxMap;

                            if(dmxChanDir == "col"){
                                dmxMap.pixels = _pixelMap.array_size[1];
                                dmxMap.pixelStep = 3;
                                dmxMap.dmxOffset = (BYTES_PER_PORT * i);
                                dmxMap.imageOffset = (mappedPorts * _pixelMap.array_size[1]) * 3;
                            }
                            else if(dmxChanDir == "row"){
                                dmxMap.pixels = _pixelMap.array_size[0];
                                dmxMap.pixelStep = _pixelMap.array_size[0] * 3;
                                dmxMap.dmxOffset = (BYTES_PER_PORT * i);
                                dmxMap.imageOffset = (mappedPorts * _pixelMap.array_size[0]) * 3;
                            }

                            dmxMapVec.push_back(dmxMap);
                            mappedPorts++;
                        }
                    }

                    _pixelMap.universes.insert(make_pair( univId, dmxMapVec ));
                    _dmxBuffers.insert(make_pair( univId, new ola::DmxBuffer() ));

                } while(index != string::npos);
            }
            else{
                //Parse override commands
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
