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


#include <string>
#include <iostream>
#include <sstream>

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


struct point {
    int x;
    int y;

    string toString(){
        stringstream stream;

        stream << "img(" << x << "," << y <<")";

        return stream.str();
    }
};


struct DmxAddress {
    int universe;
    int offset;

    int distance(DmxAddress& other);

    string toString(){
        stringstream stream;

        stream << "dmx(" << universe << "," << offset <<")";

        return stream.str();
    }
};


struct LedRun {
    DmxAddress dmxStart;
    DmxAddress dmxEnd;
    point imageStart;
    point imageEnd;

    string toString(){

        stringstream stream;

        stream << dmxStart.toString() << " " << dmxEnd.toString() << " " << imageStart.toString() << " " << imageEnd.toString();

        return stream.str();
    }

};


struct LedStrand {
    int subPortNumber;
    vector<LedRun> runs;

    bool reverse_zig_zag;

    string toString(){
        stringstream stream;

        stream << "[strand-" << subPortNumber << "]" << endl
               << "reverse=" << reverse_zig_zag << endl;

        for(int i=0; i<runs.size(); i++){
            stream << runs[i].toString() << endl;
        }

        return stream.str();
    }
};


struct LedPort {
    int portNumber;
    int maxLength;
    bool autoAllocDmx;
    DmxAddress start;
    DmxAddress end;
    vector<LedStrand> strands;

    string toString(){
        stringstream stream;

        stream << "[port-" << portNumber << "]" << endl
               << "length=" << maxLength << endl
               << "autoAlloc=" << autoAllocDmx << endl
               << "start=" << start.toString() << endl
               << "end=" << end.toString() << endl;

        for(int i=0; i<strands.size(); i++){
            stream << strands[i].toString() << endl;
        }

        return stream.str();
    }
};

struct LedArray{
    vector<int> run_lengths;
    vector<LedPort*> ports;

    string toString(){
        stringstream stream;

        stream << "[general]" << endl;

        for(int i=0; i<ports.size(); i++){

            stream << ports[i]->toString() << endl;

        }
    }
};




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

//pixel_map_config _pixelMap;

LedArray* ledArray = NULL;

map<int, ola::DmxBuffer*> _dmxBuffers;
ola::StreamingClient _olaCient(true);

int main(int argc, char** argv){
    //ros::init (argc, argv, "point_downsample");
    //_nhPtr = ros::NodeHandlePtr(new ros::NodeHandle());

    //_frameSub = _nhPtr->subscribe("/animation_host/frame", 1, imageCallback);

    //Load settings from ini
    if(!parseConfig("/home/sevenbit/.waas/pixel_map.ini")){
        ROS_ERROR("Failed to parse pixel map configuration");
        return -1;
    }

    if(ledArray != NULL){
         cout << "DEBUG" <<endl;
         cout << ledArray->toString();
    }

    //ros::spin();
}


void imageCallback(const sensor_msgs::Image::ConstPtr& imageMsg){
    /*map<int, ola::DmxBuffer*>::iterator bufferIter = _dmxBuffers.begin();

    //Render into DmuxBuffers
    for(; bufferIter != _dmxBuffers.end(); bufferIter++){
        ola::DmxBuffer* buffer = bufferIter->second;
        vector<pixel_map_config::dmx_map>::iterator univIter = _pixelMap.universes[ bufferIter->first ].begin();

        //Iterate over every dmx_map
        for(; univIter != _pixelMap.universes[ bufferIter->first ].end(); univIter++){
            if(univIter->pixelStep == 3){
                buffer->SetRange(univIter->dmxOffset, &imageMsg->data[univIter->imageOffset], univIter->pixels * 3);
            }
            else{
                for(int i=0; i < univIter->pixels; i++){
                    int pixelIndex = i * univIter->pixelStep;
                    buffer->SetRange(univIter->dmxOffset, &imageMsg->data[univIter->imageOffset + pixelIndex], 3);
                }
            }
        }
    }

    //Transmit buffers
    for(bufferIter = _dmxBuffers.begin(); bufferIter != _dmxBuffers.end(); bufferIter++){
        _olaCient.SendDmx(bufferIter->first, *bufferIter->second);
    }*/
}


vector<int> parseIntList(string str){
    vector<int> intList;

    int pos=0;

    printf("Parsing [%s]\n", str.c_str());

    while(pos < str.length()-1){
        int commaIdx = str.find(',', pos);

        if(commaIdx == string::npos){
            string numberStr = str.substr(pos, str.length()-1);
            pos = str.length();

            int num = atoi(numberStr.c_str());
            intList.push_back(num);
            break;
        }
        else{
            printf("pos=%i\n", pos);
            string numberStr = str.substr(pos, commaIdx-pos);

            printf("numberStr=%s\n", numberStr.c_str());

            int count=1;
            int openParen = str.find('(', pos);
            int closeParen = str.find(')', pos);

            if(openParen < commaIdx && closeParen < commaIdx && openParen > pos && closeParen > pos){
                string countStr = str.substr(openParen, closeParen-openParen);

                count = atoi(countStr.c_str());
            }

            pos = commaIdx+1;

            int num = atoi(numberStr.c_str());

            for(count; count > 0; count--){
                intList.push_back(num);
            }
        }

    }


    return intList;
}


bool parseConfig(std::string configPath){
    printf("Parsing pixel map[%s]\n", configPath.c_str());

    dictionary* dict = iniparser_load(configPath.c_str());

    if(dict != NULL){
        int sections = iniparser_getnsec(dict);
        printf("Pixel map has %i sections\n", sections);

        for(int i=0; i<sections; i++){
            string nodeName = iniparser_getsecname(dict, i);
            printf("Section: %s\n", nodeName.c_str());

            if(nodeName == "general"){
                printf("Parsing general section\n");
                if(ledArray != NULL){
                    ROS_WARN("Duplicate general section!");
                    perror("Duplicate general section!\n");
                }
                else{
                    ledArray = new LedArray;
                }

                printf("Making keys\n");
                string layoutKey = string(nodeName).append(":layout");

                printf("Getting layout[%s]\n", layoutKey.c_str());
                string layoutStr = iniparser_getstring(dict, layoutKey.c_str(), string(""));

                if(layoutStr.size() < 1){
                    printf("No layout defined for entry [%s] in file [%s]\n", nodeName.c_str(), configPath.c_str());
                    return false;
                }

                printf("Layout [%s]\n", layoutStr.c_str());

                ledArray->run_lengths = parseIntList(layoutStr);

            }
            else if(nodeName.find("port") == 0){
                printf("Parsing port section\n");

                string startUnivKey = string(nodeName).append(":startUniv");
                string startAddrKey = string(nodeName).append(":startAddr");
                string endUnivKey = string(nodeName).append(":endUniv");
                string endAddrKey = string(nodeName).append(":endAddr");
                string lengthKey = string(nodeName).append(":length");
                string autoAllocKey = string(nodeName).append(":auto_alloc");   //Automatic strand allocation


                int lengthExists = iniparser_find_entry(dict, (char*) lengthKey.c_str());
                int autoAllocExists = iniparser_find_entry(dict, (char*) autoAllocKey.c_str());


                if(lengthExists != 1){
                    ROS_ERROR("No length defined for port entry [%s] in file [%s]", nodeName.c_str(), configPath.c_str());
                    iniparser_freedict(dict);
                    return false;
                }


                LedPort* port = new LedPort;

                //Read maximum strand length
                port->maxLength = iniparser_getint(dict, lengthKey.c_str(), -1);

                //Read dmx auto allocate
                string autoAlloc = iniparser_getstring(dict, autoAllocKey, string("true"));
                if(autoAlloc == "true"){
                    port->autoAllocDmx = true;
                }
                else{
                    port->autoAllocDmx = false;
                }

                //Read start DMX address
                port->start.universe = iniparser_getint(dict, startUnivKey.c_str(), -1);
                port->start.offset= iniparser_getint(dict, startAddrKey.c_str(), -1);

                //Read end DMX address
                port->end.universe = iniparser_getint(dict, endUnivKey.c_str(), -1);
                port->end.offset= iniparser_getint(dict, endAddrKey.c_str(), -1);

                /*  NOTE
                 *  CONVERT FROM ONE BASED TO ZERO BASED DMX ADDRESSES
                 */

                port->start.offset--;
                port->end.offset--;

                ledArray->ports.push_back(port);
            }

        }
    }

    iniparser_freedict(dict);

    return true;
}


/*
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

            //Validate mapping coverage
            int imagePixels = _pixelMap.array_size[0] * _pixelMap.array_size[1];
            int dmxPixels = 0;

            map<int, vector<pixel_map_config::dmx_map> >::iterator univIter =  _pixelMap.universes.begin();
            for(; univIter != _pixelMap.universes.end(); univIter++){
                vector<pixel_map_config::dmx_map>::iterator dmxIter = univIter->second.begin();

                for(; dmxIter != univIter->second.end(); dmxIter++){
                    dmxPixels += dmxIter->pixels;
                }
            }

            if(imagePixels < dmxPixels){
                ROS_WARN("More DMX pixels[%i] mapped than exist in image[%i], this could be an error or simply an non-optimal mapping", dmxPixels, imagePixels);
            }
            else if(imagePixels > dmxPixels){
                ROS_WARN("Not all image pixels[%i] mapped into DMX pixels[%i]", imagePixels, dmxPixels);
            }

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
*/
