#include <stdlib.h>
#include <iostream>
#include <unistd.h>

#include <ola/DmxBuffer.h>
#include <ola/Logging.h>
#include <ola/StreamingClient.h>

using namespace std;


int main(int argc, char *argv[]) {
    unsigned int universe = 1;  // universe to use for sending data
    unsigned int i;

    // turn on OLA logging
    ola::InitLogging(ola::OLA_LOG_WARN, ola::OLA_LOG_STDERR);

    // Create a new DmxBuffer to hold the data
    ola::DmxBuffer buffer;
    // set all channels to 0
    buffer.Blackout();

    // create a new client and set the Error Closure
    ola::StreamingClient ola_client;

    // Setup the client, this connects to the server
    if (!ola_client.Setup()) {
        cout << "Setup failed" << endl;
        exit(1);
    }



    while(1){
        // send the data to the ola server
        for(int j=0; j<512; j++){
            for (i = 0; i < 100; i+=2) {
                buffer.SetChannel(j, i);

                for(int k=0; k<7; k++){
                    if (!ola_client.SendDmx(universe+k, buffer)) {
                        cerr << "Send DMX failed" << endl;
                    }
                    usleep(30);
                }
                usleep(1000);   // sleep for 20ms between updates
            }
        }

        // send the data to the ola server
        for(int j=511; j>0; j--){
            for (i = 100; i > 0; i-=2) {
                buffer.SetChannel(j, i);

                for(int k=0; k<7; k++){
                    if (!ola_client.SendDmx(universe+k, buffer)) {
                        cerr << "Send DMX failed" << endl;
                    }
                    usleep(100);
                }
                usleep(1000);   // sleep for 20ms between updates
            }
        }


    }

    // close the connection
    ola_client.Stop();
    return 0;
}

