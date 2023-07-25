
//includes
#include <iostream>
#include <vector>
#include <string>
#include <thread>

#include <Eigen/Dense>



#include "fbgs-sensing/shape_sensing_interface.h"

#include "utilities/Eigen/eigen_io.hpp"




//#include "matplotlibcpp.h"

//defines
#define SERVER_ADDRESS "192.168.1.11"
#define PORT_NUMBER "5001" //Default port number for Shape Sensing


std::shared_ptr<bool> StopDemos =
    std::make_shared<bool>(false);




void my_handler(int)
{
    *StopDemos = true;
}




std::shared_ptr<bool> start_recording =
    std::make_shared<bool>(false);












int main(int, char **)
{
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    *StopDemos = false;




    //Example code for Shape Sensing Interface

    const double recording_time = 5;
    const double recording_frequency = 100;



    ShapeSensingInterface interface(SERVER_ADDRESS,
                                    PORT_NUMBER,
                                    StopDemos,
                                    start_recording,
                                    recording_time,
                                    recording_frequency);


    if(!interface.connect())
        return 0;


    interface.startRecordingLoop();



    for(int i=5; i>=0; i--){
        std::cout << "Starting recording in : " << i << " s\r";
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::duration(std::chrono::seconds(1)));
    }

    *start_recording = true;




//    YAML::Node measurement_data;

//    YAML::Node header;
//    std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
//    header["date"] = std::ctime(&time);
//    header["notes"] = "";
//    measurement_data["header"] = header;

//    YAML::Node measurements;
//    measurements["number_of_snapshots"] = samples_stack.size();
//    measurements["frequency"] = sensor_reading_frequency;
//    measurements["number_of_sensors"] = samples_stack[0].num_sensors;


//    measurement_data["measurements"] = measurements;

//    YAML::Node FBGS;

//    for(const auto& sample : samples_stack){


//        YAML::Node data;

//        data["sample_numb"] = sample.sample_number;
//        //        data["time_stamp"] = sample.time_stamp;

//        YAML::Node sensors;
//        for(const auto& sensor : sample.sensors){
//            YAML::Node shape;
//            shape["arc_length"] = matrixToYamlNode(sensor.arc_length);
//            shape["shape"] = matrixToYamlNode(sensor.shape);

//            sensors.push_back(shape);
//        }


//        measurement_data["sensors"].push_back(sensors);

//    }



//    const std::string path = "data/" + std::to_string(static_cast<int>(sensor_reading_frequency)) + "Hz/two_sensors";
//    const std::string name = "simulation_results_" + std::to_string(static_cast<int>(recording_time)) + "s.yaml";



//    SaveFile(measurement_data, name, path);



    return 0;

}

