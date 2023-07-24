
//includes
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>


#include <yaml-cpp/yaml.h>
#include <fstream>


#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "fbgs-sensing/shape_sensing_interface.h"

#include "utilities/Eigen/eigen_io.hpp"

#include <real_time_tools/spinner.hpp>
#include <real_time_tools/thread.hpp>

//#include "matplotlibcpp.h"

//defines
#define SERVER_ADDRESS "192.168.1.11"
#define PORT_NUMBER "5001" //Default port number for Shape Sensing


std::atomic_bool StopDemos(false);


void my_handler(int)
{
    StopDemos = true;
}


bool start_recording = true;

#include <thread>



// The function we want to execute on the new thread.
void wait_input()
{
    std::cout << "input to continue \n\n\n\n";
    std::cout.flush();
    getchar();
    std::cout << "ok \n\n\n\n";
    std::cout.flush();

    start_recording = true;
}





int main(int, char **)
{
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopDemos = false;


    //Example code for Shape Sensing Interface

    ShapeSensingInterface interface(SERVER_ADDRESS,PORT_NUMBER);

    ShapeSensingInterface::Sample sample;
    std::vector<ShapeSensingInterface::Sample> samples_stack;

    if(!interface.connect())
        return 0;




    //    std::chrono::high_resolution_clock::time_point current;
    //    double current_time;
    //    double delta_time;
    //    double previus_iteration_end_time;

    const double sensor_reading_frequency = 100.0f;  //  Hz
    const double dt = 1.0f/sensor_reading_frequency;
    const double recording_time = 1; //    s

    real_time_tools::Spinner spinner;
    double spinner_reading_frequency = 20.0*sensor_reading_frequency;  //  Hz
    double dt_spinner = 1.0f/spinner_reading_frequency;
    spinner.set_period( dt_spinner );


    int count = 0;
    const int max_count = static_cast<unsigned int>(recording_time/dt);
    samples_stack.resize(max_count);


    //    std::thread thread(wait_input);
    //    thread.join();




    //    current = std::chrono::high_resolution_clock::now();
    //    elapsed = current - start;
    std::chrono::duration<double> elapsed;
    double elapsed_ms = 0;
    std::chrono::high_resolution_clock::time_point current;
    std::chrono::high_resolution_clock::time_point previous;


    std::cout << "input to continue \n\n\n\n";
    std::cout.flush();
    getchar();
    std::cout << "ok \n\n\n\n";
    std::cout.flush();

    //  Dump old samples

    unsigned int dumped = 0;
    while(interface.nextSampleReady()){
        interface.readNextSample(sample);
        dumped++;
    }

    std::cout << "dumped : " << dumped << " samples." << std::endl;


    previous = std::chrono::high_resolution_clock::now();
    while(count<max_count && StopDemos == false)
    {
        current = std::chrono::high_resolution_clock::now();

        if(interface.nextSampleReady()){
            if(interface.readNextSample(sample)){
                samples_stack[count++] = sample;
                std::cout << "count : " << count << "\r";
                std::cout.flush();

            }
        }

        elapsed = current - previous;
        elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

        // local_time += m_dt;
        while(StopDemos == false and
               elapsed_ms < dt*1000){
            spinner.spin();


            current = std::chrono::high_resolution_clock::now();
            elapsed = current - previous;
            elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        }

        previous = current;
    }


    std::vector<int> steps;

    int previus = samples_stack[0].sample_number;

    for(unsigned int i=1; i<samples_stack.size(); i++){

        int diff = samples_stack[i].sample_number - previus;
        previus = samples_stack[i].sample_number;
        if(diff != 0){
            steps.push_back(diff);
            std::cout << "diff : " << diff << "\n";
        }
    }

    //    steps.erase(steps.begin(), steps.begin() + 10);

    //    // Set the size of output image to 1200x780 pixels
    //    matplotlibcpp::figure_size(1200, 780);
    //    // Plot line from given x and y data. Color is selected automatically.
    //    matplotlibcpp::plot<std::vector<int>>(steps);
    //    // Add graph title
    //    matplotlibcpp::title("Samples steps");

    //    matplotlibcpp::show();

    //    std::cout.flush();

    std::cout << "size : " << samples_stack.size() << std::endl;


    YAML::Node measurement_data;

    YAML::Node header;
    std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    header["date"] = std::ctime(&time);
    header["notes"] = "";
    measurement_data["header"] = header;

    YAML::Node measurements;
    measurements["number_of_snapshots"] = samples_stack.size();
    measurements["frequency"] = sensor_reading_frequency;
    measurements["number_of_sensors"] = samples_stack[0].num_sensors;


    measurement_data["measurements"] = measurements;

    YAML::Node FBGS;

    for(const auto& sample : samples_stack){


        YAML::Node data;

        data["sample_numb"] = sample.sample_number;
        //        data["time_stamp"] = sample.time_stamp;

        YAML::Node sensors;
        for(const auto& sensor : sample.sensors){
            YAML::Node shape;
            shape["arc_length"] = matrixToYamlNode(sensor.arc_length);
            shape["shape"] = matrixToYamlNode(sensor.shape);

            sensors.push_back(shape);
        }


        measurement_data["sensors"].push_back(sensors);

    }



    const std::string path = "data/" + std::to_string(static_cast<int>(sensor_reading_frequency)) + "Hz/two_sensors";
    const std::string name = "simulation_results_" + std::to_string(static_cast<int>(recording_time)) + "s.yaml";



    SaveFile(measurement_data, name, path);



    return 0;

}

