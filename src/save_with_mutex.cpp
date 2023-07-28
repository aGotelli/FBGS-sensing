#include "fbgs-sensing/shape_sensing_interface.h"


#include "utilities/Eigen/eigen_io.hpp"

#include <yaml-cpp/yaml.h>

using namespace std::chrono;


#define SERVER_ADDRESS "192.168.1.11"
#define PORT_NUMBER "5001" //Default port number for Shape Sensing

std::shared_ptr<bool> start_recording =
    std::make_shared<bool>(false);



std::shared_ptr<bool> StopDemos =
    std::make_shared<bool>(false);




void my_handler(int)
{
    *StopDemos = true;
}


int main()
{

    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    *StopDemos = false;


    const double recording_time = 6;
    const double recording_frequency = 100;





    ShapeSensingInterface FBGS_interface(SERVER_ADDRESS,
                                         PORT_NUMBER,
                                         StopDemos,
                                         start_recording,
                                         recording_time,
                                         recording_frequency);

    if(!FBGS_interface.connect())
        return 0;


    FBGS_interface.startRecordinLoop();





    for(int i=5; i>=0; i--){
        std::cout << "Starting recording in : " << i << " s\r";
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::duration(std::chrono::seconds(1)));
    }
    std::cout << "\n\n\n";
    std::cout.flush();




    std::chrono::high_resolution_clock::time_point when_started;
    std::chrono::high_resolution_clock::time_point target_time;



    const unsigned int number_of_steps = static_cast<unsigned int>(recording_time*recording_frequency);



    std::vector<ShapeSensingInterface::Sample> FBGS_samples_stack(number_of_steps);
    unsigned int index = 0;

    when_started = std::chrono::high_resolution_clock::now();
    target_time = when_started + 10ms;
    unsigned int count = 0;
    unsigned int seconds = 0;
    while(not *StopDemos and
           index < number_of_steps) {

        FBGS_interface.getSample( FBGS_samples_stack[index++] );


        std::this_thread::sleep_until(target_time);
        target_time += 10ms;

        count ++;
        if(count==100){
            seconds ++;
            std::cout << "simulation time " << seconds << "\r";
            std::cout.flush();
            count =0;
        }

    }



    std::cout << "Finished to record data !!" << std::endl;











    std::cout << "\n\n\n\n\n\n" "Saving data    \n\n\n\n\n\n";
    std::cout.flush();



    YAML::Node FBGS_data;

    YAML::Node header;
    std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    header["date"] = std::ctime(&time);
    header["notes"] = "";
    FBGS_data["header"] = header;

    YAML::Node measurements;
    measurements["number_of_snapshots"] = FBGS_samples_stack.size();
    measurements["frequency"] = recording_frequency;
    measurements["number_of_sensors"] = FBGS_samples_stack[0].num_sensors;

    measurements["data_storage"] = "colmajor";


    YAML::Node order;
    order.push_back("sample_number");
    order.push_back("time_stamp");
    order.push_back("number_of_sensors");

    YAML::Node number_of_points_per_sensors;
    number_of_points_per_sensors.push_back("number_of_datapoints");

    order["number_of_points_per_sensors"] = number_of_points_per_sensors;


    YAML::Node sensors_data;
    sensors_data.push_back("arc_length_coordinates");
    sensors_data.push_back("x_positions");
    sensors_data.push_back("y_positions");
    sensors_data.push_back("z_positions");

    order["sensors_data"] = sensors_data;

    measurements["data_order"] = order;


    FBGS_data["measurements"] = measurements;

    std::cout << "\n\n\n\n\n\n" "Composed yaml    \n\n\n\n\n\n";
    std::cout.flush();


    const auto FBGS_samples_data = FBGS_interface.getDataAsEigenMatrix();


    std::cout << "\n\n\n\n\n\n" "Composed MatrixXd    \n\n\n\n\n\n";
    std::cout.flush();

    const std::string path = "data/" + std::to_string(static_cast<int>(recording_frequency)) + "Hz/mutex/";
    const std::string name = "simulation_results_" + std::to_string(static_cast<int>(recording_time)) + "s.yaml";



    SaveFile(FBGS_data, name, path);

    std::cout << "\n\n\n\n\n\n" "Saved yaml    \n\n\n\n\n\n";
    std::cout.flush();

    writeToFile("FBGS_data", FBGS_samples_data, path);

    std::cout << "\n\n\n\n\n\n" "Saved MatrixXd    \n\n\n\n\n\n";
    std::cout.flush();













    return 0;
}
