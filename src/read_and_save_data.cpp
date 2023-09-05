
//includes
#include <iostream>
#include <vector>
#include <string>
#include <thread>

#include <Eigen/Dense>



#include "fbgs-sensing/shape_sensing_interface.h"

//#include "utilities/Eigen/eigen_io.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>


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



void checkPathAndCreateFolders(const std::filesystem::path& t_path);


std::string findCMakeLists(const std::filesystem::path& directory=std::filesystem::current_path());



YAML::Node matrixToYamlNode(const Eigen::MatrixXd &t_matrix);


void SaveFile(const YAML::Node t_file,
              const std::string t_file_name,
              std::string t_path,
              const bool t_use_project_root=true);



void writeToFile(std::string t_file_name,
                 const Eigen::MatrixXd &t_matrix,
                 std::string t_path,
                 const bool t_use_project_root=true,
                 const Eigen::IOFormat &t_format=Eigen::IOFormat(16, 0, ","));







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
                                    recording_frequency);


    if(!interface.connect())
        return 0;


    interface.startRecordinLoop();





//    for(int i=5; i>=0; i--){
//        std::cout << "Starting recording in : " << i << " s\r";
//        std::cout.flush();
//        std::this_thread::sleep_for(std::chrono::duration(std::chrono::seconds(1)));
//    }

    std::cout << "Input to start" << std::endl;
    getchar();

    *start_recording = true;
    std::cout << "\n\n";

    for(int i=30; i>=0; i--){
        std::cout << "Recording : " << i << " s\r";
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::duration(std::chrono::seconds(1)));
    }

    *StopDemos = true;
    *start_recording = false;



    std::cout << "\n\n\n\n\n\n" "Saving data    \n\n\n\n\n\n";
    std::cout.flush();



    YAML::Node FBGS_node;
    Eigen::MatrixXd FBGS_data;

    interface.getSamplesData(FBGS_node, FBGS_data);

    const std::string path = "data/" + std::to_string(static_cast<int>(recording_frequency)) + "Hz/long_measurements/";
    const std::string name = "simulation_results_" + std::to_string(static_cast<int>(recording_time)) + "s.yaml";



    SaveFile(FBGS_node, name, path);

    writeToFile("FBGS_data", FBGS_data, path);




    return 0;

}





void checkPathAndCreateFolders(const std::filesystem::path& t_path)
{
    std::filesystem::path existing_path = t_path;

    while(not std::filesystem::exists(existing_path))
        existing_path = existing_path.parent_path();

    auto path_to_create = std::filesystem::relative(t_path, existing_path);

    for(const auto& folder : path_to_create){
        existing_path = existing_path / folder;
        std::filesystem::create_directory(existing_path);
    }
}






std::string findCMakeLists(const std::filesystem::path& directory)
{

    std::filesystem::path cmakeListsPath = directory / "CMakeLists.txt";
    if (std::filesystem::exists(cmakeListsPath)) {
        return directory.string() + "/";
    } else {
        std::filesystem::path parentPath = directory.parent_path();
        if (!parentPath.empty()) {
            return findCMakeLists(parentPath);
        }
    }
    return "";
}



YAML::Node matrixToYamlNode(const Eigen::MatrixXd &t_matrix)
{
    // Create a YAML node
    YAML::Node node;
    node["matrix"]["storage"] = "colmajor";
    node["matrix"]["rows"] = static_cast<int>(t_matrix.rows());
    node["matrix"]["cols"] = static_cast<int>(t_matrix.cols());

    // Convert Eigen matrix to a nested sequence in YAML
    for (int col = 0; col < t_matrix.cols(); col++) {
        for (int row = 0; row < t_matrix.rows(); row++) {
            node["matrix"]["data"].push_back(t_matrix(row, col));
        }
    }

    return node;
}


void SaveFile(const YAML::Node t_file,
              const std::string t_file_name,
              std::string t_path,
              const bool t_use_project_root)
{
    if(not t_path.empty()){
        //  Ensure relative path ends with a backslash only if a path is given
        if(not t_path.ends_with('/'))
            t_path.append("/");
    }

    if(t_use_project_root)
        t_path = findCMakeLists() + t_path;


    //  Ensure that path exists otherwise create it
    checkPathAndCreateFolders(t_path);
    //    if(not std::filesystem::exists(t_path))
    //        std::filesystem::create_directory(t_path);




    //  The file will be created in the location given by the realtive path and with the given name
    const auto file_name_and_location = t_path + t_file_name;


    //  Create file in given location with given name
    std::ofstream file(file_name_and_location.c_str());

    //  Put matrix in this file
    file << t_file;

    //  Close the file
    file.close();
}




void writeToFile(std::string t_file_name,
                 const Eigen::MatrixXd &t_matrix,
                 std::string t_path,
                 const bool t_use_project_root,
                 const Eigen::IOFormat &t_format)
{
    if(not t_path.empty()){
        //  Ensure relative path ends with a backslash only if a path is given
        if(not t_path.ends_with('/'))
            t_path.append("/");
    }

    if(t_use_project_root)
        t_path = findCMakeLists() + t_path;


    //  Ensure that path exists otherwise create it
    checkPathAndCreateFolders(t_path);



    //  Ensure it ends with .csv
    if(t_file_name.find(".csv") == std::string::npos)
        t_file_name.append(".csv");

    //  The file will be created in the location given by the realtive path and with the given name
    const auto file_name_and_location = t_path + t_file_name;

    std::filesystem::path output_path(file_name_and_location);
    std::filesystem::create_directories(output_path.parent_path());

    //  Create file in given location with given name
    std::ofstream file(file_name_and_location.c_str());

    //  Put matrix in this file
    file << t_matrix.format(t_format);

    //  Close the file
    file.close();
}




