/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

#pragma once


#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <memory>

#include <real_time_tools/timer.hpp>
#include <real_time_tools/spinner.hpp>
#include <real_time_tools/thread.hpp>
#include <time_series/time_series.hpp>

#include <mutex>

#include <deque>


#include <yaml-cpp/yaml.h>

// This class implements a simple interface to the FBGS sensing system utilizing TCP sockets
class ShapeSensingInterface
{
public:





    struct Channel
    {
        int channel_number;
        int num_gratings;
        Eigen::Vector4i error_status;
        Eigen::VectorXd	peak_wavelengths; //num_gratings x 1 vector
        Eigen::VectorXd	peak_powers; //num_gratings x 1 vector
    };

    struct Sensor
    {
        int num_curv_points;
        Eigen::VectorXd kappa;  //num_curv_points x 1 vector
        Eigen::VectorXd phi;    //num_curv_points x 1 vector

        int num_shape_points;
        Eigen::MatrixXd shape; //num_shape_points x 3 matrix
        Eigen::VectorXd arc_length; //num_shape_points x 1 vector
    };

    //Structure
    struct Sample
    {


        int sample_number;
        std::chrono::high_resolution_clock::time_point time_stamp;
        int num_channels;
        int num_sensors;

        std::vector<Channel> channels;
        std::vector<Sensor> sensors;
    };





//    // Constructor
//    ShapeSensingInterface(std::string ip_address, std::string port_number);

    ShapeSensingInterface(std::shared_ptr<const bool> t_stop_demos,
                          std::shared_ptr<const bool> t_start_recording,
                          const double t_frequency=100);
	
	// Simple destructor
	~ShapeSensingInterface();


	

	
	
	bool connect();
    bool nextSampleReady();
    bool readNextSample(Sample &sample);

    void extracted(Sample const &sample, Eigen::VectorXd &sample_data,
                   unsigned int &index) const;
    Eigen::MatrixXd getDataAsEigenMatrix() const;


    void getSamplesData(YAML::Node &t_FBGS_node, Eigen::MatrixXd &t_FBGS_data)const;


    void startRecordinLoop()
    {
        thread = std::thread([&](){recordingLoop();});

    }



    //    bool fetchDataFromTCPIP(unsigned int &index);

    //    Sample processDataAtIndex(const unsigned int index);

    //    bool fetchDataFromTCPIP();

    void recordingLoop();
    // private:

    int m_size;


    std::string m_ip_address { "192.168.1.11" };
    std::string m_port_number { "5001" };
	bool m_connected;
			
	boost::asio::io_context m_io_context;
	boost::asio::ip::tcp::resolver m_resolver;
	boost::asio::ip::tcp::socket m_socket;





    double m_frequency { 100 };




    std::shared_ptr<const bool> m_stop_demos { nullptr };

    std::shared_ptr<const bool> m_start_recording { nullptr };



    std::thread thread;
    std::chrono::high_resolution_clock::time_point m_start;

    std::deque<Sample> m_samples_stack;


};

