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

#include <real_time_tools/spinner.hpp>
#include <real_time_tools/thread.hpp>

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
        Eigen::VectorXd kappa; //num_curv_points x 1 vector
        Eigen::VectorXd phi; //num_curv_points x 1 vector

        int num_shape_points;
        Eigen::MatrixXd shape; //num_shape_points x 3 matrix
        Eigen::VectorXd arc_length; //num_shape_points x 1 vector
    };

    //Structure
    struct Sample
    {


        int sample_number;
        int num_channels;
        int num_sensors;

        std::vector<Channel> channels;
        std::vector<Sensor> sensors;
    };





	// Constructor 
	ShapeSensingInterface(std::string ip_address, std::string port_number);

    ShapeSensingInterface(const std::string ip_address,
                          const std::string port_number,
                          const double t_recording_time,
                          const double t_frequency);
	
	// Simple destructor
	~ShapeSensingInterface();
	

	
	
	bool connect();
	bool nextSampleReady();
	bool readNextSample(Sample &sample);


    bool fetchDataFromTCPIP(const unsigned int index);

    Sample processDataAtIndex(const unsigned int index);


    void startRecordingLoop();




private:
	
	std::string m_ip_address;
	std::string m_port_number;
	bool m_connected;
			
	boost::asio::io_context m_io_context;
	boost::asio::ip::tcp::resolver m_resolver;
	boost::asio::ip::tcp::socket m_socket;



    real_time_tools::RealTimeThread m_rt_thread;
    static THREAD_FUNCTION_RETURN_TYPE m_loop(void* instance_pointer);
    void recordingLoop();
    bool m_stop_loop { false };
    real_time_tools::Spinner m_spinner;
    double m_recording_time;
    double m_frequency { 100 };
    double m_dt { 1.0f/m_frequency };

    std::vector<std::string> m_data_stack { std::vector<std::string>(static_cast<int>(m_recording_time / m_dt)) };


};

