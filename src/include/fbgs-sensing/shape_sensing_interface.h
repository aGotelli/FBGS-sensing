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
	
	// Simple destructor
	~ShapeSensingInterface();
	

	
	
	bool connect();
	bool nextSampleReady();
	bool readNextSample(Sample &sample);
	

private:
	
	std::string m_ip_address;
	std::string m_port_number;
	bool m_connected;
			
	boost::asio::io_context m_io_context;
	boost::asio::ip::tcp::resolver m_resolver;
	boost::asio::ip::tcp::socket m_socket;


};

