/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <boost/array.hpp>
#include <boost/asio.hpp>


#include <deque>

#include <memory>

#include <real_time_tools/timer.hpp>
#include <real_time_tools/spinner.hpp>
#include <real_time_tools/thread.hpp>
#include <time_series/time_series.hpp>

#include <yaml-cpp/yaml.h>

// This class implements a simple interface to the FBGS sensing system utilizing TCP sockets
class IllumiSenseInterface
{
public:
    // Constructor
    IllumiSenseInterface(std::shared_ptr<const bool> t_stop_demos,
                                               std::shared_ptr<const bool> t_start_recording,
                                               const double t_frequency);
	
	// Simple destructor
	~IllumiSenseInterface();
	
	//Structure
	struct Sample
	{
		struct Channel
		{
			int channel_number;
			int num_gratings;
			Eigen::Vector4i error_status; 
			Eigen::VectorXd	peak_wavelengths; //num_gratings x 1 vector
			Eigen::VectorXd	peak_powers; //num_gratings x 1 vector
			Eigen::VectorXd	strains; //num_gratings x 1 vector
		};
		
		
		int sample_number;
		int num_channels;


        int number_of_engineered_values;
        std::chrono::high_resolution_clock::time_point time_stamp;
		std::vector<Channel> channels;
	};
	
	
	bool connect();


    void startRecordinLoop();






    void getSamplesData(YAML::Node &t_FBGS_node, Eigen::MatrixXd &t_FBGS_data)const;
	

private:

    std::string m_ip_address { "192.168.1.11" };
    std::string m_port_number { "2055" };
	bool m_connected;
	double m_radius;
			
	boost::asio::io_context m_io_context;
	boost::asio::ip::tcp::resolver m_resolver;
	boost::asio::ip::tcp::socket m_socket;





    double m_frequency { 100 };




    std::shared_ptr<const bool> m_stop_demos { nullptr };

    std::shared_ptr<const bool> m_start_recording { nullptr };



    std::thread thread;
    std::chrono::high_resolution_clock::time_point m_start;



    std::deque<Sample> m_samples_stack;


    void recordingLoop();
    bool nextSampleReady();
    bool readNextSample(Sample &sample);


    void extracted(Sample const &sample,
                   Eigen::VectorXd &sample_data,
                   unsigned int &index) const;

};

