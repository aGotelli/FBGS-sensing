/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

#include "fbgs-sensing/illumisense_interface.h"

IllumiSenseInterface::IllumiSenseInterface(std::string ip_address, std::string port_number, std::string calib_file_path) : 
	m_resolver(m_io_context), m_socket(m_io_context)
{
	m_ip_address = ip_address;
	m_port_number = port_number;
	m_connected = false;
	
	std::ifstream calib_file(calib_file_path);
	std::string line;
	if (calib_file.is_open())
	{
		// Get line 12
		for(int i = 0; i < 12; i++)
		{
			getline(calib_file,line);
		}
		
		std::string delimiter = "\t";
		
		int pos = line.find(delimiter);
		line.erase(0,pos + delimiter.length());

		
		m_radius = std::stod(line)*1e-6;
		
		calib_file.close();
		
	}
	
}

IllumiSenseInterface::~IllumiSenseInterface()
{

}

bool IllumiSenseInterface::connect()
{
	m_connected = false;
	
	try
	{
		//Get endpoints
		
		std::cout << "[FBGS] Get endpoints... "; 
		boost::asio::ip::tcp::resolver::results_type endpoints = m_resolver.resolve(m_ip_address, m_port_number);
		
		boost::asio::ip::tcp::endpoint ep = *endpoints;
		
		std::cout << "Establishing connection to " << ep << "... "; 
		
		//Connect to socket and open connection
		boost::asio::connect(m_socket, endpoints);
		
		std::cout << "Connected!" << std::endl << std::endl;
		
		m_connected = true;
		
		return true;	
				
	}
	catch(std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return false;
	}
	
	
}

bool IllumiSenseInterface::nextSampleReady()
{
	
	if(m_connected)
	{
		return (m_socket.available() >= 4);
	}
	else
	{
		std::cout << "[FBGS] TCP not connected!" << std::endl;
		return false;
	}
	
}

bool IllumiSenseInterface::readNextSample(Sample &sample)
{
	if(!m_connected)
	{
		
		std::cout << "[FBGS] TCP not connected!" << std::endl;
		return false;
	}
	else if(m_socket.available() < 4)
	{
		std::cout << "[FBGS] Data not ready!" << std::endl;
		return false;
	}	
	else
	{
		
		try
		{
			char buffer[4];
			//First read the first 4 bytes to figure out the size of the following ASCII string
			size_t size_read = boost::asio::read(m_socket,boost::asio::buffer(&buffer,4));
			
			//Convert the received bytes to signed integer
			int size = int((unsigned char)(buffer[0]) << 24 |
            (unsigned char)(buffer[1]) << 16 |
            (unsigned char)(buffer[2]) << 8 |
            (unsigned char)(buffer[3]));
            
			
			boost::asio::streambuf data;
			//Now read the remaining ASCII string of the current data package
			size_read = boost::asio::read(m_socket,data,boost::asio::transfer_exactly(size));
			
			std::string data_string;
			std::istream is(&data);
			
			//Create new, empty sample and set the passed sample to it
			Sample new_sample;
			sample = new_sample;
			
			//Skip first two entries (date and time)
			getline(is,data_string, '\t'); 
			getline(is,data_string, '\t'); 
			
			//Next string is the sample number
			getline(is,data_string, '\t'); 
			sample.sample_number = std::stoi(data_string);
			
			//Next string is number of channels
			getline(is,data_string, '\t'); 
			sample.num_channels = std::stoi(data_string);
			
			//Now we run through all channels
			for(int i = 0; i < sample.num_channels; i++)
			{
				IllumiSenseInterface::Sample::Channel channel;
				
				//Next string is channel number
				getline(is,data_string, '\t'); 
				channel.channel_number = std::stoi(data_string);
				
				//Next string is number of gratings
				getline(is,data_string, '\t'); 
				channel.num_gratings = std::stoi(data_string);
				
				//Next is error status
				getline(is,data_string, '\t'); 
				channel.error_status(0) = std::stoi(data_string);
				getline(is,data_string, '\t'); 
				channel.error_status(1) = std::stoi(data_string);
				getline(is,data_string, '\t'); 
				channel.error_status(2) = std::stoi(data_string);
				getline(is,data_string, '\t'); 
				channel.error_status(3) = std::stoi(data_string);
				
				//Next is peak wavelengths
				channel.peak_wavelengths.resize(channel.num_gratings);
				for(int j = 0; j < channel.num_gratings; j++)
				{
					getline(is,data_string, '\t'); 
					channel.peak_wavelengths(j) = std::stod(data_string);
				}
				
				//Next is peak powers
				channel.peak_powers.resize(channel.num_gratings);
				for(int j = 0; j < channel.num_gratings; j++)
				{
					getline(is,data_string, '\t'); 
					channel.peak_powers(j) = std::stod(data_string);
				}
				
				//Resize the vector of strains for later
				channel.strains.resize(channel.num_gratings);
				
				sample.channels.push_back(channel);
				
			}
			
			
			//Next is the number of engineered values (strain in our case)
			//We skip this value since this is the same as the number of all gratings over all channels
			getline(is,data_string, '\t'); 
			
			
						
			for(int i = 0; i < sample.num_channels; i++)
			{
				for(int j = 0; j < sample.channels.at(i).num_gratings; j++)
				{
					getline(is,data_string, '\t'); 
					sample.channels.at(i).strains(j) = std::stod(data_string);
				}
			}
			
			//Create sensor data
						
			//Check the number of sensors
			//If we have four channels, each with the same number of gratings, we have one sensor
			//If we have eight channels, and the first four as well as the last four have the same number of gratings, we have two sensors
			//This is the maximum the hardware in the lab can currently support
			sample.num_sensors = 0;
			if(sample.num_channels == 4 && sample.channels.at(0).num_gratings == sample.channels.at(1).num_gratings && sample.channels.at(0).num_gratings == sample.channels.at(2).num_gratings && sample.channels.at(0).num_gratings == sample.channels.at(3).num_gratings)
			{
				sample.num_sensors = 1;
			}
			else if(sample.num_channels == 8 && sample.channels.at(0).num_gratings == sample.channels.at(1).num_gratings && sample.channels.at(0).num_gratings == sample.channels.at(2).num_gratings && sample.channels.at(0).num_gratings == sample.channels.at(3).num_gratings && sample.channels.at(4).num_gratings == sample.channels.at(5).num_gratings && sample.channels.at(4).num_gratings == sample.channels.at(6).num_gratings && sample.channels.at(4).num_gratings == sample.channels.at(7).num_gratings)
			{
				sample.num_sensors = 2;
			}
			
			
			//For each sensor, computing the curvature strain of the multicore fiber at each grating position
			//Throughout this code, we are following the methods derived in:
			//Modes, Ortmaier, Burgner-Kahrs:
			//Shape Sensing Based on Longitudinal Strain Measurements Considering Elongation, Bending, and Twisting
			//IEEE Sensors Journal, 2020
			for(int i = 0; i < sample.num_sensors; i++)
			{
				IllumiSenseInterface::Sample::Sensor sensor;
                IllumiSenseInterface::Sample::Channel c1 = sample.channels.at(0 + 4*i);
                IllumiSenseInterface::Sample::Channel c2 = sample.channels.at(1 + 4*i);
                IllumiSenseInterface::Sample::Channel c3 = sample.channels.at(3 + 4*i);
                IllumiSenseInterface::Sample::Channel c4 = sample.channels.at(2 + 4*i);
                
				sensor.num_curv_points = sample.channels.at(0 + 4*i).num_gratings;
				sensor.curvature_strains.resize(sensor.num_curv_points,6);
				sensor.curvature_strains.setZero();
				
				for(int j = 0; j < sensor.num_curv_points; j++)
                {
                    // Channel 0: Core center
                    // Channel 1: 0,0,r
                    // Channel 2: 0, sin(2pi/3), cos(2pi/3)
                    // Channel 3: 0, -sin(2pi/3), cos(2pi/3)
                    // Assumption: backbone along x-axis

                    // Get strain measurement from central channel (assumed to result from pure elongation)
                    double epsilon_a = c1.strains(j)*1e-6;
                    double E_a = 1 + epsilon_a;

                    //Get strain measurements of each outer channel
                    double epsilon_1 = c2.strains(j)*1e-6;
                    double epsilon_2 = c3.strains(j)*1e-6;
                    double epsilon_3 = c4.strains(j)*1e-6;
                    double E_1 = 1 + epsilon_1;
                    double E_2 = 1 + epsilon_2;
                    double E_3 = 1 + epsilon_3;

                    //Define angle of each channel
                    double theta_1 = 0 + M_PI/2;
                    double theta_2 = 2*M_PI/3 + M_PI/2;
                    double theta_3 = 4*M_PI/3 + M_PI/2;

                    //Define helper variables
                    double s12 = std::sin(theta_1-theta_2);
                    double s23 = std::sin(theta_2-theta_3);
                    double s31 = std::sin(theta_3-theta_1);

                    //Equation (21) of Vincent's paper
                    double Q = (epsilon_1*s23+epsilon_2*s31+epsilon_3*s12)/(s23+s31+s12);

                    //Equation (23) of Vincent's paper to calculate absolute value of twist
                    double tmp_1 = std:: sqrt((E_a/2)*(E_a/2)-Q);
                    double tmp_2 = -tmp_1;


                    double twist_abs_1 = (1.0/m_radius)*std::sqrt(1-(E_a/2 + tmp_1)*(E_a/2 + tmp_1));
                    double twist_abs_2 = (1.0/m_radius)*std::sqrt(1-(E_a/2 + tmp_2)*(E_a/2 + tmp_2));
                    
                    if(abs(tmp_1) - 0.5 < 1e-4 &&  abs(tmp_2) - 0.5 < 1e-4)
                    {
						twist_abs_1 = 0;
						twist_abs_2 = 0;
					}
                    
                    std::cout << twist_abs_1 << std::endl;
                    std::cout << twist_abs_2 << std::endl << std::endl;

                    //Equation (13) of Vincent's paper to calculate bending angle
                    double s1 = std::sin(theta_1);
                    double s2 = std::sin(theta_2);
                    double s3 = std::sin(theta_3);
                    double c1 = std::cos(theta_1);
                    double c2 = std::cos(theta_2);
                    double c3 = std::cos(theta_3);

                    //Check first theta_abs value

                    double A1 = std::sqrt(E_1*E_1 - (m_radius*twist_abs_1)*(m_radius*twist_abs_1)) - E_a;
                    double A2 = std::sqrt(E_2*E_2 - (m_radius*twist_abs_1)*(m_radius*twist_abs_1)) - E_a;
                    double A3 = std::sqrt(E_3*E_3 - (m_radius*twist_abs_1)*(m_radius*twist_abs_1)) - E_a;

                    double theta_b_1 = std::atan2((A1*c2-A2*c1),(A2*s1-A1*s2));
                    double theta_b_2 = std::atan2((A1*c3-A3*c1),(A3*s1-A1*s3));
                    double theta_b_3 = std::atan2((A2*c3-A3*c2),(A3*s2-A2*s3));

                    double theta_b = theta_b_1;
                    double twist = twist_abs_1;

                    if((std::abs(theta_b_1 - theta_b_2) > 1e-4 && std::abs(theta_b_1 - theta_b_2) - 2*M_PI > 1e-4) || (std::abs(theta_b_1 - theta_b_3) > 1e-4 && std::abs(theta_b_1 - theta_b_3) - 2*M_PI > 1e-4) || std::isnan(theta_b_1) || std::isnan(theta_b_2) || std::isnan(theta_b_3))
                    {

                        A1 = std::sqrt(E_1*E_1 - (m_radius*twist_abs_2)*(m_radius*twist_abs_2)) - E_a;
                        A2 = std::sqrt(E_2*E_2 - (m_radius*twist_abs_2)*(m_radius*twist_abs_2)) - E_a;
                        A3 = std::sqrt(E_3*E_3 - (m_radius*twist_abs_2)*(m_radius*twist_abs_2)) - E_a;

                        theta_b_1 = std::atan2((A2*c3-A3*c2),(A3*s2-A2*s3));
                        theta_b_2 = std::atan2((A1*c3-A3*c1),(A3*s1-A1*s3));
                        theta_b_3 = std::atan2((A1*c2-A2*c1),(A2*s1-A1*s2));

                    if((std::abs(theta_b_1 - theta_b_2) > 1e-4 && std::abs(theta_b_1 - theta_b_2) - 2*M_PI > 1e-4) || (std::abs(theta_b_1 - theta_b_3) > 1e-4 && std::abs(theta_b_1 - theta_b_3) - 2*M_PI > 1e-4) || std::isnan(theta_b_1) || std::isnan(theta_b_2) || std::isnan(theta_b_3))
                    {
                        std::cout << "No valid twist value found" << std::endl;
                    }
                    else
                    {
                        theta_b = theta_b_1;
                        twist = twist_abs_2;
                    }
                    }


                    double kappa = 0;

                    //Use first channel to compute kappa if first channel not in rectifying plane
                    if(std::cos(theta_b - theta_1) > 1e-6)
                    {
                        kappa = -1.0/(m_radius*std::cos(theta_b - theta_1))*(std::sqrt(E_1*E_1-(m_radius*twist)*(m_radius*twist)) - E_a);

                    }
                    else // Use second channel otherwise
                    {
                        kappa = -1.0/(m_radius*std::cos(theta_b - theta_2))*(std::sqrt(E_2*E_2-(m_radius*twist)*(m_radius*twist)) - E_a);
                    }

                    //Convert kappa and theta to kappa_y and kappa_z
                    double kappa_y = kappa*std::cos(theta_b);
                    double kappa_z = kappa*std::sin(theta_b);

                    Eigen::Matrix<double,1,6> strain_values;

                    strain_values << twist, kappa_y, kappa_z, E_a, 0, 0;

                    sensor.curvature_strains.row(j) = strain_values;

                }
				
				sample.sensors.push_back(sensor);
			}
			
			return true;
					
		}
		catch(std::exception& e)
		{
			std::cerr << e.what() << std::endl;
			return false;
		}
		
	}
	
	
}

