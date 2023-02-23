/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

#include "shape_sensing_interface.h"

ShapeSensingInterface::ShapeSensingInterface(std::string ip_address, std::string port_number) : 
	m_resolver(m_io_context), m_socket(m_io_context)
{
	m_ip_address = ip_address;
	m_port_number = port_number;
	m_connected = false;

}

ShapeSensingInterface::~ShapeSensingInterface()
{

}

bool ShapeSensingInterface::connect()
{
	m_connected = false;
	
	try
	{
		//Get endpoints
		boost::asio::ip::tcp::resolver::results_type endpoints = m_resolver.resolve(m_ip_address, m_port_number);
		
		boost::asio::ip::tcp::endpoint ep = *endpoints;
		std::cout << ep << std::endl;
		
		std::cout << "Establishing connection... "; 
		
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

bool ShapeSensingInterface::nextSampleReady()
{
	if(m_connected)
	{
		return (m_socket.available() > 0);
	}
	else
	{
		std::cout << "TCP not connected!" << std::endl;
		return false;
	}
	
}

bool ShapeSensingInterface::readNextSample(Sample &sample)
{
	if(!m_connected || m_socket.available() <= 0)
	{
		
		std::cout << "TCP not connected or no data available!" << std::endl;
		return false;
	}
	else
	{
		
		try
		{
			boost::asio::streambuf data;
				
			//First read the first 4 bytes to figure out the size of the following ASCII string
			size_t size_read = boost::asio::read(m_socket,data,boost::asio::transfer_exactly(4));
			
			std::string data_string;
			std::istream is(&data);
			is >> data_string;
			
			std::string hex_string = stringToHex(data_string);
			
			int data_length = std::stoi(hex_string,0,16);
			
			
			//Now read the remaining ASCII string of the current data package
			size_read = boost::asio::read(m_socket,data,boost::asio::transfer_exactly(data_length));
			
			std::cout << size_read << std::endl;
			
			data_string.clear();
			std::istream is2(&data);
			
			is2.peek(); // try state of stream
			while(is2.good())  
			{
				getline(is2,data_string,  '\t');     // FBGS uses tab as delimiter
				std::cout << data_string << std::endl;
				is2.peek(); // set eof flag if end of data is reached
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

std::string ShapeSensingInterface::stringToHex(const std::string& input)
{
    static const char hex_digits[] = "0123456789abcdef";

    std::string output;
    output.reserve(input.length() * 2);
    for (unsigned char c : input)
    {
        output.push_back(hex_digits[c >> 4]);
        output.push_back(hex_digits[c & 15]);
    }
    return output;
}
