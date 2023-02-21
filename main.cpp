/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

//includes
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>


#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "shape_sensing_interface.h"

//defines
#define SERVER_ADDRESS "192.168.1.11"
#define PORT_NUMBER "5001"

std::string string_to_hex(const std::string& input)
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

int main(int argc, char **argv)
{
	
	try
	{
			
		//Set up variables
		boost::asio::io_context io_context;
		boost::asio::ip::tcp::resolver resolver(io_context);
		
		//Get endpoints
		boost::asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(SERVER_ADDRESS, PORT_NUMBER);
		
		boost::asio::ip::tcp::endpoint ep = *endpoints;
		std::cout << ep << std::endl;
		
		std::cout << "Establishing connection... "; 
		
		//Connect to socket and open connection
		boost::asio::ip::tcp::socket socket(io_context);
		boost::asio::connect(socket, endpoints);
		
		std::cout << "Connected!" << std::endl << std::endl;

		
		
		//Run in an endless loop and display received data
		while(1)
		{
			//boost::array<char, 128> buf;
			//boost::system::error_code error;
			//
			//size_t len = socket.read_some(boost::asio::buffer(buf), error);
			//
			//if (error == boost::asio::error::eof)
			//{
			//	std::cout << "Connection closed by server." << std::endl << std::endl;
			//	break; // Connection closed cleanly by peer.
			//}
			//else if (error)
			//{
			//	std::cout << "Error occured!" << std::endl << std::endl;
			//	throw boost::system::system_error(error); // Some other error.
			//}
			//
			//std::cout << buf.data() << std::endl << std::endl;
			
			//Check if we can read data without blocking
			if(socket.available() > 0)
			{
			
				boost::asio::streambuf data;
				
				//First read the first 4 bytes to figure out the size of the following ASCII string
				size_t size_read = boost::asio::read(socket,data,boost::asio::transfer_exactly(4));
				
				std::cout << size_read << std::endl;
				
				std::string data_string;
				std::istream is(&data);
				is >> data_string;
				
				std::string hex_string = string_to_hex(data_string);
				
				std::cout << hex_string << std::endl;
				int data_length = std::stoi(hex_string,0,16);
				std::cout << data_length << std::endl;
			
				
				//Now read the remaining ASCII string of the current data package
				size_read = boost::asio::read(socket,data,boost::asio::transfer_exactly(data_length));
				
				std::cout << size_read << std::endl;
				
				data_string.clear();
				std::istream is2(&data);;
				
				is2.peek(); // try state of stream
				while(is2.good())  
				{
					getline(is2,data_string,  '\t');     // FBGS uses tab as delimiter
					std::cout << data_string << std::endl;
					is2.peek(); // set eof flag if end of data is reached
				}
				
				break;
			}
			
			
    
		}
				
	}
	catch(std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}			
	return 0;
}

