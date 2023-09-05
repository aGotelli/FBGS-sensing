/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

#include "fbgs-sensing/illumisense_interface.h"
#include <yaml-cpp/node/node.h>

IllumiSenseInterface::IllumiSenseInterface(std::shared_ptr<const bool> t_stop_demos,
                                            std::shared_ptr<const bool> t_start_recording,
                                            const double t_frequency) :
    m_resolver(m_io_context),
    m_socket(m_io_context),
    m_frequency(t_frequency),
    m_stop_demos( t_stop_demos ),
    m_start_recording( t_start_recording )
{
    m_connected = false;


    m_samples_stack.clear();
}

IllumiSenseInterface::~IllumiSenseInterface()
{
    m_socket.close();

    thread.join();
}

bool IllumiSenseInterface::connect()
{
    m_connected = false;

    try
    {
        //Get endpoints
        std::cout << "[FBGS] Get endpoints... \n";
        boost::asio::ip::tcp::resolver::results_type endpoints = m_resolver.resolve(m_ip_address, m_port_number);

        boost::asio::ip::tcp::endpoint ep = *endpoints;

        std::cout << "Establishing connection to " << ep << "... \n";

        //Connect to socket and open connection
        boost::asio::connect(m_socket, endpoints);

        std::cout << "\n\n\n            Connected!\n\n\n" << std::endl << std::endl;


        real_time_tools::Spinner spinner;
        double dt_spinner = 0.15;
        spinner.set_period( dt_spinner );

        //   wait next sample to be ready
        static int pos=0;
        char cursor[4]={'/','-','\\','|'};
        while(m_socket.available() < 4
               and not *m_stop_demos){
            std::cout << "Waiting for ILLumiSense data stream...  " << cursor[pos] << "\r";
            pos = (pos+1) % 4;
            std::cout.flush();
            spinner.spin();
        }

        m_connected = true;


        std::cout << "\n\n\n            Reading the stream!\n\n\n" << std::endl << std::endl;


        return true;

    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }


    return true;
	
}



void IllumiSenseInterface::startRecordinLoop()
{
    thread = std::thread([&](){recordingLoop();});

}



void IllumiSenseInterface::recordingLoop()
{
    Sample sample;

    unsigned int dumped = 0;
    while(nextSampleReady()){
        readNextSample(sample);
        dumped++;
    }

    std::cout << "dumped : " << dumped << " samples before starting the recording loop." << std::endl;



    m_start = std::chrono::high_resolution_clock::now();
    while(not *m_stop_demos){

        if(nextSampleReady()){

            readNextSample(sample);

            if(*m_start_recording){
                m_samples_stack.push_back( sample );

            }
        }
    }
}








bool IllumiSenseInterface::nextSampleReady()
{
	
    return (m_socket.available() >= 4);

}

bool IllumiSenseInterface::readNextSample(Sample &sample)
{
    if((m_socket.available() < 4))
        return false;

		
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

        sample.time_stamp = std::chrono::high_resolution_clock::now();

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
        sample.number_of_engineered_values = std::stod(data_string);


        for(int i = 0; i < sample.num_channels; i++)
        {
            for(int j = 0; j < sample.channels.at(i).num_gratings; j++)
            {
                getline(is,data_string, '\t');
                sample.channels.at(i).strains(j) = std::stod(data_string);
            }
        }


        return true;

    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }

}





void IllumiSenseInterface::getSamplesData(YAML::Node &t_FBGS_node, Eigen::MatrixXd &t_FBGS_data)const
{

    /*
    data_order:
        0: sample_number                    int 1x1
        1: time_stamp                       double 1x1
        2: quantity_of_optical_lines        int 1x1
        3: number of FBG's per channel      vector
          - number of FBG's (or gratings)   int 1x1 -> N
        3: optical_lines:                   vector
          - Optical line number             int 1x1
          - System status A                 int 1x1
          - System status B                 int 1x1
          - System status C                 int 1x1
          - System status D                 int 1x1
          - Peak wavelengths                int Nx1
          - Peak Powers                     int Nx1
          - Strains                         int Nx1
     */


    unsigned int header_block_row = 3 + m_samples_stack[0].num_channels;
    unsigned int data_block_row = 0;

    for(const auto& channel : m_samples_stack[0].channels){
        unsigned int num_gratings = channel.num_gratings;

        //  Add rows for the first element of every channel data
        data_block_row += 5;

        // Add rows for peak powers, wavelengths and strain
        data_block_row += num_gratings*3;
    }

    unsigned int number_of_columns = m_samples_stack.size();
    unsigned int number_of_rows = header_block_row + data_block_row;

    t_FBGS_data = Eigen::MatrixXd(number_of_rows, number_of_columns);






    std::chrono::high_resolution_clock::duration time_since_start;
    for(unsigned int col=0; const auto& sample : m_samples_stack){

        unsigned int start_row = 0;

        time_since_start = sample.time_stamp - m_start;
        Eigen::VectorXd sample_data = Eigen::VectorXd::Zero(number_of_rows);

        //        std::cout << "sample number : " << sample.sample_number << "\n";
//        std::cout.flush();

        sample_data.block(start_row, 0, 3, 1) << sample.sample_number,
            time_since_start.count() / 1e9,
            sample.num_channels;
        start_row += 3;

        for(const auto& channel : sample.channels)
            sample_data(start_row++, 0) = channel.num_gratings;



        for(const auto& channel : sample.channels){
            unsigned int channel_rows = 0;

            channel_rows++; //  channel number
            channel_rows++; //  Error A
            channel_rows++; //  Error B
            channel_rows++; //  Error C
            channel_rows++; //  Error D



            unsigned int num_gratings = channel.num_gratings;

            channel_rows += num_gratings;   //  peaks wavelengths
            channel_rows += num_gratings;   //  peaks powers
            channel_rows += num_gratings;   //  strains


            sample_data.block(start_row, 0, channel_rows, 1) << channel.channel_number,
                channel.error_status.cast<double>(),
                channel.peak_wavelengths,
                channel.peak_powers,
                channel.strains;

            start_row += channel_rows;

        }


        t_FBGS_data.col(col++) = sample_data;
    }






    t_FBGS_node["number_of_snapshots"] = m_samples_stack.size();
    t_FBGS_node["frequency"] = m_frequency;
    t_FBGS_node["duration"] = std::chrono::duration<double>(m_samples_stack[m_samples_stack.size()-1].time_stamp - m_start).count();
    t_FBGS_node["number_of_channels"] = m_samples_stack[0].num_channels;

    t_FBGS_node["data_storage"] = "colmajor";


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

    t_FBGS_node["data_order"] = order;

}

	

