/*
This code implements an interface to the FBGS sensing system
Copyright (C) 2022 Sven Lilge, Continuum Robotics Laboratory, University of Toronto Mississauga
*/

#include "fbgs-sensing/shape_sensing_interface.h"





//ShapeSensingInterface::ShapeSensingInterface(std::string ip_address,
//                                             std::string port_number) :
//    m_resolver(m_io_context), m_socket(m_io_context)
//{
//    m_ip_address = ip_address;
//    m_port_number = port_number;
//    m_connected = false;

//}


ShapeSensingInterface::ShapeSensingInterface(const std::string ip_address,
                        const std::string port_number,
                        std::shared_ptr<const bool> t_stop_demos,
                        std::shared_ptr<const bool> t_start_recording,
                        const double t_frequency) :
    m_resolver(m_io_context),
    m_socket(m_io_context),
    m_frequency(t_frequency),
    m_stop_demos( t_stop_demos ),
    m_start_recording( t_start_recording )
{
    m_ip_address = ip_address;
    m_port_number = port_number;
    m_connected = false;


    m_samples_stack.clear();
}

ShapeSensingInterface::~ShapeSensingInterface()
{
    m_socket.close();

    thread.join();
}

bool ShapeSensingInterface::connect()
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
            std::cout << "Waiting for Shape Sensing data stream...  " << cursor[pos] << "\r";
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


bool ShapeSensingInterface::nextSampleReady()
{

    return (m_socket.available() >= 4);

	
}








void ShapeSensingInterface::recordingLoop()
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

//                std::cout << "Sample number : " << m_samples_stack[m_samples_stack.size()-1].sample_number << "\r";
//                std::cout.flush();
            }

        }


    }
}




bool ShapeSensingInterface::readNextSample(Sample &sample)
{

    if((m_socket.available() >= 4)) {
        try {
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
                ShapeSensingInterface::Channel channel;

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

                sample.channels.push_back(channel);

            }


            //Now run through the file to the end
            getline(is,data_string, '\t');
            int num_sensors = 0;
            while(data_string == "Curvature [1/cm]")
            {
                ShapeSensingInterface::Sensor sensor;
                sensor.num_curv_points = sample.channels.at(0 + 4*num_sensors).num_gratings;

                //Save kappa (curvature) values
                sensor.kappa.resize(sensor.num_curv_points);
                for(int j = 0; j < sensor.num_curv_points; j++)
                {
                    getline(is,data_string, '\t');
                    sensor.kappa(j) = 100*std::stod(data_string); //convert 1/cm to 1/m
                }

                //Next entry is text field (skip)
                getline(is,data_string, '\t');

                //Save phi (curvature angle) values in rad
                sensor.phi.resize(sensor.num_curv_points);
                for(int j = 0; j < sensor.num_curv_points; j++)
                {
                    getline(is,data_string, '\t');
                    sensor.phi(j) = std::stod(data_string);
                }

                //Next entry is text field (skip)
                getline(is,data_string, '\t');

                //Next entry is number of shape points
                getline(is,data_string, '\t');
                sensor.num_shape_points = std::stoi(data_string);

                sensor.shape.resize(sensor.num_shape_points,3);
                sensor.arc_length.resize(sensor.num_shape_points);

                //Save all x values and arclength values
                for(int j = 0; j < sensor.num_shape_points; j++)
                {
                    //X
                    getline(is,data_string, '\t');
                    sensor.shape(j,0) = 0.01*std::stod(data_string); //convert cm to m
                    //Arc legnth
                    sensor.arc_length(j) = 0.001*j; //1 mm resolution, starting at 0
                }


                //Next entry is text field (skip) and again number of shape points (skip too)
                getline(is,data_string, '\t');
                getline(is,data_string, '\t');

                //Save all y values
                for(int j = 0; j < sensor.num_shape_points; j++)
                {
                    //X
                    getline(is,data_string, '\t');
                    sensor.shape(j,1) = 0.01*std::stod(data_string); //convert cm to m
                }


                //Next entry is text field (skip) and again number of shape points (skip too)
                getline(is,data_string, '\t');
                getline(is,data_string, '\t');

                //Save all z values
                for(int j = 0; j < sensor.num_shape_points; j++)
                {
                    //Z
                    getline(is,data_string, '\t');
                    sensor.shape(j,2) = 0.01*std::stod(data_string); //convert cm to m
                }


                //Next entry is either new curvature data (while loop will restart and add new sensor) or new line (no new sensor)
                getline(is,data_string, '\t');

                sample.sensors.push_back(sensor);

                num_sensors++;
            }

            sample.num_sensors = num_sensors;

            return true;

        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
    }

}

void ShapeSensingInterface::extracted(Sample const &sample,
                                      Eigen::VectorXd &sample_data,
                                      unsigned int &index) const {
    for (auto sensor : sample.sensors) {

        unsigned int sensor_shape_points = sensor.num_shape_points;

        sample_data.block(index, 0, sensor_shape_points, 1)
            << sensor.arc_length;

        index += sensor_shape_points;

        Eigen::VectorXd shape_vector = Eigen::Map<Eigen::VectorXd>(
            sensor.shape.data(), sensor.shape.size());

        sample_data.block(index, 0, sensor_shape_points * 3, 1) << shape_vector;

        index += sensor_shape_points * 3;
    }
}

void ShapeSensingInterface::getSamplesData(YAML::Node &t_FBGS_node, Eigen::MatrixXd &t_FBGS_data)const
{

    /*
    data_order:
        0: sample_number
        1: time_stamp
        2: number_of_sensors
        number_of_points_per_sensors:
          - number_of_datapoints
        sensors_data:
          - arc_length_coordinates
          - x_positions
          - y_positions
          - z_positions
     */


    unsigned int number_of_rows = 3;

    for(const auto& sensor : m_samples_stack[0].sensors){
        unsigned int num_shape_points = sensor.num_shape_points;

        //  Add row for sensor number of points
        number_of_rows++;

        // Add rows for sensor arc length, x, y and z positions
        number_of_rows += num_shape_points*4;
    }

    unsigned int number_of_columns = m_samples_stack.size();



    t_FBGS_data = Eigen::MatrixXd(number_of_rows, number_of_columns);


    for(unsigned int col=0; const auto& sample : m_samples_stack){

        Eigen::VectorXd sample_data(number_of_rows);

//        std::cout << "sample number : " << sample.sample_number << "\n";
        std::cout.flush();

        unsigned int index = 0;
        sample_data.block<3, 1>(0, 0) <<sample.sample_number,
                                        sample.time_stamp.time_since_epoch().count(),
                                        sample.num_sensors;

        index += 3;

        for(const auto& sensor : sample.sensors)
            sample_data.block(index++, 0, 1, 1) << sensor.num_shape_points;

        extracted(sample, sample_data, index);


        t_FBGS_data.col(col++) = sample_data;
    }






    t_FBGS_node["number_of_snapshots"] = m_samples_stack.size();
    t_FBGS_node["frequency"] = m_frequency;
    t_FBGS_node["duration"] = std::chrono::duration<double>(m_samples_stack[m_samples_stack.size()-1].time_stamp - m_start).count();
    t_FBGS_node["number_of_sensors"] = m_samples_stack[0].num_sensors;

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
