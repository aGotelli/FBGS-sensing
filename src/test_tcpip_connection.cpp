#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <real_time_tools/spinner.hpp>

#include <algorithm>
#include <numeric>

#include "utilities/Eigen/eigen_io.hpp"


#include <QApplication>
#include <QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>



std::atomic_bool StopDemos(false);


void my_handler(int)
{
    StopDemos = true;
}




int main(int argc, char *argv[])
{
    QApplication* app = new QApplication(argc, argv);


    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopDemos = false;


    std::string m_ip_address = "192.168.1.11";
    std::string m_port_number = "5001";

    boost::asio::io_context m_io_context;
    boost::asio::ip::tcp::resolver m_resolver(m_io_context);
    boost::asio::ip::tcp::socket m_socket(m_io_context);


    const double frequency = 100;
    const double recording_time = 10;
    const unsigned int number_of_steps = recording_time*frequency;
    unsigned int it_t = 0;


    std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> time_stamps(number_of_steps);
    std::vector<unsigned int> samples_numbers(number_of_steps);


    auto readData = [&]()->unsigned int
    {
        unsigned int sample_numb = 0;

        char buffer[4];
        auto asio_buffer = boost::asio::buffer(&buffer,4);
        int size;
        boost::asio::streambuf data;
        std::string data_string;
        std::istream is(&data);


        try
        {

            //  First read the first 4 bytes to figure out the size of the following ASCII string
            boost::asio::read(m_socket, asio_buffer);

            //  Convert the received bytes to signed integer
            size = int((unsigned char)(buffer[0]) << 24 |
                       (unsigned char)(buffer[1]) << 16 |
                       (unsigned char)(buffer[2]) << 8 |
                       (unsigned char)(buffer[3]));


            //  Now read the remaining ASCII string of the current data package
            boost::asio::read(m_socket,data,boost::asio::transfer_exactly(size));

            getline(is,data_string, '\t');
            getline(is,data_string, '\t');

            //Next string is the sample number
            getline(is,data_string, '\t');
            sample_numb = std::stoi(data_string);

        }
        catch(std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }

        return sample_numb;
    };



    //Get endpoints
    std::cout << "[FBGS] Get endpoints... ";
    boost::asio::ip::tcp::resolver::results_type endpoints = m_resolver.resolve(m_ip_address, m_port_number);

    boost::asio::ip::tcp::endpoint ep = *endpoints;

    std::cout << "Establishing connection to " << ep << "... ";

    //Connect to socket and open connection
    boost::asio::connect(m_socket, endpoints);

    std::cout << "Connected!" << std::endl << std::endl;


    real_time_tools::Spinner spinner;
    double dt_spinner = 0.15;
    spinner.set_period( dt_spinner );

    //   wait next sample to be ready
    static int pos=0;
    char cursor[4]={'/','-','\\','|'};
    while(m_socket.available() < 4
           and not StopDemos){
        std::cout << "Waiting channel to be on...  " << cursor[pos] << "\r";
        pos = (pos+1) % 4;
        std::cout.flush();
        spinner.spin();
    }

    std::cout << "Sensor Ready!" << std::endl << std::endl;

    unsigned int dumped = 0;
    while(m_socket.available() >= 4
           and not StopDemos){
        readData();
        dumped++;
    }


    std::cout << "dumped : " << dumped << " samples." << std::endl;









    const auto start = std::chrono::high_resolution_clock::now();


    while(it_t < number_of_steps
           and not StopDemos){

        if(m_socket.available() >= 4){
            time_stamps[it_t] = std::chrono::high_resolution_clock::now();
            samples_numbers[it_t] = readData();
            it_t++;
        }



    }

    std::vector<double> time_steps;
    for(unsigned int i=1; i<time_stamps.size(); i++){
        std::chrono::duration<double> diff = time_stamps[i] - time_stamps[i-1];
        time_steps.push_back( std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() );
    }

    const std::chrono::duration<double> recording = time_stamps[time_stamps.size()-1] - start;
    const double recorded_time = std::chrono::duration_cast<std::chrono::milliseconds>(recording).count();

    const double total = std::accumulate(time_steps.begin(),
                                         time_steps.end(),
                                         0.0);

    const double size = time_steps.size();

    const double mean = total/size;

    double min = 14000;
    double max = -14000;
    for (const auto& time_step : time_steps) {
        if(time_step>max) max = time_step;
        if(time_step<min) min = time_step;
    }

    // Now calculate the variance
    double sq_sum = std::inner_product(time_steps.begin(), time_steps.end(), time_steps.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / size - mean * mean);



    const double dt = recording_time / size;
    const double Hz = 1.0f/(dt/1000);

    std::cout <<"Recorded for " << recorded_time/1000.0 << " seconds\n"
                "   As bach of " << size << " samples\n"
                "   Corresponding dt " << dt << " ms (" << Hz << " Hz)\n"
                "   Mean : " << mean << "\n"
                "   min : " << min << "\n"
                "   max : " << max << "\n"
                "   standard deviation : " << stdev << "\n";
    std::cout.flush();


    Eigen::VectorXd time_steps_stack = Eigen::Map<Eigen::VectorXd>(time_steps.data(), time_steps.size());
    writeToFile("time_steps", time_steps_stack, "data/");



    QtCharts::QChart* chart = new QtCharts::QChart();

    QtCharts::QLineSeries* series = new QtCharts::QLineSeries();
    series->setColor(Qt::red);
    for(unsigned int index=0;const auto time_step : time_steps)
        series->append(index++, time_step);
    chart->addSeries(series);


    std::vector<unsigned int> sample_number_steps;
    for(unsigned int i=1; i<samples_numbers.size(); i++){
        unsigned int diff = samples_numbers[i] - samples_numbers[i-1];
        if(diff<100)
            sample_number_steps.push_back( diff );
    }

    QtCharts::QLineSeries* series_samples_numb = new QtCharts::QLineSeries();
    series_samples_numb->setColor(Qt::blue);
    for(unsigned int index=0;const auto sample_number_step : sample_number_steps)
        series_samples_numb->append(index++, sample_number_step);
    chart->addSeries(series_samples_numb);




    chart->createDefaultAxes();
    chart->axes(Qt::Horizontal).back()->setTitleText( "sample number" );
//    chart->axes(Qt::Horizontal).back()->setRange(-1.1, 1.1);
    chart->axes(Qt::Vertical).back()->setTitleText( "Time step [ms]" );
//    chart->axes(Qt::Vertical).back()->setRange(-1.1, 1.1);
//    chart->legend();
//    chart->setTitle( "Robot configuration" );

    QtCharts::QChartView* chart_view = new QtCharts::QChartView(chart);
    chart_view->resize(800,800);
    chart_view->show();


    app->exec();


    return 0;
}
