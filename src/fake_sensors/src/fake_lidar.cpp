//=================================================================================================
//                                          INCLUDES
//=================================================================================================
#include <chrono>
#include <memory>
#include <cmath>
#include <utility>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


//=================================================================================================
//                                        LIDAR CONFIG 
//=================================================================================================
class LidarConfig
{
public:
    const char* PARAM_MIN_RANGE = "min_range";
    const char* PARAM_MAX_RANGE = "max_range";
    const char* PARAM_MIN_ANGLE = "min_angle";
    const char* PARAM_MAX_ANGLE = "max_angle";
    const char* PARAM_SAMPLE_COUNT = "sample_count";
    const char* PARAM_SAMPLING_FREQUENCY = "sampling_frequency";

    //param od pliku csv i szumu
    const char* PARAM_CSV_FILE_PATH = "csv_file_path";
    const char* PARAM_GAUSSIAN_NOISE = "gaussian_noise_stddev";

    const double DEFAULT_MIN_RANGE = 0.2;
    const double DEFAULT_MAX_RANGE = 2.0;
    const double DEFAULT_MIN_ANGLE = -M_PI;
    const double DEFAULT_MAX_ANGLE = M_PI;
    const int DEFAULT_SAMPLE_COUNT = 360;
    const double DEFAULT_SAMPLING_FREQUENCY = 10.0;
    
    //domyślna wartość szumu
    const double DEFAULT_GAUSSIAN_NOISE = 0.0;
    const std::string DEAULT_CSV_FILE_PATH = "config/lidar_data_large.csv";

    const char* DESCRIPTION_MIN_RANGE =
    "minimum range value [m]";
    const char* DESCRIPTION_MAX_RANGE =
    "maximum range value [m]";
    const char* DESCRIPTION_MIN_ANGLE =
    "start angle of the scan [rad]";
    const char* DESCRIPTION_MAX_ANGLE =
    "end angle of the scan [rad]";
    const char* DESCRIPTION_SAMPLE_COUNT =
    "Number of samples per full laser scan";
    const char* DESCRIPTION_SAMPLING_FREQUENCY =
    "Number of full Scans per second.";

    //opisujemy nowe dane
    const char* DESCRIPTION_CSV_FILE_PATH =
    "Path to CSV file containing scan data";
    const char* DESCRIPTION_GAUSSIAN_NOISE =
    "Standard deviation for Gaussian noise.";

public:
    std::pair<double,double> range;
    std::pair<double,double> angle;
    int sample_count;
    double sampling_frequency;
    double gaussian_noise_stddev;
    std::string csv_file_path;

public:
    void declare_parameters(rclcpp::Node *node);
    void update_parameters(rclcpp::Node *node);
    void print_config(rclcpp::Node *node);

    double get_scan_step() const;
    int get_scan_period_ms() const;
    double get_scaled_sample(double scale) const;


};
    
double LidarConfig::get_scaled_sample(double scale) const
{
    return range.first + (range.second - range.first) * scale;
}

int LidarConfig::get_scan_period_ms() const
{
    return std::lround(1000/sampling_frequency);
}

double LidarConfig::get_scan_step() const
{
    return (angle.second-angle.first)/sample_count;
}    

void LidarConfig::declare_parameters(rclcpp::Node *node)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_MIN_RANGE;
    node->declare_parameter(PARAM_MIN_RANGE, DEFAULT_MIN_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MAX_RANGE;
    node->declare_parameter(PARAM_MAX_RANGE, DEFAULT_MAX_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MIN_ANGLE;
    node->declare_parameter(PARAM_MIN_ANGLE, DEFAULT_MIN_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_MAX_ANGLE;
    node->declare_parameter(PARAM_MAX_ANGLE, DEFAULT_MAX_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_SAMPLE_COUNT;
    node->declare_parameter(PARAM_SAMPLE_COUNT, DEFAULT_SAMPLE_COUNT, descriptor);
    descriptor.description = DESCRIPTION_SAMPLING_FREQUENCY;
    node->declare_parameter(PARAM_SAMPLING_FREQUENCY, DEFAULT_SAMPLING_FREQUENCY, descriptor);

    descriptor.description = DESCRIPTION_CSV_FILE_PATH;
    node->declare_parameter(PARAM_CSV_FILE_PATH, "", descriptor);
    descriptor.description = DESCRIPTION_GAUSSIAN_NOISE;
    node->declare_parameter(PARAM_GAUSSIAN_NOISE, DEFAULT_GAUSSIAN_NOISE, descriptor);
}

void LidarConfig::update_parameters(rclcpp::Node *node)
{
    this->range.first = node->get_parameter(PARAM_MIN_RANGE).as_double();
    this->range.second = node->get_parameter(PARAM_MAX_RANGE).as_double();
    this->angle.first = node->get_parameter(PARAM_MIN_ANGLE).as_double();
    this->angle.second = node->get_parameter(PARAM_MAX_ANGLE).as_double();
    this->sample_count = node->get_parameter(PARAM_SAMPLE_COUNT).as_int();
    this->sampling_frequency = node->get_parameter(PARAM_SAMPLING_FREQUENCY).as_double();

    this->csv_file_path = node->get_parameter(PARAM_CSV_FILE_PATH).as_string();
    this->gaussian_noise_stddev = node->get_parameter(PARAM_GAUSSIAN_NOISE).as_double();
}

void LidarConfig::print_config(rclcpp::Node *node)
{
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_RANGE, this->range.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_RANGE, this->range.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_ANGLE, this->angle.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_ANGLE, this->angle.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_SAMPLE_COUNT, this->sample_count);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_SAMPLING_FREQUENCY,
                this->sampling_frequency);
    RCLCPP_INFO(node->get_logger(), "Scan step = %f", this->get_scan_step());

    RCLCPP_INFO(node->get_logger(), "CSV file path: %s", this->csv_file_path.c_str());
    RCLCPP_INFO(node->get_logger(), "Gaussian noise stddev: %f", this->gaussian_noise_stddev);
}

//=================================================================================================
//                                         FAKE LIDAR NODE 
//=================================================================================================
class FakeLidar: public rclcpp::Node
{
public:
    FakeLidar();

private:
    LidarConfig _config;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scan_publisher;
    rclcpp::TimerBase::SharedPtr _scan_timer;

    std::vector<std::vector<double>> _csv_data;

private:
    void _publish_fake_scan();

    void _read_csv();
    double _apply_noise(double value);

};

FakeLidar::FakeLidar(): Node("fake_lidar")
{
    _config.declare_parameters(this);
    _config.update_parameters(this);
    _config.print_config(this);
    _scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan",10);
    _scan_timer = this->create_wall_timer(
        std::chrono::milliseconds(_config.get_scan_period_ms()),
        std::bind(&FakeLidar::_publish_fake_scan, this)
    );

    _read_csv();
}

void FakeLidar::_read_csv()
{
    std::ifstream file(_config.csv_file_path);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open CSV file: %s", _config.csv_file_path.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        while (std::getline(ss, value, ','))
        {
            row.push_back(std::stod(value));
        }

        _csv_data.push_back(row);
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Loaded CSV file with %lu rows", _csv_data.size());
}

double FakeLidar::_apply_noise(double value)
{
    if (_config.gaussian_noise_stddev > 0.0)
    {
        static std::default_random_engine generator;
        std::normal_distribution<double> distribution(0.0, _config.gaussian_noise_stddev);
        return value + distribution(generator);
    }
    return value;
}
    
void FakeLidar::_publish_fake_scan()
{
    auto msg = sensor_msgs::msg::LaserScan();
    
    msg.header.stamp = this->now();
    msg.header.frame_id = "laser_frame";

    msg.angle_min = _config.angle.first;
    msg.angle_max = _config.angle.second;
    msg.range_min = _config.range.first;
    msg.range_max = _config.range.second;
    msg.angle_increment = _config.get_scan_step();
    msg.time_increment = 0;
    msg.scan_time = _config.get_scan_period_ms()/1000.0;

    if (_csv_data.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No data loaded from CSV file");
        return;
    }

    std::vector<float> ranges;
    for (double value : _csv_data[0])
    {
        double noisy_value = _apply_noise(value);
        if (noisy_value >= _config.range.first && noisy_value <= _config.range.second)
        {
            ranges.push_back(static_cast<float>(noisy_value));
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Value out of range: %f", noisy_value);
        }
    }

    msg.ranges = ranges;
    _scan_publisher->publish(msg);

}

//=================================================================================================
//                                          MAIN 
//=================================================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FakeLidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

