#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string.h>
#include <cstdlib>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Float32.h"

#define truncated_coverage_angle_ 3.14*4/6

const double W = 0.3;
const double L = 0.7;
// PARAMETER TO BE DEFINED 
// const double k = 2 * 1.4; //sqrt
// const double k = 4; //cubed
// const double k = 3.5; //jaewon's algorithm sqrt
const double k = 7.5; //jaewon's algorithm cubed
const double deltath = 0.1; // disparity gijoon
const double MAX_Velocity = 100;//limited and crahsed so trying limitless again 15.0; //
const double MAX_STEERING_ANGLE = 0.4819;// 25 * 3.14 / 180; //
const double MIN_SAFE_Distance = 15.0; //
const double additional_gap = 5; //originally 1.2
const int average_size = 9;

const double DELTA_CURVE_ANGLE_THRESHOLD = 5; //in degrees
const double CURVE_VELOCITY_COEFFICIENT = 0.1; //how much lower the speed will be when curving

double min(double a, double b) {
    return a > b ? b : a;
}

double get_velocity_by_steeringAngle(double delta_steering_angle, double front_distance) {
    // double velocity = std::min(k * sqrt(front_distance), MAX_Velocity); //squared
    double velocity = std::min(k * pow(front_distance, 1/3), MAX_Velocity); //cubed
	if (delta_steering_angle > 3.14159265358979 / 180 * DELTA_CURVE_ANGLE_THRESHOLD) {
		return velocity / delta_steering_angle * CURVE_VELOCITY_COEFFICIENT;
	}
	else {
		return velocity;
	} //we should think about cubed later on as well
}

std::vector<double> apply_smoothing_filter(const std::vector<double>& input_vector)//, size_t smoothing_filter_size)
{
    std::vector<double> smoothened_vector;
    /*for(size_t i=smoothing_filter_size; i<input_vector.size()-smoothing_filter_size; ++i)
    {
        double current_sum = 0;
        for(size_t j = i-smoothing_filter_size + 1; j<i+smoothing_filter_size; ++j)
        {
            assert(j>=0 && j<input_vector.size() && "Convolution operator boundary condition violated.");
            current_sum += input_vector[j];
        }
        smoothened_vector.push_back(current_sum/(2.0*smoothing_filter_size - 1.0));
    }*/
    for(size_t i=0; i<input_vector.size(); i++)
    {
        double avg_range = 0;
        if (i < average_size / 2){
            /*for (int j = 0; j < 1 + i + (average_size / 2); j++)
                avg_range += input_vector.at(j);
            avg_range /= 1 + i + (average_size / 2);*/
            avg_range = input_vector[i];
        }
        else if ((input_vector.size() - 1 - i) < average_size / 2){
            /*for (int j = input_vector.size() - 1; j >= i - average_size / 2; j--)
                avg_range += input_vector.at(j);
            avg_range /= input_vector.size() - i + (average_size / 2);*/
            avg_range = input_vector[i];
        }
        else{
            for (int j = 0; j < average_size; j++)
                avg_range += input_vector[j + i - average_size / 2];
            avg_range /= average_size;
        }
        smoothened_vector.push_back(avg_range);
    }
    return smoothened_vector;
}

std::pair <int, int> truncated_start_and_end_indices(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
    const double truncation_angle_coverage) {
    const auto truncated_range_size = static_cast<size_t>(
        (truncation_angle_coverage / (scan_msg->angle_max - scan_msg->angle_min)) * scan_msg->ranges.size());
    const size_t start_index = (scan_msg->ranges.size() / 2) - (truncated_range_size / 2);
    const size_t end_index = (scan_msg->ranges.size() / 2) + (truncated_range_size / 2);
    return std::make_pair(start_index, end_index);
}

size_t maximum_element_index(const std::vector<double>& input_vector)
{
    //std::vector<double> smoothened_vector = apply_smoothing_filter(input_vector, 3);
    //const auto max_value_iterator = std::max_element(smoothened_vector.begin(), smoothened_vector.end());
    //return std::distance(smoothened_vector.begin(), max_value_iterator);
	std::vector<double> mod_vector;
	mod_vector.assign(input_vector.begin(), input_vector.end());
	for (int i = 0; i < mod_vector.size(); i++) {
		mod_vector[i] -= std::pow(abs(i - int(mod_vector.size())/2), (double)1/3) * 0.03;
	}
    const auto max_value_iterator = std::max_element(mod_vector.begin(), mod_vector.end());
    return std::distance(mod_vector.begin(), max_value_iterator); //commented for new algorithm considering the straight
    
}

class longest_path {
public:
    longest_path():
        node_handle_(ros::NodeHandle()),
        //lidar_sub_(node_handle_.subscribe("scan", 100, &longest_path::scan_callback, this)),
        //drive_pub_(node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 100)), // originally "nav"
        
        lidar_sub_(node_handle_.subscribe("team_cat/scan", 100, &longest_path::scan_callback, this)),
        drive_pub_(node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("team_cat/drive", 100)), // originally "nav"
        truncated_(false){}
    

    std::vector<double> preprocess_lidar_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg) const
    {
        std::vector<double> filtered_ranges;
        for (size_t i = truncated_start_index_; i < truncated_end_index_; ++i)
        {
            if (std::isnan(scan_msg->ranges[i])) // the situation not expected are sifted...
            {
                filtered_ranges.push_back(0.0);
            }
            /*else if (scan_msg->ranges[i] > max_accepted_distance_ || std::isinf(scan_msg->ranges[i]))
            {
                filtered_ranges.push_back(max_accepted_distance_);
            }*/
            else
            {
                filtered_ranges.push_back(scan_msg->ranges[i]);
            }
        }
        return filtered_ranges;
    }


    std::vector <int> find_disparities(const std::vector <double>& LIDARList, int start_i, int end_i) {
        std::vector<int> ret;
        ret.clear();

        double prev_distance = LIDARList[start_i];
        for (int i = start_i; i < end_i; i++) {
            if (abs(LIDARList[i] - prev_distance) > deltath)
                ret.push_back(i);
            prev_distance = LIDARList[i];
        }

        return ret;
    }


    int get_delta_ThetaIndex(double distance, double angle_increment) {
        double theta = W / 2 / distance * additional_gap; //a little bit of space
        int delta_index = (int)(theta / angle_increment);
        //ROS_INFO("Delta Index is %i", delta_index);
        return delta_index;
    }

    //std::pair <double, double> NextMotion(const std::vector <double>& LIDARList)
    //{
    //    double finaltheta = 0;
    //    double finalvelocity = 0;


    //    return std::make_pair(finaltheta, finalvelocity);
    //}

    void extend_disparity(std::vector<double>& distance_range, const int start_index, int delta_index) {
        //double p_min = distance_range.at(start_index);
        double p_min = std::min(distance_range.at(start_index), distance_range.at(start_index-1));
        //for (int i = start_index + 1; i < start_index + delta_index + 1; i++) {
        //    distance_range.at(i) = min(distance_range.at(i), p_min);
        //}
        //for (int i = start_index + 1; i < start_index + delta_index + 1; i++) {
        //    distance_range.at(i) = min(distance_range.at(i), p_min);
        //}
        int current_index = start_index;
        while (current_index < distance_range.size() && current_index < start_index + delta_index + 1) {
            distance_range.at(current_index) = std::min(distance_range.at(current_index), p_min);
            current_index++;
        }
        current_index = start_index - 1;
        while (current_index > 0 && current_index > start_index - delta_index - 1) {
            distance_range.at(current_index) = std::min(distance_range.at(current_index), p_min);
            current_index--;
        }
    }

    bool BackCollision_detect(const sensor_msgs::LaserScan::ConstPtr& scan_msg, int truncated_start_idx, int truncated_end_idx) {
        bool risky = false;
        double RMAX = sqrt(W * W / 4 + L * L);
        double MIN = scan_msg->ranges[0];
        for (int i = 1; i < (sizeof(scan_msg->ranges)/sizeof(scan_msg->ranges[0]))/2 - 3.14/2/(scan_msg->angle_increment); i++) {
            MIN = min(scan_msg->ranges[i], MIN);
        }
        for (int j = (sizeof(scan_msg->ranges)/sizeof(scan_msg->ranges[0]))/2 + 3.14/2/(scan_msg->angle_increment); j < sizeof(scan_msg->ranges)/sizeof(scan_msg->ranges[0]); j++) {
            MIN = min(scan_msg->ranges[j], MIN);
        }
        //ROS_INFO("RMAX is %f and MIN is %f", RMAX, MIN);
        return RMAX > MIN;
    }// finish 


    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        if (!truncated_)
        {
            const auto truncated_indices =
                truncated_start_and_end_indices(scan_msg, truncated_coverage_angle_);
            truncated_start_index_ = truncated_indices.first;
            truncated_end_index_ = truncated_indices.second;
            truncated_ = true;
        }

        // Pre-Process (zero out nans and Filter)
        auto filtered_ranges = preprocess_lidar_scan(scan_msg);
        filtered_ranges = apply_smoothing_filter(filtered_ranges);

        double steering_angle = 0;


        int original_max = maximum_element_index(filtered_ranges);

        int max_element_index = -1;

        if (!BackCollision_detect(scan_msg, truncated_start_index_, truncated_end_index_)) {
            auto disparities_indices = find_disparities(filtered_ranges, 0, filtered_ranges.size());

            for (int disparity_index = 0; disparity_index < disparities_indices.size(); disparity_index++) {
                int range_index = disparities_indices.at(disparity_index);
                int delta_index = get_delta_ThetaIndex(filtered_ranges.at(range_index), scan_msg->angle_increment);
                extend_disparity(filtered_ranges, range_index, delta_index);
            }
            /* max_element_index = maximum_element_index(filtered_ranges);
            steering_angle = scan_msg->angle_min + scan_msg->angle_increment * (truncated_start_index_ + max_element_index); Erased for Bamjoon's algorithm*/
			int temp = maximum_element_index(filtered_ranges);
	    	if (filtered_ranges.at(temp) < 15) {
            	max_element_index = temp;
			//ROS_INFO("No While!");
	    	}
			else { //If there is plenty space ahead, go in a diretion close to being straight as possible
				max_element_index = filtered_ranges.size() / 2;
				if (temp < filtered_ranges.size() / 2) {
					while (filtered_ranges.at(max_element_index) < 15 && max_element_index > 0)
						max_element_index--;
					//max_element_index = (filtered_ranges.size() / 2 + temp) / 2;
				}
				else {
					while (filtered_ranges.at(max_element_index) < 15 && (max_element_index < filtered_ranges.size() - 1))
						max_element_index++;
					//max_element_index = (filtered_ranges.size() / 2 + temp) / 2;
				}
			}
			double temp_steering_angle = scan_msg->angle_min + scan_msg->angle_increment * (truncated_start_index_ + max_element_index);
	    	/* if (max_element_index > filtered_ranges.size() / 2 - 15 && max_element_index < filtered_ranges.size() / 2 + 15)
	        	temp_steering_angle = 0; Comment to see if anything will be different*/
	        steering_angle = temp_steering_angle / 2; //3 * std::pow(temp_steering_angle, 1/3.);
            
            //ROS_INFO("The difference is %i", max_element_index - original_max);
            /*std::string x = "";
            for(int i=-2; i<2; i++) {
                x += (", "+ boost::lexical_cast<std::string>(filtered_ranges.at(maximum_element_index + i)));
            }
            ROS_INFO(x);*/
			if(steering_angle > MAX_STEERING_ANGLE) steering_angle = MAX_STEERING_ANGLE;
			if(steering_angle < -MAX_STEERING_ANGLE) steering_angle = -MAX_STEERING_ANGLE;
            ROS_INFO("Steering Angle is %f", steering_angle * 180 / 3.14);
            //ROS_INFO("1: %f, 2: %f, 3: %f, 4: %f, 5: %f\n", filtered_ranges.at(max_element_index-2), filtered_ranges.at(max_element_index-1), filtered_ranges.at(max_element_index), filtered_ranges.at(max_element_index+1), filtered_ranges.at(max_element_index+2));
            //ROS_INFO("Disparity Angle is %f", )
        }
        //else ROS_INFO("None");

        //max_element_index = maximum_element_index(filtered_ranges);

        // double velocity = std::min(k * filtered_ranges.at(filtered_ranges.size()/2), MAX_Velocity);
        // double velocity = std::min(k * sqrt(filtered_ranges.at(filtered_ranges.size()/2)), MAX_Velocity);
        // double velocity = std::min(k * std::pow(filtered_ranges.at(filtered_ranges.size()/2), 1/3.), MAX_Velocity);
        double velocity = get_velocity_by_steeringAngle(steering_angle, filtered_ranges.at(filtered_ranges.size()/2));

        // Publish Drive message
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = velocity;
        //ROS_INFO("Velocity is %f", velocity);

        drive_pub_.publish(drive_msg);
    }

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber lidar_sub_;
    ros::Publisher drive_pub_;

    int truncated_start_index_;
    int truncated_end_index_;
    bool truncated_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ourdrive");
    longest_path longpath;
    ros::spin();
    return 0;
}
