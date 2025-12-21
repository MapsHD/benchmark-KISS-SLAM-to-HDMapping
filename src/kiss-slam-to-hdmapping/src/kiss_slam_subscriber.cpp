/**
 * KISS-SLAM to HDMapping Converter
 *
 * Reads a ROS2 bag containing KISS-SLAM output topics and converts to HDMapping format.
 * Input topics:
 *   - /kiss_slam/local_map (sensor_msgs/PointCloud2)
 *   - /kiss_slam/odometry (nav_msgs/Odometry)
 * Output:
 *   - session.json
 *   - scan_lio_*.laz
 *   - trajectory_lio_*.csv
 *   - poses.reg / lio_initial_poses.reg
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>
#include <nlohmann/json.hpp>
#include "rosbag2_cpp/reader.hpp"
#include "rclcpp/serialization.hpp"
#include "laz_writer.hpp"

struct TrajectoryPose
{
    double timestamp_ns;
    double x_m;
    double y_m;
    double z_m;
    double qw;
    double qx;
    double qy;
    double qz;
    Eigen::Affine3d pose;
};

namespace fs = std::filesystem;

std::vector<Point3Di> points_global;
std::vector<TrajectoryPose> trajectory;
std::vector<std::vector<TrajectoryPose>> chunks_trajectory;

bool save_poses(const std::string &file_name, std::vector<Eigen::Affine3d> &m_poses, std::vector<std::string> &filenames)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.good())
    {
        std::cerr << "Cannot save file: " << file_name << std::endl;
        return false;
    }

    outfile << m_poses.size() << std::endl;
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        outfile << filenames[i] << std::endl;
        outfile << m_poses[i](0, 0) << " " << m_poses[i](0, 1) << " " << m_poses[i](0, 2) << " " << m_poses[i](0, 3) << std::endl;
        outfile << m_poses[i](1, 0) << " " << m_poses[i](1, 1) << " " << m_poses[i](1, 2) << " " << m_poses[i](1, 3) << std::endl;
        outfile << m_poses[i](2, 0) << " " << m_poses[i](2, 1) << " " << m_poses[i](2, 2) << " " << m_poses[i](2, 3) << std::endl;
        outfile << "0 0 0 1" << std::endl;
    }
    outfile.close();
    return true;
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input_bag> <output_directory>" << std::endl;
        return 1;
    }

    const std::string input_bag = argv[1];
    const std::string output_directory = argv[2];

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializationPointCloud2;
    rclcpp::Serialization<nav_msgs::msg::Odometry> serializationOdom;

    std::cout << "Processing bag: " << input_bag << std::endl;

    rosbag2_cpp::Reader bag;
    bag.open(input_bag);

    while (bag.has_next())
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = bag.read_next();

        // Process point cloud messages
        if (msg->topic_name == "/kiss_slam/local_map")
        {
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            serializationPointCloud2.deserialize_message(&serialized_msg, cloud_msg.get());

            if (!cloud_msg || cloud_msg->data.empty())
            {
                std::cerr << "Empty PointCloud2 message!" << std::endl;
                continue;
            }

            size_t num_points = cloud_msg->width * cloud_msg->height;

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

            for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z)
            {
                Point3Di point_global;

                if (cloud_msg->header.stamp.sec != 0 || cloud_msg->header.stamp.nanosec != 0)
                {
                    uint64_t sec_in_ms = static_cast<uint64_t>(cloud_msg->header.stamp.sec) * 1000ULL;
                    uint64_t ns_in_ms = static_cast<uint64_t>(cloud_msg->header.stamp.nanosec) / 1'000'000ULL;
                    point_global.timestamp = sec_in_ms + ns_in_ms;
                }

                point_global.point = Eigen::Vector3d(*iter_x, *iter_y, *iter_z);
                point_global.intensity = 0;
                point_global.index_pose = static_cast<int>(i);
                point_global.lidarid = 0;
                point_global.index_point = static_cast<int>(i);

                points_global.push_back(point_global);
            }

            std::cout << "Processed cloud with " << num_points << " points, total: " << points_global.size() << std::endl;
        }

        // Process odometry messages
        if (msg->topic_name == "/kiss_slam/odometry")
        {
            auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            serializationOdom.deserialize_message(&serialized_msg, odom_msg.get());

            if (!odom_msg)
            {
                std::cerr << "Odometry deserialization error!" << std::endl;
                continue;
            }

            TrajectoryPose pose;

            uint64_t sec_in_ms = static_cast<uint64_t>(odom_msg->header.stamp.sec) * 1000ULL;
            uint64_t ns_in_ms = static_cast<uint64_t>(odom_msg->header.stamp.nanosec) / 1'000'000ULL;
            pose.timestamp_ns = sec_in_ms + ns_in_ms;

            pose.x_m = odom_msg->pose.pose.position.x;
            pose.y_m = odom_msg->pose.pose.position.y;
            pose.z_m = odom_msg->pose.pose.position.z;
            pose.qw = odom_msg->pose.pose.orientation.w;
            pose.qx = odom_msg->pose.pose.orientation.x;
            pose.qy = odom_msg->pose.pose.orientation.y;
            pose.qz = odom_msg->pose.pose.orientation.z;

            pose.pose = Eigen::Affine3d::Identity();
            Eigen::Vector3d trans(pose.x_m, pose.y_m, pose.z_m);
            Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
            pose.pose.translation() = trans;
            pose.pose.linear() = q.toRotationMatrix();

            trajectory.push_back(pose);
        }
    }

    std::cout << "Total points: " << points_global.size() << std::endl;
    std::cout << "Total poses: " << trajectory.size() << std::endl;

    // Chunk point clouds (2M points per chunk)
    std::vector<std::vector<Point3Di>> chunks_pc;
    std::vector<Point3Di> chunk;
    int counter = 0;

    for (size_t i = 0; i < points_global.size(); i++)
    {
        chunk.push_back(points_global[i]);

        if (chunk.size() > 2000000)
        {
            counter++;
            chunks_pc.push_back(chunk);
            chunk.clear();
            std::cout << "Adding chunk [" << counter << "]" << std::endl;
        }
    }

    // Remaining points
    std::cout << "Remaining points: " << chunk.size() << std::endl;
    if (chunk.size() > 1000000)
    {
        chunks_pc.push_back(chunk);
    }

    std::cout << "Cleaning global points..." << std::endl;
    points_global.clear();

    // Index trajectory to chunks
    std::cout << "Indexing trajectory to chunks..." << std::endl;
    chunks_trajectory.resize(chunks_pc.size());

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        for (size_t j = 0; j < chunks_pc.size(); j++)
        {
            if (!chunks_pc[j].empty())
            {
                if (trajectory[i].timestamp_ns >= chunks_pc[j][0].timestamp &&
                    trajectory[i].timestamp_ns < chunks_pc[j][chunks_pc[j].size() - 1].timestamp)
                {
                    chunks_trajectory[j].push_back(trajectory[i]);
                }
            }
        }
    }

    for (size_t i = 0; i < chunks_trajectory.size(); i++)
    {
        std::cout << "Chunk " << i << " has " << chunks_trajectory[i].size() << " trajectory poses" << std::endl;
    }

    // Transform chunks to local coordinate system
    std::cout << "Transforming chunks to local coordinate system..." << std::endl;
    for (size_t i = 0; i < chunks_pc.size(); i++)
    {
        if (chunks_trajectory[i].empty())
            continue;

        Eigen::Vector3d trans(chunks_trajectory[i][0].x_m, chunks_trajectory[i][0].y_m, chunks_trajectory[i][0].z_m);
        Eigen::Quaterniond q(chunks_trajectory[i][0].qw, chunks_trajectory[i][0].qx, chunks_trajectory[i][0].qy, chunks_trajectory[i][0].qz);

        Eigen::Affine3d first_affine = Eigen::Affine3d::Identity();
        first_affine.translation() = trans;
        first_affine.linear() = q.toRotationMatrix();
        Eigen::Affine3d first_affine_inv = first_affine.inverse();

        for (auto &p : chunks_pc[i])
        {
            p.point = first_affine_inv * p.point;
        }
    }

    // Create output directory
    if (!fs::exists(output_directory))
    {
        try
        {
            fs::create_directories(output_directory);
            std::cout << "Created output directory: " << output_directory << std::endl;
        }
        catch (const fs::filesystem_error &e)
        {
            std::cerr << "Error creating directory: " << e.what() << std::endl;
            return 1;
        }
    }

    fs::path outwd = output_directory;

    // Calculate offset
    Eigen::Vector3d offset(0, 0, 0);
    int cc = 0;
    for (size_t i = 0; i < chunks_trajectory.size(); i++)
    {
        for (size_t j = 0; j < chunks_trajectory[i].size(); j++)
        {
            Eigen::Vector3d trans_curr(chunks_trajectory[i][j].x_m, chunks_trajectory[i][j].y_m, chunks_trajectory[i][j].z_m);
            offset += trans_curr;
            cc++;
        }
    }
    if (cc > 0)
        offset /= cc;

    std::vector<Eigen::Affine3d> m_poses;
    std::vector<std::string> file_names;

    // Save chunks
    for (size_t i = 0; i < chunks_pc.size(); i++)
    {
        if (chunks_pc[i].empty() || chunks_trajectory[i].empty())
            continue;

        // Save LAZ file
        fs::path laz_path = outwd / ("scan_lio_" + std::to_string(i) + ".laz");
        std::cout << "Saving: " << laz_path << " with " << chunks_pc[i].size() << " points" << std::endl;
        saveLaz(laz_path.string(), chunks_pc[i]);
        file_names.push_back("scan_lio_" + std::to_string(i) + ".laz");

        // Save trajectory CSV
        fs::path csv_path = outwd / ("trajectory_lio_" + std::to_string(i) + ".csv");
        std::cout << "Saving: " << csv_path << std::endl;

        std::ofstream outfile(csv_path);
        if (!outfile.good())
        {
            std::cerr << "Cannot save file: " << csv_path << std::endl;
            return 1;
        }

        outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 timestampUnix_nanoseconds" << std::endl;

        Eigen::Vector3d trans(chunks_trajectory[i][0].x_m, chunks_trajectory[i][0].y_m, chunks_trajectory[i][0].z_m);
        Eigen::Quaterniond q(chunks_trajectory[i][0].qw, chunks_trajectory[i][0].qx, chunks_trajectory[i][0].qy, chunks_trajectory[i][0].qz);

        Eigen::Affine3d first_affine = Eigen::Affine3d::Identity();
        first_affine.translation() = trans;
        first_affine.linear() = q.toRotationMatrix();
        Eigen::Affine3d first_affine_inv = first_affine.inverse();
        m_poses.push_back(first_affine);

        for (size_t j = 0; j < chunks_trajectory[i].size(); j++)
        {
            Eigen::Vector3d trans_curr(chunks_trajectory[i][j].x_m, chunks_trajectory[i][j].y_m, chunks_trajectory[i][j].z_m);
            Eigen::Quaterniond q_curr(chunks_trajectory[i][j].qw, chunks_trajectory[i][j].qx, chunks_trajectory[i][j].qy, chunks_trajectory[i][j].qz);

            Eigen::Affine3d curr_affine = Eigen::Affine3d::Identity();
            curr_affine.translation() = trans_curr;
            curr_affine.linear() = q_curr.toRotationMatrix();

            auto pose = first_affine_inv * curr_affine;

            outfile << std::setprecision(20) << chunks_trajectory[i][j].timestamp_ns * 1e9 << " "
                    << std::setprecision(10)
                    << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " " << pose(0, 3) << " "
                    << pose(1, 0) << " " << pose(1, 1) << " " << pose(1, 2) << " " << pose(1, 3) << " "
                    << pose(2, 0) << " " << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << " "
                    << std::setprecision(20) << chunks_trajectory[i][j].timestamp_ns * 1e9
                    << std::endl;
        }
        outfile.close();
    }

    // Apply offset to poses
    for (auto &m : m_poses)
    {
        m.translation() -= offset;
    }

    // Save pose files
    fs::path initial_poses_path = outwd / "lio_initial_poses.reg";
    save_poses(initial_poses_path.string(), m_poses, file_names);

    fs::path poses_path = outwd / "poses.reg";
    save_poses(poses_path.string(), m_poses, file_names);

    // Save session.json
    fs::path session_path = outwd / "session.json";
    std::cout << "Saving: " << session_path << std::endl;

    nlohmann::json jj;
    nlohmann::json j;
    j["offset_x"] = 0.0;
    j["offset_y"] = 0.0;
    j["offset_z"] = 0.0;
    j["folder_name"] = outwd.string();
    j["out_folder_name"] = outwd.string();
    j["poses_file_name"] = poses_path.string();
    j["initial_poses_file_name"] = initial_poses_path.string();
    j["out_poses_file_name"] = poses_path.string();
    j["lidar_odometry_version"] = "KISS-SLAM";

    jj["Session Settings"] = j;

    nlohmann::json jlaz_file_names;
    for (size_t i = 0; i < chunks_pc.size(); i++)
    {
        if (chunks_pc[i].empty() || chunks_trajectory[i].empty())
            continue;

        fs::path laz_path = outwd / ("scan_lio_" + std::to_string(i) + ".laz");
        nlohmann::json jfn{{"file_name", laz_path.string()}};
        jlaz_file_names.push_back(jfn);
    }
    jj["laz_file_names"] = jlaz_file_names;

    std::ofstream session_file(session_path);
    session_file << jj.dump(2);
    session_file.close();

    std::cout << "Conversion complete!" << std::endl;
    return 0;
}
