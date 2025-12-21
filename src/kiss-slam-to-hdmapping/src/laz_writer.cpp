#include "laz_writer.hpp"
#include <laszip/laszip_api.h>
#include <iostream>
#include <limits>
#include <cstring>

bool saveLaz(const std::string &filename, const std::vector<Point3Di> &points_global)
{
    constexpr float scale = 0.0001f; // one tenth of millimeter

    // Find bounding box
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double max_z = std::numeric_limits<double>::lowest();
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();

    for (const auto &p : points_global)
    {
        if (p.point.x() < min_x) min_x = p.point.x();
        if (p.point.x() > max_x) max_x = p.point.x();
        if (p.point.y() < min_y) min_y = p.point.y();
        if (p.point.y() > max_y) max_y = p.point.y();
        if (p.point.z() < min_z) min_z = p.point.z();
        if (p.point.z() > max_z) max_z = p.point.z();
    }

    std::cout << "Processing: " << filename << " with " << points_global.size() << " points" << std::endl;

    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        std::cerr << "ERROR: creating laszip writer" << std::endl;
        return false;
    }

    laszip_header *header;
    if (laszip_get_header_pointer(laszip_writer, &header))
    {
        std::cerr << "ERROR: getting header pointer" << std::endl;
        return false;
    }

    // Populate header
    header->file_source_ID = 4711;
    header->global_encoding = (1 << 0);
    header->version_major = 1;
    header->version_minor = 2;
    header->point_data_format = 1;
    header->point_data_record_length = 28;
    header->number_of_point_records = points_global.size();
    header->number_of_points_by_return[0] = points_global.size();
    header->number_of_points_by_return[1] = 0;
    header->x_scale_factor = scale;
    header->y_scale_factor = scale;
    header->z_scale_factor = scale;
    header->max_x = max_x;
    header->min_x = min_x;
    header->max_y = max_y;
    header->min_y = min_y;
    header->max_z = max_z;
    header->min_z = min_z;

    laszip_BOOL compress = (strstr(filename.c_str(), ".laz") != nullptr);

    if (laszip_open_writer(laszip_writer, filename.c_str(), compress))
    {
        std::cerr << "ERROR: opening laszip writer for " << filename << std::endl;
        return false;
    }

    std::cout << "Writing file " << filename << (compress ? " (compressed)" : " (uncompressed)") << std::endl;

    laszip_point *point;
    if (laszip_get_point_pointer(laszip_writer, &point))
    {
        std::cerr << "ERROR: getting point pointer" << std::endl;
        return false;
    }

    laszip_I64 p_count = 0;
    laszip_F64 coordinates[3];

    for (size_t i = 0; i < points_global.size(); i++)
    {
        const auto &p = points_global[i];
        point->intensity = static_cast<laszip_U16>(p.intensity);
        point->gps_time = p.timestamp * 1e9;
        p_count++;
        coordinates[0] = p.point.x();
        coordinates[1] = p.point.y();
        coordinates[2] = p.point.z();

        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            std::cerr << "ERROR: setting coordinates for point " << p_count << std::endl;
            return false;
        }

        if (laszip_write_point(laszip_writer))
        {
            std::cerr << "ERROR: writing point " << p_count << std::endl;
            return false;
        }
    }

    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        std::cerr << "ERROR: getting point count" << std::endl;
        return false;
    }

    std::cout << "Successfully written " << p_count << " points" << std::endl;

    if (laszip_close_writer(laszip_writer))
    {
        std::cerr << "ERROR: closing laszip writer" << std::endl;
        return false;
    }

    if (laszip_destroy(laszip_writer))
    {
        std::cerr << "ERROR: destroying laszip writer" << std::endl;
        return false;
    }

    std::cout << "saveLaz DONE" << std::endl;
    return true;
}
