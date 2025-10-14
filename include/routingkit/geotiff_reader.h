#ifndef ROUTING_KIT_GEOTIFF_READER_H
#define ROUTING_KIT_GEOTIFF_READER_H

#include <string>

namespace RoutingKit {

class GeoTiffReader {
public:
    GeoTiffReader(const std::string& file_path);
    ~GeoTiffReader();

    double get_value(double lon, double lat) const;
    bool line_intersects(double lon1, double lat1, double lon2, double lat2) const;

private:
    class GeoTiffReaderImpl;
    GeoTiffReaderImpl* impl;
};

} // namespace RoutingKit

#endif // ROUTING_KIT_GEOTIFF_READER_H
