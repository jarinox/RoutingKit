#include <routingkit/geotiff_reader.h>
#include <gdal_priv.h>
#include <stdexcept>

namespace RoutingKit {

const double NO_DATA = -9999.0;

class GeoTiffReader::GeoTiffReaderImpl {
public:
    GeoTiffReaderImpl(const std::string& file_path) {
        GDALAllRegister();
        dataset = (GDALDataset*) GDALOpen(file_path.c_str(), GA_ReadOnly);
        if (dataset == nullptr) {
            throw std::runtime_error("Could not open GeoTIFF file: " + file_path);
        }

        if (dataset->GetGeoTransform(geotransform) != CE_None) {
            GDALClose(dataset);
            throw std::runtime_error("Could not get geotransform from GeoTIFF file.");
        }

        if (dataset->GetRasterCount() < 1) {
            GDALClose(dataset);
            throw std::runtime_error("No raster bands found in GeoTIFF file.");
        }
        band = dataset->GetRasterBand(1);
    }

    ~GeoTiffReaderImpl() {
        if (dataset != nullptr) {
            GDALClose(dataset);
        }
    }

    /*
     *  Check if the line between (lon1, lat1) and (lon2, lat2) intersects any pixel which should be avoid
     *  Uses Bresenhams algorithm to determine which pixels are intersected by the line
     */ 
    bool line_intersects(double lon1, double lat1, double lon2, double lat2) const {
        double inv_geotransform[6];
        if (!GDALInvGeoTransform(geotransform, inv_geotransform)) {
            throw std::runtime_error("Cannot invert geotransform.");
        }

        double col1 = inv_geotransform[0] + inv_geotransform[1] * lon1 + inv_geotransform[2] * lat1;
        double row1 = inv_geotransform[3] + inv_geotransform[4] * lon1 + inv_geotransform[5] * lat1;

        double col2 = inv_geotransform[0] + inv_geotransform[1] * lon2 + inv_geotransform[2] * lat2;
        double row2 = inv_geotransform[3] + inv_geotransform[4] * lon2 + inv_geotransform[5] * lat2;

        int x1 = static_cast<int>(col1);
        int y1 = static_cast<int>(row1);
        int x2 = static_cast<int>(col2);
        int y2 = static_cast<int>(row2);

        int dx = abs(x2 - x1);
        int sx = x1 < x2 ? 1 : -1;
        int dy = -abs(y2 - y1);
        int sy = y1 < y2 ? 1 : -1;
        int err = dx + dy;
        int e2;

        while (true) {
            if (x1 >= 0 && x1 < band->GetXSize() && y1 >= 0 && y1 < band->GetYSize()) {
                double value;
                if (band->RasterIO(GF_Read, x1, y1, 1, 1, &value, 1, 1, GDT_Float64, 0, 0) != CE_None) {
                    throw std::runtime_error("Failed to read raster data.");
                }
                if (value == 1.0) {
                    return true;
                }
            }

            if (x1 == x2 && y1 == y2) break;
            e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x1 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y1 += sy;
            }
        }

        return false;
    }

    double get_value(double lon, double lat) const {
        double inv_geotransform[6];
        if (!GDALInvGeoTransform(geotransform, inv_geotransform)) {
            throw std::runtime_error("Cannot invert geotransform.");
        }

        double col = inv_geotransform[0] + inv_geotransform[1] * lon + inv_geotransform[2] * lat;
        double row = inv_geotransform[3] + inv_geotransform[4] * lon + inv_geotransform[5] * lat;

        int x = static_cast<int>(col);
        int y = static_cast<int>(row);

        if (x < 0 || x >= band->GetXSize() || y < 0 || y >= band->GetYSize()) {
            return NO_DATA;
        }

        double value;
        if (band->RasterIO(GF_Read, x, y, 1, 1, &value, 1, 1, GDT_Float64, 0, 0) != CE_None) {
            throw std::runtime_error("Failed to read raster data.");
        }

        return value;
    }

private:
    GDALDataset* dataset;
    double geotransform[6];
    GDALRasterBand* band;
};

GeoTiffReader::GeoTiffReader(const std::string& file_path) : impl(new GeoTiffReaderImpl(file_path)) {}

GeoTiffReader::~GeoTiffReader() {
    delete impl;
}

bool GeoTiffReader::line_intersects(double lon1, double lat1, double lon2, double lat2) const {
    return impl->line_intersects(lon1, lat1, lon2, lat2);
}

double GeoTiffReader::get_value(double lon, double lat) const {
    return impl->get_value(lon, lat);
}

} // namespace RoutingKit
