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

double GeoTiffReader::get_value(double lon, double lat) const {
    return impl->get_value(lon, lat);
}

} // namespace RoutingKit
