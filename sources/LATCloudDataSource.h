#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include <QObject>

enum class lATCloudDataSourceType {
	EMPTY,
	ONE_FRAME_PCD,
	MULTIPLE_FRAMES_CSV_LIVOX2
};

enum class lATCloudDataSourceInitialFormat {
	EMPTY,
	ONE_FRAME_XYZ,
	ONE_FRAME_XYZI,
	ONE_FRAME_XYZRGB,
	ONE_FRAME_XYZRGBA
};

class LATCloudDataSource final : public QObject
{
	Q_OBJECT

public:
	explicit LATCloudDataSource(QObject* parent = Q_NULLPTR);
	~LATCloudDataSource();

	lATCloudDataSourceType type() const;
	lATCloudDataSourceInitialFormat initialFormat() const;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOneFrameForView() const;

	void parsePCDFile(const QString& fileName);

private:
	lATCloudDataSourceType _type;
	lATCloudDataSourceInitialFormat _initialFormat;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloudOneFrameForView;

	pcl::PointCloud<pcl::PointXYZ>::Ptr _initialCloudXYZ;
	pcl::PointCloud<pcl::PointXYZI>::Ptr _initialCloudXYZI;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _initialCloudXYZRGB;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _initialCloudXYZRGBA;

	Q_DISABLE_COPY(LATCloudDataSource)
	Q_DISABLE_MOVE(LATCloudDataSource)
};
