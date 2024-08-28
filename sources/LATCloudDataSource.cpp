#include "LATCloudDataSource.h"

#include "LATGlobalContext.h"

#include <QColor>

LATCloudDataSource::LATCloudDataSource(QObject* parent)
	:QObject(parent)
{
	_type = lATCloudDataSourceType::EMPTY;
	_initialFormat = lATCloudDataSourceInitialFormat::EMPTY;

	_cloudOneFrameForView.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

	_initialCloudXYZ.reset(new pcl::PointCloud<pcl::PointXYZ>);
	_initialCloudXYZI.reset(new pcl::PointCloud<pcl::PointXYZI>);
	_initialCloudXYZRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	_initialCloudXYZRGBA.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
}

LATCloudDataSource::~LATCloudDataSource()
{

}

lATCloudDataSourceType LATCloudDataSource::type() const
{
	return _type;
}

lATCloudDataSourceInitialFormat LATCloudDataSource::initialFormat() const
{
	return _initialFormat;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr LATCloudDataSource::cloudOneFrameForView() const
{
	return _cloudOneFrameForView;
}

void LATCloudDataSource::parsePCDFile(const QString& pcdFileName)
{
	pcl::PCLPointCloud2::Ptr pointCloud2(new pcl::PCLPointCloud2);

	globalLAT.displayLogInformation(QString(tr("Trying to read .pcd file - %1.").arg(pcdFileName)));

	auto returnStatus = pcl::io::loadPCDFile(pcdFileName.toStdString(), *pointCloud2);

	if (returnStatus != 0)
	{
		globalLAT.displayLogInformation(QString(tr("Error reading point cloud in %1 file.").arg(pcdFileName)));
		return;
	}

	auto pointCloudFields = pointCloud2->fields;
	QString fieldsFormat;
	for (auto& field : pointCloudFields)
	{
		fieldsFormat.append(QString::fromStdString(field.name));
	}

	if (fieldsFormat == "xyz")
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
		_initialCloudXYZ->clear();

		returnStatus = pcl::io::loadPCDFile(pcdFileName.toStdString(), *cloudXYZ);
		if (returnStatus != 0)
		{
			globalLAT.displayLogInformation(QString(tr("Error reading point cloud XYZ format in %1 file.").arg(pcdFileName)));
			return;
		}

		_cloudOneFrameForView->clear();

		for (auto& point : cloudXYZ->points)
		{
			auto pointXYZRGBA = pcl::PointXYZRGBA(point.x, point.y, point.z, 255, 255, 255, 1);
			_cloudOneFrameForView->push_back(pointXYZRGBA);
		}

		globalLAT.displayLogInformation(QString(tr("%1 file is in XYZ format. We convert all %2 points to XYZRGBA format for preview where RGBA = {255, 255, 255, 1}.").arg(pcdFileName).arg(_cloudOneFrameForView->size())));

		_initialFormat = lATCloudDataSourceInitialFormat::ONE_FRAME_XYZ;
		_type = lATCloudDataSourceType::ONE_FRAME_PCD;
		return;
	}

	if (fieldsFormat == "xyzintensity")
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);
		_initialCloudXYZI->clear();

		returnStatus = pcl::io::loadPCDFile(pcdFileName.toStdString(), *cloudXYZI);
		if (returnStatus != 0)
		{
			globalLAT.displayLogInformation(QString(tr("Error reading point cloud XYZ format in %1 file.").arg(pcdFileName)));
			return;
		}

		_cloudOneFrameForView->clear();
		QColor colorRGB;

		for (auto& point : cloudXYZI->points)
		{
			colorRGB = QColor::fromRgba(qRgba(point.intensity, point.intensity, point.intensity, 1));
			if (pointCloud2->fields[3].datatype == pcl::PCLPointField::PointFieldTypes::FLOAT32)
			{
				colorRGB = QColor::fromHslF(point.intensity, 1.0f, 0.5f);
			}
			else if (pointCloud2->fields[3].datatype == pcl::PCLPointField::PointFieldTypes::UINT8)
			{
				colorRGB = QColor::fromHslF((point.intensity / 255.0f), 1.0f, 0.5f);
			}

			auto pointXYZRGBA = pcl::PointXYZRGBA(point.x, point.y, point.z, colorRGB.red(), colorRGB.green(), colorRGB.blue(), 1);
			_cloudOneFrameForView->push_back(pointXYZRGBA);
		}

		globalLAT.displayLogInformation(QString(tr("%1 file is in XYZI format. We convert all %2 points to XYZRGBA format for preview.").arg(pcdFileName).arg(_cloudOneFrameForView->size())));
		pcl::copyPointCloud(*cloudXYZI, *_initialCloudXYZI);

		_initialFormat = lATCloudDataSourceInitialFormat::ONE_FRAME_XYZI;
		_type = lATCloudDataSourceType::ONE_FRAME_PCD;
		return;
	}

	if (fieldsFormat == "xyzrgba")
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA(new pcl::PointCloud<pcl::PointXYZRGBA>);
		_cloudOneFrameForView->clear();
		_initialCloudXYZRGBA->clear();

		returnStatus = pcl::io::loadPCDFile(pcdFileName.toStdString(), *_cloudOneFrameForView);
		if (returnStatus != 0)
		{
			globalLAT.displayLogInformation(QString(tr("Error reading point cloud XYZRGBA format in %1 file.").arg(pcdFileName)));
			return;
		}

		globalLAT.displayLogInformation(QString(tr("%1 file is in XYZRGBA format. %2 points in file.").arg(pcdFileName).arg(_cloudOneFrameForView->size())));

		pcl::copyPointCloud(*cloudXYZRGBA, *_initialCloudXYZRGBA);

		_initialFormat = lATCloudDataSourceInitialFormat::ONE_FRAME_XYZRGBA;
		_type = lATCloudDataSourceType::ONE_FRAME_PCD;
		return;

	}

	if (fieldsFormat == "xyzrgb")
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
		_cloudOneFrameForView->clear();
		_initialCloudXYZRGB->clear();

		returnStatus = pcl::io::loadPCDFile(pcdFileName.toStdString(), *cloudXYZRGB);
		if (returnStatus != 0)
		{
			globalLAT.displayLogInformation(QString(tr("Error reading point cloud XYZRGB format in %1 file.").arg(pcdFileName)));
			return;
		}

		globalLAT.displayLogInformation(QString(tr("%1 file is in XYZRGB format. We convert all %2 points to XYZRGBA format for preview where A = 1.").arg(pcdFileName).arg(_cloudOneFrameForView->size())));
		
		pcl::copyPointCloud(*cloudXYZRGB, *_cloudOneFrameForView);
		pcl::copyPointCloud(*cloudXYZRGB, *_initialCloudXYZRGB);

		_initialFormat = lATCloudDataSourceInitialFormat::ONE_FRAME_XYZRGB;
		_type = lATCloudDataSourceType::ONE_FRAME_PCD;
		return;
	}

	globalLAT.displayLogInformation(QString(tr("Error reading point cloud in %1 file. FIELDS format '%2' in .pcd file is not supported").arg(pcdFileName).arg(fieldsFormat)));
}
