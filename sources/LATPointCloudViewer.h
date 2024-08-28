#pragma once

#include <QWidget>
#include <QFileDialog>
#include <QProgressDialog>
#include <QFuture>
#include <QFutureWatcher>
#include <QtConcurrentRun>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>


#include <boost/math/special_functions/round.hpp>

#include <vtkVersion.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>

class LATPointCloudViewer final : public QVTKOpenGLNativeWidget
{
	Q_OBJECT

public:
	explicit LATPointCloudViewer(QWidget* parent = Q_NULLPTR);
	~LATPointCloudViewer();

Q_SIGNALS:
	void onPreviewDataSource();

private:

	Q_DISABLE_COPY(LATPointCloudViewer)
	Q_DISABLE_MOVE(LATPointCloudViewer)

	boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;

	vtkSmartPointer<vtkRenderer> _renderer;
	vtkSmartPointer<vtkGenericOpenGLRenderWindow> _renderWindow;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloud;

	void prepareRender();

private slots:

	void previewDataSource();
};
