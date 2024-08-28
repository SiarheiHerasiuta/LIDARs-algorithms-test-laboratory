#include <QVBoxLayout>
#include <QGroupBox>

#include "LATGlobalContext.h"
#include "LATPointCloudViewer.h"
#include "LATCloudDataSource.h"

#include "Eigen/Dense"

LATPointCloudViewer::LATPointCloudViewer(QWidget* parent)
	: QVTKOpenGLNativeWidget(parent)
{
	
	QSurfaceFormat fmt;
	fmt.setRenderableType(QSurfaceFormat::OpenGL);
	fmt.setVersion(4, 6);
	fmt.setProfile(QSurfaceFormat::CoreProfile);
	fmt.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
	fmt.setRedBufferSize(1);
	fmt.setGreenBufferSize(1);
	fmt.setBlueBufferSize(1);
	fmt.setDepthBufferSize(1);
	fmt.setStencilBufferSize(0);
	fmt.setAlphaBufferSize(1);
	fmt.setStereo(false);
	fmt.setSamples(vtkOpenGLRenderWindow::GetGlobalMaximumNumberOfMultiSamples());

	QSurfaceFormat::setDefaultFormat(fmt);
	
	connect(this, SIGNAL(onPreviewDataSource()), this, SLOT(previewDataSource()));

	prepareRender();

	_viewer->setShowFPS(true);
}

LATPointCloudViewer::~LATPointCloudViewer()
{
}

void LATPointCloudViewer::previewDataSource()
{
	if (globalLAT.cloudDataSource != Q_NULLPTR)
	{
		if (globalLAT.cloudDataSource->type() == lATCloudDataSourceType::ONE_FRAME_PCD)
		{
			prepareRender();

			pcl::copyPointCloud(*globalLAT.cloudDataSource->cloudOneFrameForView(), *_cloud);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> handler(_cloud);
			_viewer->addPointCloud(_cloud, handler, "PCD file cloud");
			
			this->update();
		}
	}
}

void LATPointCloudViewer::prepareRender()
{
	_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	_viewer.reset(new pcl::visualization::PCLVisualizer("preview", false));
	_viewer->setBackgroundColor(0.1, 0.1, 0.1);

	_renderer = vtkSmartPointer<vtkRenderer>::New();
	_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	_renderWindow->AddRenderer(_renderer);

	_viewer.reset(new pcl::visualization::PCLVisualizer(_renderer, _renderWindow, "preview", false));
	this->setRenderWindow(_viewer->getRenderWindow());
	_viewer->setupInteractor(this->interactor(), this->renderWindow());
}


