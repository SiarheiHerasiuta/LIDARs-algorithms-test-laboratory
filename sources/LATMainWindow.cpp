#include <QApplication>
#include <QDebug>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QVBoxLayout>
#include <QStatusBar>
#include <QDockWidget>
#include <QFileDialog>
#include <QDateTime>

#include "LATGlobalContext.h"

#include "LATMainWindow.h"
#include "LATConsoleDockWindow.h"
#include "LATAboutDialog.h"
#include "LATPointCloudViewer.h"

LATMainWindow::LATMainWindow(QWidget* parent)
	: QMainWindow(parent)
{
	qRegisterMetaType<tick_count>("tick_count");

	connect(this, SIGNAL(onConsoleMessage(const QString&, const tick_count&)), this, SLOT(showApplicationConsoleAndStatusBarMessage(const QString&, const tick_count&)));
	connect(this, SIGNAL(onPreviewPointCloud()), this, SLOT(previewPointCloud()));
	connect(this, SIGNAL(onFileOpenMenuVisibility()), this, SLOT(fileOpenMenuVisibility()));

	createActions();
	createMenus();
	createStatusBar();
	resize(640, 480);
	globalLAT.latWindow = this;
}

LATMainWindow::~LATMainWindow()
{
}

void LATMainWindow::createActions()
{
	_fileOpenAction = new QAction(tr("&Open point cloud file data source"), this);
	_fileOpenAction->setShortcuts(QKeySequence::Open);
	connect(_fileOpenAction, SIGNAL(triggered()), this, SLOT(openFile()));

	_aboutAction = new QAction(tr("&About"), this);
	connect(_aboutAction, SIGNAL(triggered()), this, SLOT(showAboutDialog()));

	_aboutQtAction = new QAction(tr("About &Qt"), this);
	connect(_aboutQtAction, SIGNAL(triggered()), qApp, SLOT(aboutQt()));

	_consoleAction = new QAction(tr("LIDAR's algorithms test laboratory debug &console"), this);
	_consoleAction->setCheckable(true);
	_consoleAction->setStatusTip(tr("Show/Hide the Console docking window"));
	connect(_consoleAction, SIGNAL(triggered()), this, SLOT(onConsoleAction()));

	_previewAction = new QAction(tr("LIDAR's algorithms test laboratory &preview window"), this);
	_previewAction->setCheckable(true);
	_previewAction->setStatusTip(tr("Show/Hide the Console docking window"));
	connect(_previewAction, SIGNAL(triggered()), this, SLOT(onPreviewAction()));

	_exitAction = new QAction(tr("E&xit"), this);
	_exitAction->setShortcuts(QKeySequence::Quit);
	connect(_exitAction, SIGNAL(triggered()), qApp, SLOT(closeAllWindows()));
}

void LATMainWindow::createMenus()
{
	_fileMenu = menuBar()->addMenu(tr("&File"));
	_fileMenu->addAction(_fileOpenAction);
	_separatorAction = _fileMenu->addSeparator();
	_fileMenu->addSeparator();
	_fileMenu->addAction(_exitAction);

	_previewMenu = menuBar()->addMenu(tr("&Preview"));
	_previewAction->setChecked(true);
	_previewMenu->addAction(_previewAction);

	_debugMenu = menuBar()->addMenu(tr("&Debug"));
	_consoleAction->setChecked(true);
	_debugMenu->addAction(_consoleAction);

	_helpMenu = menuBar()->addMenu(tr("&Help"));
	_helpMenu->addAction(_aboutAction);
	_helpMenu->addAction(_aboutQtAction);

	displayConsoleDockWindow();
	displayPreviewDockWindow();
}

void LATMainWindow::createStatusBar()
{
	statusBar()->showMessage(tr("Ready to work"), 0);
}

void LATMainWindow::displayConsoleDockWindow()
{
	if (_consoleAction->isChecked())
	{
		if (_consoleDockWidget == Q_NULLPTR)
		{
			_consoleDockWidget = new QDockWidget(tr("LIDAR's algorithms test laboratory application console"), this);
			_consoleDockWidget->setAllowedAreas(Qt::AllDockWidgetAreas);
			_console = new LATConsoleDockWindow(_consoleDockWidget);
			_consoleDockWidget->setWidget(_console);
			addDockWidget(Qt::BottomDockWidgetArea, _consoleDockWidget);
			connect(_consoleDockWidget, SIGNAL(visibilityChanged(bool)), this, SLOT(onConsoleAction(bool)));
		}
		if (_consoleDockWidget != Q_NULLPTR)
		{
			_consoleDockWidget->show();
			_consoleDockWidget->raise();
		}
	}
	else
	{
		if (_consoleDockWidget != Q_NULLPTR)
		{
			_consoleDockWidget->hide();
		}
	}
}

void LATMainWindow::displayPreviewDockWindow()
{
	if (_previewAction->isChecked())
	{
		if (_pointCloudViewerDockWidget == Q_NULLPTR)
		{
			_pointCloudViewerDockWidget = new QDockWidget(tr("LIDAR's algorithms test laboratory interactive preview"), this);
			_pointCloudViewerDockWidget->setAllowedAreas(Qt::AllDockWidgetAreas);
			_pointCloudViewer = new LATPointCloudViewer(_pointCloudViewerDockWidget);
			_pointCloudViewerDockWidget->setWidget(_pointCloudViewer);
			addDockWidget(Qt::TopDockWidgetArea, _pointCloudViewerDockWidget);
			connect(_pointCloudViewerDockWidget, SIGNAL(visibilityChanged(bool)), this, SLOT(onPreviewAction(bool)));
		}
		if (_pointCloudViewerDockWidget != Q_NULLPTR)
		{
			_pointCloudViewerDockWidget->show();
			_pointCloudViewerDockWidget->raise();
		}
	}
	else
	{
		if (_pointCloudViewerDockWidget != Q_NULLPTR)
		{
			_pointCloudViewerDockWidget->hide();
		}
	}
}

void LATMainWindow::showStatusBarMessage(const QString& message)
{
	statusBar()->showMessage(message, 5000);
}

void LATMainWindow::showApplicationConsoleMessage(const QString& message)
{
	if (_console != Q_NULLPTR)
	{
		if (_consoleMessage.compare(message) != 0)
		{
			_console->putStringData(message);
		}
		_consoleMessage.clear();
		_consoleMessage.append(message);
	}
}

void LATMainWindow::showApplicationConsoleAndStatusBarMessage(const QString& message, const tick_count& counter)
{
	tbb::tick_count::interval_t interval = counter - globalLAT.globalLATTimeCounter;

	auto timimngMessage = QString("[%1]-[%2]-[%3]\n").arg(QString::number(interval.seconds(), 'g'), 16).arg(QDateTime::currentDateTime().toString("hh:mm:ss")).arg(message);
	showStatusBarMessage(timimngMessage);
	showApplicationConsoleMessage(timimngMessage);
}

void LATMainWindow::openFile()
{
	auto filePath = QFileDialog::getOpenFileName(this, tr("Open point cloud"), QDir::currentPath(), tr("Point cloud data format (*.pcd)"));

	if (filePath.isEmpty())
		return;

	QFileInfo fileInfo(filePath);
	auto fileSuffix = fileInfo.suffix().toLower();

	if (fileSuffix == "pcd")
	{
		globalLAT.controlThread->addControlTask(LATControlThreadTask(LATCONTROL_PREPARE_POINT_CLOUD_DATA_SOURCE_FROM_PCD_FILE, filePath));
		globalLAT.controlThread->addControlTask(LATControlThreadTask(LATCONTROL_PREVIEW_POINT_CLOUD_DATA_SOURCE));
	}
}

void LATMainWindow::previewPointCloud()
{
	if (_pointCloudViewer != Q_NULLPTR)
	{
		emit _pointCloudViewer->onPreviewDataSource();
	}
}

void LATMainWindow::onConsoleAction()
{
	_consoleAction->setChecked(_consoleAction->isChecked());
	displayConsoleDockWindow();
}

void LATMainWindow::onConsoleAction(bool checked)
{
	_consoleAction->setChecked(checked);
}

void LATMainWindow::onPreviewAction()
{
	_previewAction->setChecked(_previewAction->isChecked());
	displayPreviewDockWindow();
}

void LATMainWindow::onPreviewAction(bool checked)
{
	_previewAction->setChecked(checked);
}

void LATMainWindow::fileOpenMenuVisibility()
{
	_fileOpenAction->setEnabled(!_fileOpenAction->isEnabled());
}

void LATMainWindow::showAboutDialog()
{
	LATAboutDialog aboutDialog(this);
	aboutDialog.resize(512, 384);
	aboutDialog.exec();
}
