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

LATMainWindow::LATMainWindow(QWidget *parent) 
 : QMainWindow(parent)
{
	qRegisterMetaType<tick_count>("tick_count");

	connect(this, SIGNAL(onConsoleMessage(const QString&, const tick_count&)), this, SLOT(showApplicationConsoleAndStatusBarMessage(const QString&, const tick_count&)));
	
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
	_fileOpenAction = new QAction(tr("&Open..."), this);
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

	_debugMenu = menuBar()->addMenu(tr("&Debug"));
	_consoleAction->setChecked(true);
	_debugMenu->addAction(_consoleAction);
	
	displayConsoleDockWindow();

	_helpMenu = menuBar()->addMenu(tr("&Help"));
	_helpMenu->addAction(_aboutAction);
	_helpMenu->addAction(_aboutQtAction);
}

void LATMainWindow::createStatusBar()
{
	statusBar()->showMessage(tr("Ready to work"),0);
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
			addDockWidget(Qt::LeftDockWidgetArea, _consoleDockWidget);
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

void LATMainWindow::showStatusBarMessage( const QString &message )
{
	statusBar()->showMessage(message, 5000);
}

void LATMainWindow::showApplicationConsoleMessage( const QString &message )
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

void LATMainWindow::showApplicationConsoleAndStatusBarMessage(const QString &message, const tick_count &counter)
{
	tbb::tick_count::interval_t interval = counter - globalLAT.globalLATTimeCounter;

	auto timimngMessage = QString("[%1]-[%2]-[%3]\n").arg(QString::number(interval.seconds(), 'g'), 16).arg(QDateTime::currentDateTime().toString("hh:mm:ss")).arg(message);
	showStatusBarMessage(timimngMessage);
	showApplicationConsoleMessage(timimngMessage);
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

void LATMainWindow::showAboutDialog()
{
	LATAboutDialog aboutDialog(this);
	aboutDialog.resize(512, 384);
	aboutDialog.exec();
}
