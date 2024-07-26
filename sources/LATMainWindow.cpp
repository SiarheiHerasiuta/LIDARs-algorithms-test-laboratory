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

LATMainWindow::LATMainWindow(QWidget *p) : QMainWindow(p)
{
	createActions();
	createMenus();
	createStatusBar();
	createToolBars();
	createDockWindows();
	resize(640, 480);
}

LATMainWindow::~LATMainWindow()
{
}

void LATMainWindow::createActions()
{
	aboutAction = new QAction(tr("&About"), this);
	connect(aboutAction, SIGNAL(triggered()), this, SLOT(showAboutDialog()));

	aboutQtAction = new QAction(tr("About &Qt"), this);
	connect(aboutQtAction, SIGNAL(triggered()), qApp, SLOT(aboutQt()));

}

void LATMainWindow::createMenus()
{
	helpMenu = menuBar()->addMenu(tr("&Help"));
	helpMenu->addAction(aboutAction);
	helpMenu->addAction(aboutQtAction);
}

void LATMainWindow::createStatusBar()
{
	statusBar()->showMessage(tr("Ready to work"),0);
}

void LATMainWindow::createToolBars()
{
	consoleAction = new QAction(this);
	consoleAction->setEnabled(false);
	consoleAction->setObjectName(QStringLiteral("Console"));
	consoleAction->setIconText(consoleAction->objectName());
	connect(consoleAction,SIGNAL(triggered()),this,SLOT(onConsoleAction()));
}

void LATMainWindow::displayConsoleDockWindow()
{
	if (!consoleAction->isEnabled())
	{
		consoleDockWidget->show();
		consoleDockWidget->raise();
	}
}

void LATMainWindow::createDockWindows()
{
	if (!consoleAction->isEnabled())
	{
		consoleDockWidget = new QDockWidget(tr("LIDAR's algorithms test laboratory application console"), this);
		consoleDockWidget->setAllowedAreas(Qt::AllDockWidgetAreas);
		console = new LATConsoleDockWindow(consoleDockWidget);
		consoleDockWidget->setWidget(console);
		addDockWidget(Qt::LeftDockWidgetArea, consoleDockWidget);
		connect(consoleDockWidget, SIGNAL(visibilityChanged(bool)), this, SLOT(onConsoleAction(bool)));
		consoleDockWidget->show();
	}
}

void LATMainWindow::showStatusBarMessage( const QString &message )
{
	statusBar()->showMessage(message, 5000);
}

void LATMainWindow::showApplicationConsoleMessage( const QString &message )
{
	if (console != Q_NULLPTR)
	{
		if (consoleMessage.compare(message) != 0)
		{
			console->putStringData(message);
		}
		consoleMessage.clear();
		consoleMessage.append(message);
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
	consoleAction->setEnabled(!consoleAction->isEnabled());
	displayConsoleDockWindow();
}

void LATMainWindow::onConsoleAction(bool visible)
{
	consoleAction->setEnabled(!visible);
}

void LATMainWindow::showAboutDialog()
{
	LATAboutDialog aboutDialog(this);
	aboutDialog.resize(512, 384);
	aboutDialog.exec();
}
