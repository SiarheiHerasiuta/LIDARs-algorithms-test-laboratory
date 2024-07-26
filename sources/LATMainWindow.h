#pragma once

#include <QMainWindow>

#include "LATConsoleDockWindow.h"
#include "tick_count.h"

using namespace tbb;

class LATMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	LATMainWindow(QWidget *parent);
	~LATMainWindow();

	public slots:
		void showAboutDialog();
		void showStatusBarMessage(const QString& message);
		void showApplicationConsoleMessage(const QString& message);
		void showApplicationConsoleAndStatusBarMessage(const QString& message, const tick_count& counter);

private:
	Q_DISABLE_COPY(LATMainWindow)
	
	void createActions();
	void createMenus();
	void createStatusBar();
	void createToolBars();
	void createDockWindows();

	void displayConsoleDockWindow();

	QMenu* helpMenu;

	QStatusBar* statusbar;

	QAction* aboutAction;
	QAction* aboutQtAction;

	QAction* consoleAction;

	LATConsoleDockWindow* console;

	QDockWidget* consoleDockWidget;

	QString consoleMessage;

	private slots:
		void onConsoleAction();
		void onConsoleAction(bool visible);
};
