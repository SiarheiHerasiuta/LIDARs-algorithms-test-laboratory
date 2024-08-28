#pragma once

#include <QMainWindow>

#include "LATConsoleDockWindow.h"
#include "LATPointCloudViewer.h"
#include "tick_count.h"

using namespace tbb;

class LATMainWindow final : public QMainWindow
{
	Q_OBJECT

public:
	LATMainWindow(QWidget* parent = Q_NULLPTR);
	~LATMainWindow();

Q_SIGNALS:
	void onConsoleMessage(const QString& message, const tick_count& counter);
	void onPreviewPointCloud();
	void onFileOpenMenuVisibility();

private:
	
	Q_DISABLE_COPY(LATMainWindow)
	Q_DISABLE_MOVE(LATMainWindow)

	void createActions();
	void createMenus();
	void createStatusBar();

	void displayConsoleDockWindow();
	void displayPreviewDockWindow();

	QMenu* _fileMenu = Q_NULLPTR;
	QMenu* _previewMenu = Q_NULLPTR;
	QMenu* _debugMenu = Q_NULLPTR;
	QMenu* _helpMenu = Q_NULLPTR;

	QStatusBar* _statusbar = Q_NULLPTR;

	QAction* _aboutAction = Q_NULLPTR;
	QAction* _aboutQtAction = Q_NULLPTR;
	QAction* _fileOpenAction = Q_NULLPTR;
	QAction* _exitAction = Q_NULLPTR;
	QAction* _separatorAction = Q_NULLPTR;
	QAction* _consoleAction = Q_NULLPTR;
	QAction* _previewAction = Q_NULLPTR;

	LATConsoleDockWindow* _console = Q_NULLPTR;
	LATPointCloudViewer* _pointCloudViewer = Q_NULLPTR;

	QDockWidget* _consoleDockWidget = Q_NULLPTR;
	QDockWidget* _pointCloudViewerDockWidget = Q_NULLPTR;

	QString _consoleMessage;

private slots:
	void showAboutDialog();
	void showStatusBarMessage(const QString& message);
	void showApplicationConsoleMessage(const QString& message);
	void showApplicationConsoleAndStatusBarMessage(const QString& message, const tick_count& counter);
	void openFile();
	void previewPointCloud();

	void onConsoleAction();
	void onConsoleAction(bool checked);
	void onPreviewAction();
	void onPreviewAction(bool checked);
	void fileOpenMenuVisibility();

};
