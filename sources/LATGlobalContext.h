#pragma once

#include <QSettings>
#include <QObject>

#include "tick_count.h"

#include "LATMainWindow.h"
#include "LATCloudDataSource.h"
#include "LATControlThread.h"

struct LATGlobalContext final : public QObject {

	Q_OBJECT

public:
	LATGlobalContext();
	~LATGlobalContext();

	static LATGlobalContext* globalLATContext;

	LATMainWindow* latWindow = Q_NULLPTR;

	QSettings* globalSettings = Q_NULLPTR;

	LATControlThread* controlThread = Q_NULLPTR;
	LATCloudDataSource* cloudDataSource = Q_NULLPTR;

	tbb::tick_count globalLATTimeCounter;

	void initThreadsContext();
	void releaseThreadsContext();

	QString licenseText() const;
	QString releaseDateText() const;
	QString versionProductNameText() const;

	void displayLogInformation(const QString& message);

	Q_DISABLE_COPY(LATGlobalContext)
	Q_DISABLE_MOVE(LATGlobalContext)
};

#define globalLAT (*LATGlobalContext::globalLATContext)
