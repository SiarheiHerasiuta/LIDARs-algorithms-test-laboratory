#pragma once

#include <QSettings>
#include <QObject>

#include "tick_count.h"

#include "LATMainWindow.h"

struct LATGlobalContext Q_DECL_FINAL : public QObject {
	
	Q_OBJECT

public:
	LATGlobalContext();
	~LATGlobalContext();

	static LATGlobalContext* globalLATContext;

	LATMainWindow* latWindow = Q_NULLPTR;

	QSettings* globalSettings = Q_NULLPTR;
	
	tbb::tick_count globalLATTimeCounter;

	QString licenseText() const;
	QString releaseDateText() const;
	QString versionProductNameText() const;

	void displayLogInformation(const QString& message);

	Q_DISABLE_COPY(LATGlobalContext)
};

#define globalLAT (*LATGlobalContext::globalLATContext)
