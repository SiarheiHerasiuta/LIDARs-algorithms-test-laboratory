#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrent>

#include "LATGlobalContext.h"

#include "Version.h"

LATGlobalContext * LATGlobalContext::globalLATContext = Q_NULLPTR;

LATGlobalContext::LATGlobalContext() 
{
	globalSettings = new QSettings("LATLaboratory.ini", QSettings::IniFormat);
	globalLATTimeCounter = tbb::tick_count::now();
}

LATGlobalContext::~LATGlobalContext()
{
	if (globalSettings != Q_NULLPTR)
	{
		delete globalSettings;
		globalSettings = Q_NULLPTR;
	}
}

QString LATGlobalContext::licenseText() const
{
	return QString(licenseLATLaboratory);
}

QString LATGlobalContext::releaseDateText() const
{
	return QString(LAT_RELEASE);
}

QString LATGlobalContext::versionProductNameText() const
{
	return QString(VER_PRODUCTNAME_STR);
}

void LATGlobalContext::displayLogInformation(const QString& message)
{
	if (latWindow != Q_NULLPTR)
	{
		emit latWindow->onConsoleMessage(message, tick_count::now());
	}
}
