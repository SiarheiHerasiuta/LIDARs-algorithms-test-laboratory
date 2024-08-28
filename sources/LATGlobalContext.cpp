#include "LATGlobalContext.h"

#include "Version.h"

LATGlobalContext* LATGlobalContext::globalLATContext = Q_NULLPTR;

LATGlobalContext::LATGlobalContext()
{
	globalLATTimeCounter = tbb::tick_count::now();
}

LATGlobalContext::~LATGlobalContext()
{

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
	return QString(LAT_VER_PRODUCTNAME_STR);
}

void LATGlobalContext::displayLogInformation(const QString& message)
{
	if (latWindow != Q_NULLPTR)
	{
		emit latWindow->onConsoleMessage(message, tick_count::now());
	}
}

void LATGlobalContext::initThreadsContext()
{
	if (globalSettings == Q_NULLPTR)
	{
		globalSettings = new QSettings("LATLaboratory.ini", QSettings::IniFormat);
	}
	if (controlThread == Q_NULLPTR)
	{
		controlThread = new LATControlThread;
		controlThread->start();
	}
	if (cloudDataSource == Q_NULLPTR)
	{
		cloudDataSource = new LATCloudDataSource;
	}
}

void LATGlobalContext::releaseThreadsContext()
{
	if (globalSettings != Q_NULLPTR)
	{
		delete globalSettings;
		globalSettings = Q_NULLPTR;
	}
	if (controlThread != Q_NULLPTR)
	{
		delete controlThread;
		controlThread = Q_NULLPTR;
	}
	if (cloudDataSource != Q_NULLPTR)
	{
		delete cloudDataSource;
		cloudDataSource = Q_NULLPTR;
	}
}
