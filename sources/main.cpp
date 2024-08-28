#include <clocale>  

#include <QApplication>
#include <QOperatingSystemVersion>
#include <QStyleFactory>
#include <QException>
#include <QMessageBox>
#include <QProcess>
#include <QDebug>

#include "LATGlobalContext.h"
#include "LATMainWindow.h"

int main(int argc, char** argv)
{
	setlocale(LC_CTYPE, "Russian");
	QT_REQUIRE_VERSION(argc, argv, "5.15.14");

	LATGlobalContext::globalLATContext = new LATGlobalContext();
	int executionResult = -1;
	static QString applicationFilePath;
	try
	{
		if (LATGlobalContext::globalLATContext != Q_NULLPTR)
		{
			LATGlobalContext::globalLATContext->initThreadsContext();
#if  defined(Q_OS_WIN)
			if (QOperatingSystemVersion::current() >= QOperatingSystemVersion::Windows10)
			{
				QApplication::setStyle(QStyleFactory::create("Fusion"));
			}
			QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

			QApplication a(argc, argv);
			LATMainWindow mw(Q_NULLPTR);
			applicationFilePath = QApplication::applicationFilePath();
			mw.show();
			executionResult = a.exec();
		}
		if (LATGlobalContext::globalLATContext != Q_NULLPTR)
		{
			LATGlobalContext::globalLATContext->releaseThreadsContext();
			delete LATGlobalContext::globalLATContext;
			LATGlobalContext::globalLATContext = Q_NULLPTR;
		}
	}
	catch (std::exception& e)
	{
		qCritical() << QString("STD Exception: %1").arg(e.what());
	}
	catch (...)
	{
		qCritical() << QString("Unhandled Exception");
	}
	return executionResult;
}