#include <QCoreApplication>

#include "LATGlobalContext.h"
#include "LATControlThread.h"

LATControlThread::LATControlThread(QObject* parent)
	: QThread(parent), _continueJob(false), _tasks(QVector<LATControlThreadTask>())
{
}

LATControlThread::~LATControlThread()
{
	_mutex.lock();
	_continueJob = false;
	_waitCondition.wakeOne();
	_mutex.unlock();
	wait();
}

void LATControlThread::run()
{
	_continueJob = true;
	while (_continueJob)
	{
		if ((!controlTasks().isEmpty()))
		{
			if (controlTasks()[0].taskType() == LATCONTROL_EMPTY_TASK)
			{
				removeControlTaskAt();
			}
			else if (controlTasks()[0].taskType() == LATCONTROL_PREPARE_POINT_CLOUD_DATA_SOURCE_FROM_PCD_FILE)
			{
				auto filePath = controlTasks()[0].taskData();
				if (globalLAT.cloudDataSource != Q_NULLPTR)
				{
					emit globalLAT.latWindow->onFileOpenMenuVisibility();
					globalLAT.cloudDataSource->parsePCDFile(filePath.toString());
				}
				removeControlTaskAt();
			}
			else if (controlTasks()[0].taskType() == LATCONTROL_PREVIEW_POINT_CLOUD_DATA_SOURCE)
			{
				if (globalLAT.latWindow != Q_NULLPTR)
				{
					emit globalLAT.latWindow->onPreviewPointCloud();
					emit globalLAT.latWindow->onFileOpenMenuVisibility();
				}
				removeControlTaskAt();
			}
		}
		else
		{
			QThread::usleep(1);
			qApp->processEvents();
		}
	}
}
