#pragma once

#include <QVector>
#include <QVariant>
#include <QReadWriteLock>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>

enum lATControlThreadTaskType {
	LATCONTROL_EMPTY_TASK,
	LATCONTROL_PREPARE_POINT_CLOUD_DATA_SOURCE_FROM_PCD_FILE,
	LATCONTROL_PREVIEW_POINT_CLOUD_DATA_SOURCE
};

typedef struct lATControlThreadTask final {

public:
	inline  lATControlThreadTask() Q_DECL_NOTHROW { _type = LATCONTROL_EMPTY_TASK; }
	inline  lATControlThreadTask(const  lATControlThreadTaskType& type, const QVariant& data = QVariant()) Q_DECL_NOTHROW {
		_type = type;
		_data = data;
	}
	inline ~lATControlThreadTask() = default;

	QVariant taskData() { return _data; }
	void taskData(const QVariant& data) { _data = data; }

	lATControlThreadTaskType taskType() { return _type; }
	void taskType(const  lATControlThreadTaskType& type) { _type = type; }

private:
	lATControlThreadTaskType _type;
	QVariant _data;

} LATControlThreadTask;

class LATControlThread final : public QThread
{
	Q_OBJECT

public:
	explicit LATControlThread(QObject* parent = nullptr);
	~LATControlThread();

	void run() Q_DECL_OVERRIDE;

	bool controlThreadContinueJob() { return this->_continueJob; }
	void controlThreadContinueJob(const bool& continueJob) { this->_continueJob = continueJob; }

	QVector<LATControlThreadTask> controlTasks() { return this->_tasks; }
	void controlTasks(QVector<LATControlThreadTask> tasks) { this->_tasks = tasks; }
	void addControlTask(LATControlThreadTask task) { this->_tasks.append(task); }
	void removeControlTaskAt(const int& index = 0) { this->_tasks.removeAt(index); }

private:
	Q_DISABLE_COPY(LATControlThread)
	Q_DISABLE_MOVE(LATControlThread)
	
	bool _continueJob;
	QVector<LATControlThreadTask> _tasks;
	QMutex _mutex;
	QWaitCondition _waitCondition;
	
};
