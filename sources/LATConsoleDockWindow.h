#pragma once

#include <QPlainTextEdit>
#include <QFile>

class LATConsoleDockWindow : public QPlainTextEdit
{
	Q_OBJECT

public:
	explicit LATConsoleDockWindow(QWidget* parent = Q_NULLPTR);

public slots:
	void putStringData(const QString& data);

signals:
	void getData(const QByteArray& data);

protected:
	virtual void keyPressEvent(QKeyEvent* event = Q_NULLPTR);
	virtual void mousePressEvent(QMouseEvent* event = Q_NULLPTR);
	virtual void mouseDoubleClickEvent(QMouseEvent* event = Q_NULLPTR);
	virtual void contextMenuEvent(QContextMenuEvent* event = Q_NULLPTR);

private:
	QFile _logFile;
};
