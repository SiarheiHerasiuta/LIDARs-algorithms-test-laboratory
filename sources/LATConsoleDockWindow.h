#pragma once

#include <QPlainTextEdit>
#include <QFile>

class LATConsoleDockWindow : public QPlainTextEdit
{
	Q_OBJECT

public:
	explicit LATConsoleDockWindow(QWidget* parent = nullptr);

public slots:
	void putStringData(const QString& data);

signals:
	void getData(const QByteArray& data);

protected:
	virtual void keyPressEvent(QKeyEvent* event);
	virtual void mousePressEvent(QMouseEvent* event);
	virtual void mouseDoubleClickEvent(QMouseEvent* event);
	virtual void contextMenuEvent(QContextMenuEvent* event);

private:
	QFile _logFile;
};
