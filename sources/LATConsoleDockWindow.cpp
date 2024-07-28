#include <QScrollBar>
#include <QtCore/QDebug>
#include <QString>
#include <QDir>
#include <QDateTime>
#include <QTextStream>

#include "LATConsoleDockWindow.h"

LATConsoleDockWindow::LATConsoleDockWindow(QWidget* parent = Q_NULLPTR)
	: QPlainTextEdit(parent)
{
	document()->setMaximumBlockCount(100);
	auto consolePallette = palette();
	consolePallette.setColor(QPalette::Base, Qt::black);
	consolePallette.setColor(QPalette::Text, Qt::green);
	setPalette(consolePallette);

	auto id = QFontDatabase::addApplicationFont(":/resources/font/Hack-Regular.ttf");
	auto hackRegularFontFamily = QFontDatabase::applicationFontFamilies(id).at(0);
	QFont textFont(hackRegularFontFamily);
	textFont.setPointSizeF(10);

	setFont(textFont);
	setEnabled(true);
	setCursorWidth(0);
	_logFile.setFileName(QDir::currentPath() + QString("\\") + QString("LATLaboratory -") + QDateTime::currentDateTime().toString("[yyyy_MM_dd]-[hh_mm_ss_zzz].log"));
}

void LATConsoleDockWindow::keyPressEvent(QKeyEvent* event)
{
	switch (event->key())
	{
	case Qt::Key_Backspace:
	case Qt::Key_Left:
	case Qt::Key_Right:
	case Qt::Key_Up:
	case Qt::Key_Down:
		break;
	default:
		emit getData(event->text().toLocal8Bit());
	}
}

void LATConsoleDockWindow::mousePressEvent(QMouseEvent* event)
{
	Q_UNUSED(event)
		setFocus();
}

void LATConsoleDockWindow::mouseDoubleClickEvent(QMouseEvent* event)
{
	Q_UNUSED(event)
}

void LATConsoleDockWindow::contextMenuEvent(QContextMenuEvent* event)
{
	Q_UNUSED(event)
}

void LATConsoleDockWindow::putStringData(const QString& data)
{
	insertPlainText(data);
	auto scrollBar = verticalScrollBar();
	scrollBar->setValue(scrollBar->maximum());
	if (!_logFile.isOpen())
	{
		_logFile.open(QFile::ReadWrite | QFile::Text);
	}
	if (_logFile.isOpen())
	{
		QTextStream outputStream(&_logFile);
		outputStream.setCodec("UTF-8");
		outputStream << data;
	}
}
