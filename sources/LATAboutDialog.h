#pragma once

#include <QObject>
#include <QDialog>

class LATAboutDialog : public QDialog
{
	Q_OBJECT

public:
	LATAboutDialog(QWidget* parent);

private:
	Q_DISABLE_COPY(LATAboutDialog)
};
