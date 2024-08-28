#pragma once

#include <QDialog>

class LATAboutDialog final : public QDialog
{
	Q_OBJECT

public:
	LATAboutDialog(QWidget* parent = Q_NULLPTR);
	~LATAboutDialog();

private:
	Q_DISABLE_COPY(LATAboutDialog)
	Q_DISABLE_MOVE(LATAboutDialog)
};
