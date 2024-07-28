#include <QTabWidget>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>

#include "LATGlobalContext.h"
#include "LATAboutDialog.h"

LATAboutDialog::LATAboutDialog(QWidget* parent = Q_NULLPTR) 
	: QDialog(parent) 
{

	setWindowTitle(tr("About LIDAR's algorithms test laboratory"));

	auto qtwTab = new QTabWidget(this);
	auto vblMain = new QVBoxLayout(this);

	auto qtLicense = new QTextEdit(qtwTab);
	qtLicense->setReadOnly(true);
	qtLicense->setPlainText(globalLAT.licenseText());

	auto about=new QWidget(qtwTab);

	auto text=new QLabel(about);
	text->setOpenExternalLinks(true);
	text->setText(tr(
		"<h3>%1 (%2)</h3>"
		"<p>Copyright %4 Siarhei Herasiuta <a href='mailto:contacts@robotics.by'>contacts@robotics.by</a></p>"
		"<p>LIDAR's algorithms test laboratory is a point cloud processing and analysis software.</p>"
		"<p>You can read latest project information and download sources from <tt><a href=\"https://github.com/SiarheiHerasiuta/LIDARs-algorithms-test-laboratory\">Github</a></tt> portal.</p>"
		"<p><tt><a href=\"%3\">%3</a></tt></p>"
		).arg(globalLAT.versionProductNameText()).arg(globalLAT.releaseDateText()).arg(QLatin1String("http://www.robotics.by/")).arg(QLatin1String("2024")));
	auto qhbl = new QHBoxLayout(about);
	qhbl->addWidget(text);

	qtwTab->addTab(about, tr("&About LIDAR's algorithms test laboratory"));
	qtwTab->addTab(qtLicense, tr("&License"));
	
	vblMain->addWidget(qtwTab);
}