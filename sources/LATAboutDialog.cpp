#include <QTabWidget>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>

#include "LATGlobalContext.h"
#include "LATAboutDialog.h"

#include <boost/version.hpp>
#include <vtkVersion.h>
#include <pcl/pcl_config.h>
#include <Eigen/src/Core/util/Macros.h>

LATAboutDialog::LATAboutDialog(QWidget* parent)
	: QDialog(parent)
{
	setWindowTitle(tr("About LIDAR's algorithms test laboratory"));

	auto qtwTab = new QTabWidget(this);
	auto vblMain = new QVBoxLayout(this);

	auto qtLicense = new QTextEdit(qtwTab);
	qtLicense->setReadOnly(true);
	qtLicense->setPlainText(globalLAT.licenseText());

	auto about = new QWidget(qtwTab);
	auto openSource = new QWidget(qtwTab);

	auto textAbout = new QLabel(about);
	textAbout->setOpenExternalLinks(true);
	textAbout->setText(tr(
		"<h3>%1 (%2)</h3>"
		"<p>Copyright %4 Siarhei Herasiuta <a href='mailto:contacts@robotics.by'>contacts@robotics.by</a></p>"
		"<p>LIDAR's algorithms test laboratory is a point cloud processing and analysis software.</p>"
		"<p>You can read latest project information and download sources from <tt><a href=\"https://github.com/SiarheiHerasiuta/LIDARs-algorithms-test-laboratory\">Github</a></tt> portal.</p>"
		"<p><tt><a href=\"%3\">%3</a></tt></p>"
	).arg(globalLAT.versionProductNameText()).arg(globalLAT.releaseDateText()).arg(QLatin1String("http://www.robotics.by/")).arg(QLatin1String("2024")));
	auto layoutAbout = new QHBoxLayout(about);
	layoutAbout->addWidget(textAbout);

	auto textOpenSource = new QLabel(openSource);
	textOpenSource->setOpenExternalLinks(true);
	textOpenSource->setText(tr(
		"<h3>Major open source libraries with versions:</h3>"
		"<p>Boost C++ Libraries <tt><a href=\"https://www.boost.org\">%1</a></tt></p>"
		"<p>VTK - The Visualization Toolkit <tt><a href=\"https://vtk.org/\">%2</a></tt></p>"
		"<p>The Point Cloud Library (PCL) <tt><a href=\"https://pointclouds.org/\">%3</a></tt></p>"
		"<p>Eigen - C++ template library for linear algebra <tt><a href=\"https://eigen.tuxfamily.org/\">%4.%5.%6</a></tt></p>"
	).arg(BOOST_LIB_VERSION).arg(GetVTKVersion()).arg(PCL_VERSION_PRETTY).arg(EIGEN_WORLD_VERSION).arg(EIGEN_MAJOR_VERSION).arg(EIGEN_MINOR_VERSION));
	auto layoutOpenSource = new QHBoxLayout(openSource);
	layoutOpenSource->addWidget(textOpenSource);

	qtwTab->addTab(about, tr("&About LIDAR's algorithms test laboratory"));
	qtwTab->addTab(qtLicense, tr("&License"));
	qtwTab->addTab(openSource, tr("&Open source libraries and algorithms"));
	vblMain->addWidget(qtwTab);
}

LATAboutDialog::~LATAboutDialog()
{

}
