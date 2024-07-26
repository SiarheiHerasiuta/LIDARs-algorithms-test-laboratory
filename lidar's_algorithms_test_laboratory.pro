TARGET     = LATLaboratory

DEFINES   += __TBB_NO_IMPLICIT_LINKAGE TBB_USE_ASSERT=0 NOMINMAX WIN32_LEAN_AND_MEAN _ENABLE_EXTENDED_ALIGNED_STORAGE

CONFIG += c++17

TEMPLATE  = app

INCLUDEPATH = sources/thirdparty/tbb/include

HEADERS    = sources/Version.h\
		   sources/LATMainWindow.h\
		   sources/LATGlobalContext.h\
		   sources/LATAboutDialog.h\
		   sources/LATConsoleDockWindow.h\
		   sources/thirdparty/tbb/include/tick_count.h

		
SOURCES    = sources/main.cpp\
		   sources/LATMainWindow.cpp\
		   sources/LATGlobalContext.cpp\
		   sources/LATAboutDialog.cpp\
		   sources/LATConsoleDockWindow.cpp
 	     
FORMS	= 

QT          += core gui widgets xml

MOC_DIR = $$PWD/.moc
RCC_DIR = $$PWD/.rcc
UI_DIR = $$PWD/.ui
OBJECTS_DIR = $$PWD/.obj

RESOURCES = "lidar's_algorithms_test_laboratory.qrc"

win32 {
   QMAKE_CXXFLAGS_DEBUG 	+= -MP
   QMAKE_CXXFLAGS_RELEASE  	+= -Oi -MP -Ox -GL
   QMAKE_LFLAGS_RELEASE    = /INCREMENTAL:NO /LTCG

   RC_FILE = "lidar's_algorithms_test_laboratory.rc"
}




