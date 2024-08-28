TARGET     	= LATLaboratory

DEFINES   	+= __TBB_NO_IMPLICIT_LINKAGE TBB_USE_ASSERT=0 NOMINMAX WIN32_LEAN_AND_MEAN _ENABLE_EXTENDED_ALIGNED_STORAGE

CONFIG 		+= c++17

TEMPLATE  	= app

INCLUDEPATH = sources/thirdparty/tbb/include\
			sources/thirdparty/\
			sources/thirdparty/VTK-9.3.1/include\
			sources/thirdparty/boost-1.84.0/include\
			sources/thirdparty/pcl-1.14.1/include

HEADERS    	= sources/Version.h\
			sources/LATMainWindow.h\
			sources/LATGlobalContext.h\
			sources/LATAboutDialog.h\
			sources/LATConsoleDockWindow.h\
			sources/LATPointCloudViewer.h\
			sources/LATCloudDataSource.h\
			sources/LATControlThread.h\
			sources/thirdparty/tbb/include/tick_count.h

		
SOURCES    	= sources/main.cpp\
			sources/LATMainWindow.cpp\
			sources/LATGlobalContext.cpp\
			sources/LATAboutDialog.cpp\
			sources/LATPointCloudViewer.cpp\
			sources/LATCloudDataSource.cpp\
			sources/LATControlThread.cpp\
			sources/LATConsoleDockWindow.cpp
 	     
QT          += core gui widgets xml concurrent

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
   
   CONFIG(release, debug|release) {
  	LIBS += -L$$PWD/sources/thirdparty/vtk-9.3.1/libs_release -lDbghelp -lpsapi\
	-lvtkcgns-9.3 -lvtkChartsCore-9.3 -lvtkCommonColor-9.3 -lvtkCommonComputationalGeometry-9.3 -lvtkCommonCore-9.3 -lvtkCommonDataModel-9.3 -lvtkCommonExecutionModel-9.3\
	-lvtkCommonMath-9.3 -lvtkCommonMisc-9.3 -lvtkCommonSystem-9.3 -lvtkCommonTransforms-9.3 -lvtkDICOMParser-9.3 -lvtkDomainsChemistry-9.3 -lvtkDomainsChemistryOpenGL2-9.3\
	-lvtkdoubleconversion-9.3 -lvtkexodusII-9.3 -lvtkexpat-9.3 -lvtkFiltersAMR-9.3 -lvtkFiltersCellGrid-9.3 -lvtkFiltersCore-9.3 -lvtkFiltersExtraction-9.3\
	-lvtkFiltersFlowPaths-9.3 -lvtkFiltersGeneral-9.3 -lvtkFiltersGeneric-9.3 -lvtkFiltersGeometry-9.3 -lvtkFiltersGeometryPreview-9.3 -lvtkFiltersHybrid-9.3\
	-lvtkFiltersHyperTree-9.3 -lvtkFiltersImaging-9.3 -lvtkFiltersModeling-9.3 -lvtkFiltersParallel-9.3 -lvtkFiltersParallelImaging-9.3 -lvtkFiltersPoints-9.3\
	-lvtkFiltersProgrammable-9.3 -lvtkFiltersReduction-9.3 -lvtkFiltersSelection-9.3 -lvtkFiltersSMP-9.3 -lvtkFiltersSources-9.3 -lvtkFiltersStatistics-9.3\
	-lvtkFiltersTemporal-9.3 -lvtkFiltersTensor-9.3 -lvtkFiltersTexture-9.3 -lvtkFiltersTopology-9.3 -lvtkFiltersVerdict-9.3 -lvtkfmt-9.3\
	-lvtkGeovisCore-9.3 -lvtkgl2ps-9.3 -lvtkglew-9.3 -lvtkGUISupportQt-9.3 -lvtkGUISupportQtQuick-9.3 -lvtkGUISupportQtSQL-9.3 -lvtkhdf5-9.3 -lvtkhdf5_hl-9.3\
	-lvtkImagingColor-9.3 -lvtkImagingCore-9.3 -lvtkImagingFourier-9.3 -lvtkImagingGeneral-9.3 -lvtkImagingHybrid-9.3 -lvtkImagingMath-9.3 -lvtkImagingMorphological-9.3\
	-lvtkImagingSources-9.3 -lvtkImagingStatistics-9.3 -lvtkImagingStencil-9.3 -lvtkInfovisLayout-9.3 -lvtkInteractionImage-9.3 -lvtkInteractionStyle-9.3\
	-lvtkInteractionWidgets-9.3 -lvtkIOAMR-9.3 -lvtkIOAsynchronous-9.3 -lvtkIOCellGrid-9.3 -lvtkIOCesium3DTiles-9.3 -lvtkIOCGNSReader-9.3 -lvtkIOChemistry-9.3\
	-lvtkIOCityGML-9.3 -lvtkIOCONVERGECFD-9.3 -lvtkIOCore-9.3 -lvtkIOEngys-9.3 -lvtkIOEnSight-9.3 -lvtkIOERF-9.3 -lvtkIOExodus-9.3 -lvtkIOExport-9.3\
	-lvtkIOExportGL2PS-9.3 -lvtkIOExportPDF-9.3 -lvtkIOFDS-9.3 -lvtkIOFLUENTCFF-9.3 -lvtkIOGeometry-9.3 -lvtkIOHDF-9.3 -lvtkIOImage-9.3 -lvtkIOImport-9.3\
	-lvtkIOInfovis-9.3 -lvtkIOIOSS-9.3 -lvtkIOLegacy-9.3 -lvtkIOLSDyna-9.3 -lvtkIOMINC-9.3 -lvtkIOMotionFX-9.3 -lvtkIOMovie-9.3 -lvtkIONetCDF-9.3\
	-lvtkIOOggTheora-9.3 -lvtkIOParallel-9.3 -lvtkIOParallelXML-9.3 -lvtkIOPLY-9.3 -lvtkIOSegY-9.3 -lvtkIOSQL-9.3 -lvtkioss-9.3 -lvtkIOTecplotTable-9.3\
	-lvtkIOVeraOut-9.3 -lvtkIOVideo-9.3 -lvtkIOXML-9.3 -lvtkIOXMLParser-9.3 -lvtkjpeg-9.3 -lvtkjsoncpp-9.3 -lvtkkissfft-9.3 -lvtklibharu-9.3 -lvtklibproj-9.3\
	-lvtklibxml2-9.3 -lvtkloguru-9.3 -lvtklz4-9.3 -lvtklzma-9.3 -lvtkmetaio-9.3 -lvtknetcdf-9.3 -lvtkogg-9.3 -lvtkParallelCore-9.3 -lvtkParallelDIY-9.3\
	-lvtkpng-9.3 -lvtkpugixml-9.3 -lvtkRenderingAnnotation-9.3 -lvtkRenderingCellGrid-9.3 -lvtkRenderingContext2D-9.3 -lvtkRenderingContextOpenGL2-9.3\
	-lvtkRenderingCore-9.3 -lvtkRenderingGL2PSOpenGL2-9.3 -lvtkRenderingHyperTreeGrid-9.3 -lvtkRenderingImage-9.3\
	-lvtkRenderingLabel-9.3 -lvtkRenderingLICOpenGL2-9.3 -lvtkRenderingLOD-9.3 -lvtkRenderingOpenGL2-9.3 -lvtkRenderingQt-9.3 -lvtkRenderingSceneGraph-9.3\
	-lvtkRenderingUI-9.3 -lvtkRenderingVolume-9.3 -lvtkRenderingVolumeOpenGL2-9.3 -lvtkRenderingVtkJS-9.3 -lvtksqlite-9.3 -lvtksys-9.3 -lvtkTestingCore-9.3\
	-lvtkTestingRendering-9.3 -lvtktheora-9.3 -lvtktiff-9.3 -lvtktoken-9.3 -lvtkverdict-9.3 -lvtkViewsContext2D-9.3 -lvtkViewsCore-9.3 -lvtkViewsInfovis-9.3\
	-lvtkViewsQt-9.3 -lvtkWrappingTools-9.3 -lvtkzlib-9.3 -lvtkfreetype-9.3  -lvtkRenderingFreeType-9.3
	LIBS += -L$$PWD/sources/thirdparty/pcl-1.14.1/libs_release\
	-lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_io_ply -lpcl_kdtree -lpcl_keypoints -lpcl_ml -lpcl_octree -lpcl_outofcore -lpcl_people\
	-lpcl_recognition -lpcl_registration -lpcl_sample_consensus -lpcl_search -lpcl_segmentation -lpcl_stereo -lpcl_surface -lpcl_tracking\
	-lpcl_visualization
	LIBS += -L$$PWD/sources/thirdparty/boost-1.84.0/libs_release\
	-llibboost_wserialization-vc143-mt-s-x64-1_84 -llibboost_wave-vc143-mt-s-x64-1_84 -llibboost_url-vc143-mt-s-x64-1_84 -llibboost_unit_test_framework-vc143-mt-s-x64-1_84\
	-llibboost_type_erasure-vc143-mt-s-x64-1_84 -llibboost_timer-vc143-mt-s-x64-1_84 -llibboost_thread-vc143-mt-s-x64-1_84 -llibboost_test_exec_monitor-vc143-mt-s-x64-1_84\
	-llibboost_system-vc143-mt-s-x64-1_84 -llibboost_stacktrace_windbg_cached-vc143-mt-s-x64-1_84 -llibboost_stacktrace_windbg-vc143-mt-s-x64-1_84\
	-llibboost_stacktrace_noop-vc143-mt-s-x64-1_84 -llibboost_serialization-vc143-mt-s-x64-1_84 -llibboost_regex-vc143-mt-s-x64-1_84 -llibboost_random-vc143-mt-s-x64-1_84\
	-llibboost_python27-vc143-mt-s-x64-1_84 -llibboost_program_options-vc143-mt-s-x64-1_84 -llibboost_prg_exec_monitor-vc143-mt-s-x64-1_84 -llibboost_numpy27-vc143-mt-s-x64-1_84\
	-llibboost_nowide-vc143-mt-s-x64-1_84 -llibboost_math_tr1l-vc143-mt-s-x64-1_84 -llibboost_math_tr1f-vc143-mt-s-x64-1_84 -llibboost_math_tr1-vc143-mt-s-x64-1_84\
	-llibboost_math_c99l-vc143-mt-s-x64-1_84 -llibboost_math_c99f-vc143-mt-s-x64-1_84 -llibboost_math_c99-vc143-mt-s-x64-1_84 -llibboost_log_setup-vc143-mt-s-x64-1_84\
	-llibboost_log-vc143-mt-s-x64-1_84 -llibboost_locale-vc143-mt-s-x64-1_84 -llibboost_json-vc143-mt-s-x64-1_84 -llibboost_iostreams-vc143-mt-s-x64-1_84\
	-llibboost_graph-vc143-mt-s-x64-1_84 -llibboost_filesystem-vc143-mt-s-x64-1_84 -llibboost_fiber-vc143-mt-s-x64-1_84 -llibboost_exception-vc143-mt-s-x64-1_84\
	-llibboost_date_time-vc143-mt-s-x64-1_84 -llibboost_coroutine-vc143-mt-s-x64-1_84 -llibboost_contract-vc143-mt-s-x64-1_84 -llibboost_context-vc143-mt-s-x64-1_84\
	-llibboost_container-vc143-mt-s-x64-1_84 -llibboost_chrono-vc143-mt-s-x64-1_84 -llibboost_atomic-vc143-mt-s-x64-1_84
   }
   CONFIG(debug, debug|release) {
	LIBS += -L$$PWD/sources/thirdparty/vtk-9.3.1/libs_debug -lDbghelp -lpsapi\
	-lvtkcgns-9.3d -lvtkChartsCore-9.3d -lvtkCommonColor-9.3d -lvtkCommonComputationalGeometry-9.3d -lvtkCommonCore-9.3d -lvtkCommonDataModel-9.3d -lvtkCommonExecutionModel-9.3d\
	-lvtkCommonMath-9.3d -lvtkCommonMisc-9.3d -lvtkCommonSystem-9.3d -lvtkCommonTransforms-9.3d -lvtkDICOMParser-9.3d -lvtkDomainsChemistry-9.3d -lvtkDomainsChemistryOpenGL2-9.3d\
	-lvtkdoubleconversion-9.3d -lvtkexodusII-9.3d -lvtkexpat-9.3d -lvtkFiltersAMR-9.3d -lvtkFiltersCellGrid-9.3d -lvtkFiltersCore-9.3d -lvtkFiltersExtraction-9.3d\
	-lvtkFiltersFlowPaths-9.3d -lvtkFiltersGeneral-9.3d -lvtkFiltersGeneric-9.3d -lvtkFiltersGeometry-9.3d -lvtkFiltersGeometryPreview-9.3d -lvtkFiltersHybrid-9.3d\
	-lvtkFiltersHyperTree-9.3d -lvtkFiltersImaging-9.3d -lvtkFiltersModeling-9.3d -lvtkFiltersParallel-9.3d -lvtkFiltersParallelImaging-9.3d -lvtkFiltersPoints-9.3d\
	-lvtkFiltersProgrammable-9.3d -lvtkFiltersReduction-9.3d -lvtkFiltersSelection-9.3d -lvtkFiltersSMP-9.3d -lvtkFiltersSources-9.3d -lvtkFiltersStatistics-9.3d\
	-lvtkFiltersTemporal-9.3d -lvtkFiltersTensor-9.3d -lvtkFiltersTexture-9.3d -lvtkFiltersTopology-9.3d -lvtkFiltersVerdict-9.3d -lvtkfmt-9.3d\
	-lvtkGeovisCore-9.3d -lvtkgl2ps-9.3d -lvtkglew-9.3d -lvtkGUISupportQt-9.3d -lvtkGUISupportQtQuick-9.3d -lvtkGUISupportQtSQL-9.3d -lvtkhdf5-9.3d -lvtkhdf5_hl-9.3d\
	-lvtkImagingColor-9.3d -lvtkImagingCore-9.3d -lvtkImagingFourier-9.3d -lvtkImagingGeneral-9.3d -lvtkImagingHybrid-9.3d -lvtkImagingMath-9.3d -lvtkImagingMorphological-9.3d\
	-lvtkImagingSources-9.3d -lvtkImagingStatistics-9.3d -lvtkImagingStencil-9.3d -lvtkInfovisLayout-9.3d -lvtkInteractionImage-9.3d -lvtkInteractionStyle-9.3d\
	-lvtkInteractionWidgets-9.3d -lvtkIOAMR-9.3d -lvtkIOAsynchronous-9.3d -lvtkIOCellGrid-9.3d -lvtkIOCesium3DTiles-9.3d -lvtkIOCGNSReader-9.3d -lvtkIOChemistry-9.3d\
	-lvtkIOCityGML-9.3d -lvtkIOCONVERGECFD-9.3d -lvtkIOCore-9.3d -lvtkIOEngys-9.3d -lvtkIOEnSight-9.3d -lvtkIOERF-9.3d -lvtkIOExodus-9.3d -lvtkIOExport-9.3d\
	-lvtkIOExportGL2PS-9.3d -lvtkIOExportPDF-9.3d -lvtkIOFDS-9.3d -lvtkIOFLUENTCFF-9.3d -lvtkIOGeometry-9.3d -lvtkIOHDF-9.3d -lvtkIOImage-9.3d -lvtkIOImport-9.3d\
	-lvtkIOInfovis-9.3d -lvtkIOIOSS-9.3d -lvtkIOLegacy-9.3d -lvtkIOLSDyna-9.3d -lvtkIOMINC-9.3d -lvtkIOMotionFX-9.3d -lvtkIOMovie-9.3d -lvtkIONetCDF-9.3d\
	-lvtkIOOggTheora-9.3d -lvtkIOParallel-9.3d -lvtkIOParallelXML-9.3d -lvtkIOPLY-9.3d -lvtkIOSegY-9.3d -lvtkIOSQL-9.3d -lvtkioss-9.3d -lvtkIOTecplotTable-9.3d\
	-lvtkIOVeraOut-9.3d -lvtkIOVideo-9.3d -lvtkIOXML-9.3d -lvtkIOXMLParser-9.3d -lvtkjpeg-9.3d -lvtkjsoncpp-9.3d -lvtkkissfft-9.3d -lvtklibharu-9.3d -lvtklibproj-9.3d\
	-lvtklibxml2-9.3d -lvtkloguru-9.3d -lvtklz4-9.3d -lvtklzma-9.3d -lvtkmetaio-9.3d -lvtknetcdf-9.3d -lvtkogg-9.3d -lvtkParallelCore-9.3d -lvtkParallelDIY-9.3d\
	-lvtkpng-9.3d -lvtkpugixml-9.3d -lvtkRenderingAnnotation-9.3d -lvtkRenderingCellGrid-9.3d -lvtkRenderingContext2D-9.3d -lvtkRenderingContextOpenGL2-9.3d\
	-lvtkRenderingCore-9.3d -lvtkRenderingGL2PSOpenGL2-9.3d -lvtkRenderingHyperTreeGrid-9.3d -lvtkRenderingImage-9.3d\
	-lvtkRenderingLabel-9.3d -lvtkRenderingLICOpenGL2-9.3d -lvtkRenderingLOD-9.3d -lvtkRenderingOpenGL2-9.3d -lvtkRenderingQt-9.3d -lvtkRenderingSceneGraph-9.3d\
	-lvtkRenderingUI-9.3d -lvtkRenderingVolume-9.3d -lvtkRenderingVolumeOpenGL2-9.3d -lvtkRenderingVtkJS-9.3d -lvtksqlite-9.3d -lvtksys-9.3d -lvtkTestingCore-9.3d\
	-lvtkTestingRendering-9.3d -lvtktheora-9.3d -lvtktiff-9.3d -lvtktoken-9.3d -lvtkverdict-9.3d -lvtkViewsContext2D-9.3d -lvtkViewsCore-9.3d -lvtkViewsInfovis-9.3d\
	-lvtkViewsQt-9.3d -lvtkWrappingTools-9.3d -lvtkzlib-9.3d -lvtkfreetype-9.3d  -lvtkRenderingFreeType-9.3d
	LIBS += -L$$PWD/sources/thirdparty/pcl-1.14.1/libs_debug\
	-lpcl_commond -lpcl_featuresd -lpcl_filtersd -lpcl_iod -lpcl_io_plyd -lpcl_kdtreed -lpcl_keypointsd -lpcl_mld -lpcl_octreed -lpcl_outofcored -lpcl_peopled\
	-lpcl_recognitiond -lpcl_registrationd -lpcl_sample_consensusd -lpcl_searchd -lpcl_segmentationd -lpcl_stereod -lpcl_surfaced -lpcl_trackingd\
	-lpcl_visualizationd
	LIBS += -L$$PWD/sources/thirdparty/boost-1.84.0/libs_debug\
	-llibboost_wserialization-vc143-mt-sgd-x64-1_84 -llibboost_wave-vc143-mt-sgd-x64-1_84 -llibboost_url-vc143-mt-sgd-x64-1_84 -llibboost_unit_test_framework-vc143-mt-sgd-x64-1_84\
	-llibboost_type_erasure-vc143-mt-sgd-x64-1_84 -llibboost_timer-vc143-mt-sgd-x64-1_84 -llibboost_thread-vc143-mt-sgd-x64-1_84 -llibboost_test_exec_monitor-vc143-mt-sgd-x64-1_84\
	-llibboost_system-vc143-mt-sgd-x64-1_84 -llibboost_stacktrace_windbg_cached-vc143-mt-sgd-x64-1_84 -llibboost_stacktrace_windbg-vc143-mt-sgd-x64-1_84\
	-llibboost_stacktrace_noop-vc143-mt-sgd-x64-1_84 -llibboost_serialization-vc143-mt-sgd-x64-1_84 -llibboost_regex-vc143-mt-sgd-x64-1_84 -llibboost_random-vc143-mt-sgd-x64-1_84\
	-llibboost_python27-vc143-mt-sgd-x64-1_84 -llibboost_program_options-vc143-mt-sgd-x64-1_84 -llibboost_prg_exec_monitor-vc143-mt-sgd-x64-1_84 -llibboost_numpy27-vc143-mt-sgd-x64-1_84\
	-llibboost_nowide-vc143-mt-sgd-x64-1_84 -llibboost_math_tr1l-vc143-mt-sgd-x64-1_84 -llibboost_math_tr1f-vc143-mt-sgd-x64-1_84 -llibboost_math_tr1-vc143-mt-sgd-x64-1_84\
	-llibboost_math_c99l-vc143-mt-sgd-x64-1_84 -llibboost_math_c99f-vc143-mt-sgd-x64-1_84 -llibboost_math_c99-vc143-mt-sgd-x64-1_84 -llibboost_log_setup-vc143-mt-sgd-x64-1_84\
	-llibboost_log-vc143-mt-sgd-x64-1_84 -llibboost_locale-vc143-mt-sgd-x64-1_84 -llibboost_json-vc143-mt-sgd-x64-1_84 -llibboost_iostreams-vc143-mt-sgd-x64-1_84\
	-llibboost_graph-vc143-mt-sgd-x64-1_84 -llibboost_filesystem-vc143-mt-sgd-x64-1_84 -llibboost_fiber-vc143-mt-sgd-x64-1_84 -llibboost_exception-vc143-mt-sgd-x64-1_84\
	-llibboost_date_time-vc143-mt-sgd-x64-1_84 -llibboost_coroutine-vc143-mt-sgd-x64-1_84 -llibboost_contract-vc143-mt-sgd-x64-1_84 -llibboost_context-vc143-mt-sgd-x64-1_84\
	-llibboost_container-vc143-mt-sgd-x64-1_84 -llibboost_chrono-vc143-mt-sgd-x64-1_84 -llibboost_atomic-vc143-mt-sgd-x64-1_84
   }
}




