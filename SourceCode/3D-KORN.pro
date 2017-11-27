#-------------------------------------------------
#
# Project created by QtCreator 2016-10-29T00:10:26
#
#-------------------------------------------------

QT       += core gui opengl serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 3D-KORN
TEMPLATE = app

# PCL
INCLUDEPATH += "C:\Program Files\PCL 1.8.0\include\pcl-1.8"
INCLUDEPATH += "F:\VTK\include\vtk-7.0"
INCLUDEPATH += "C:\Program Files\PCL 1.8.0\3rdParty\Boost\include\boost-1_61"
INCLUDEPATH += "C:\Program Files\PCL 1.8.0\3rdParty\Qhull\include"
INCLUDEPATH += "C:\Program Files\PCL 1.8.0\3rdParty\FLANN\include"
INCLUDEPATH += "C:\Program Files\PCL 1.8.0\3rdParty\Eigen\eigen3"
INCLUDEPATH += "C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\inc"
INCLUDEPATH += "C:\Program Files\OpenNI2\Include"
INCLUDEPATH += "C:\Program Files (x86)\Intel\RSSDK\include"
INCLUDEPATH += "C:\Program Files (x86)\Intel\RSSDK\src\libpxc"



LIBS += opengl32.lib advapi32.lib Ws2_32.lib user32.lib shell32.lib gdi32.lib kernel32.lib
LIBS += "-LC:\Program Files\PCL 1.8.0\lib"
LIBS += "-LF:\VTK\lib"
LIBS += "-LC:\Program Files\PCL 1.8.0\3rdParty\Qhull\lib"
LIBS += "-LC:\Program Files\PCL 1.8.0\3rdParty\FLANN\lib"
LIBS += "-LC:\Program Files\PCL 1.8.0\3rdParty\Boost\lib"
LIBS += "-LC:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\Lib\x64"
LIBS += "-LC:\Program Files\OpenNI2\Lib"
LIBS += "C:/Program Files (x86)/Intel/RSSDK/lib/x64/*.lib"
LIBS += "C:/Program Files (x86)/Intel/RSSDK/sample/common/lib/x64/v140/*.lib"



LIBS += -lpcl_common_release
LIBS += -lpcl_features_release
LIBS += -lpcl_filters_release
LIBS += -lpcl_io_ply_release
LIBS += -lpcl_io_release
LIBS += -lpcl_kdtree_release
LIBS += -lpcl_keypoints_release
LIBS += -lpcl_ml_release
LIBS += -lpcl_octree_release
LIBS += -lpcl_outofcore_release
LIBS += -lpcl_people_release
LIBS += -lpcl_recognition_release
LIBS += -lpcl_registration_release
LIBS += -lpcl_sample_consensus_release
LIBS += -lpcl_search_release
LIBS += -lpcl_segmentation_release
LIBS += -lpcl_stereo_release
LIBS += -lpcl_surface_release
LIBS += -lpcl_tracking_release
LIBS += -lpcl_visualization_release
LIBS += -llibboost_atomic-vc140-mt-1_61
LIBS += -llibboost_chrono-vc140-mt-1_61
LIBS += -llibboost_container-vc140-mt-1_61
LIBS += -llibboost_context-vc140-mt-1_61
LIBS += -llibboost_coroutine-vc140-mt-1_61
LIBS += -llibboost_date_time-vc140-mt-1_61
LIBS += -llibboost_exception-vc140-mt-1_61
LIBS += -llibboost_filesystem-vc140-mt-1_61
LIBS += -llibboost_graph-vc140-mt-1_61
LIBS += -llibboost_iostreams-vc140-mt-1_61
LIBS += -llibboost_locale-vc140-mt-1_61
LIBS += -llibboost_log-vc140-mt-1_61
LIBS += -llibboost_log_setup-vc140-mt-1_61
LIBS += -llibboost_math_c99-vc140-mt-1_61
LIBS += -llibboost_math_c99f-vc140-mt-1_61
LIBS += -llibboost_math_c99l-vc140-mt-1_61
LIBS += -llibboost_math_tr1-vc140-mt-1_61
LIBS += -llibboost_math_tr1f-vc140-mt-1_61
LIBS += -llibboost_math_tr1l-vc140-mt-1_61
LIBS += -llibboost_mpi-vc140-mt-1_61
LIBS += -llibboost_prg_exec_monitor-vc140-mt-1_61
LIBS += -llibboost_program_options-vc140-mt-1_61
LIBS += -llibboost_random-vc140-mt-1_61
LIBS += -llibboost_regex-vc140-mt-1_61
LIBS += -llibboost_serialization-vc140-mt-1_61
LIBS += -llibboost_signals-vc140-mt-1_61
LIBS += -llibboost_system-vc140-mt-1_61
LIBS += -llibboost_test_exec_monitor-vc140-mt-1_61
LIBS += -llibboost_thread-vc140-mt-1_61
LIBS += -llibboost_timer-vc140-mt-1_61
LIBS += -llibboost_type_erasure-vc140-mt-1_61
LIBS += -llibboost_unit_test_framework-vc140-mt-1_61
LIBS += -llibboost_wave-vc140-mt-1_61
LIBS += -llibboost_wserialization-vc140-mt-1_61
LIBS += -lflann_cpp_s
LIBS += -lqhullstatic
LIBS += -lvtkalglib-7.0
LIBS += -lvtkChartsCore-7.0
LIBS += -lvtkCommonColor-7.0
LIBS += -lvtkCommonComputationalGeometry-7.0
LIBS += -lvtkCommonCore-7.0
LIBS += -lvtkCommonDataModel-7.0
LIBS += -lvtkCommonExecutionModel-7.0
LIBS += -lvtkCommonMath-7.0
LIBS += -lvtkCommonMisc-7.0
LIBS += -lvtkCommonSystem-7.0
LIBS += -lvtkCommonTransforms-7.0
LIBS += -lvtkDICOMParser-7.0
LIBS += -lvtkDomainsChemistry-7.0
LIBS += -lvtkDomainsChemistryOpenGL2-7.0
LIBS += -lvtkexoIIc-7.0
LIBS += -lvtkexpat-7.0
LIBS += -lvtkFiltersAMR-7.0
LIBS += -lvtkFiltersCore-7.0
LIBS += -lvtkFiltersExtraction-7.0
LIBS += -lvtkFiltersFlowPaths-7.0
LIBS += -lvtkFiltersGeneral-7.0
LIBS += -lvtkFiltersGeneric-7.0
LIBS += -lvtkFiltersGeometry-7.0
LIBS += -lvtkFiltersHybrid-7.0
LIBS += -lvtkFiltersHyperTree-7.0
LIBS += -lvtkFiltersImaging-7.0
LIBS += -lvtkFiltersModeling-7.0
LIBS += -lvtkFiltersParallel-7.0
LIBS += -lvtkFiltersParallelImaging-7.0
LIBS += -lvtkFiltersProgrammable-7.0
LIBS += -lvtkFiltersSelection-7.0
LIBS += -lvtkFiltersSMP-7.0
LIBS += -lvtkFiltersSources-7.0
LIBS += -lvtkFiltersStatistics-7.0
LIBS += -lvtkFiltersTexture-7.0
LIBS += -lvtkFiltersVerdict-7.0
LIBS += -lvtkfreetype-7.0
LIBS += -lvtkGUISupportQtSQL-7.0
LIBS += -lvtkGUISupportQt-7.0
LIBS += -lvtkGeovisCore-7.0
LIBS += -lvtkglew-7.0
#LIBS += -lvtkgl2ps-7.0
LIBS += -lvtkhdf5-7.0
LIBS += -lvtkhdf5_hl-7.0
LIBS += -lvtkImagingColor-7.0
LIBS += -lvtkImagingCore-7.0
LIBS += -lvtkImagingFourier-7.0
LIBS += -lvtkImagingGeneral-7.0
LIBS += -lvtkImagingHybrid-7.0
LIBS += -lvtkImagingMath-7.0
LIBS += -lvtkImagingMorphological-7.0
LIBS += -lvtkImagingSources-7.0
LIBS += -lvtkImagingStatistics-7.0
LIBS += -lvtkImagingStencil-7.0
LIBS += -lvtkInfovisCore-7.0
LIBS += -lvtkInfovisLayout-7.0
LIBS += -lvtkInteractionImage-7.0
LIBS += -lvtkInteractionStyle-7.0
LIBS += -lvtkInteractionWidgets-7.0
LIBS += -lvtkIOAMR-7.0
LIBS += -lvtkIOCore-7.0
LIBS += -lvtkIOEnSight-7.0
LIBS += -lvtkIOExodus-7.0
LIBS += -lvtkIOExport-7.0
LIBS += -lvtkIOGeometry-7.0
LIBS += -lvtkIOImage-7.0
LIBS += -lvtkIOImport-7.0
LIBS += -lvtkIOInfovis-7.0
LIBS += -lvtkIOLegacy-7.0
LIBS += -lvtkIOLSDyna-7.0
LIBS += -lvtkIOMINC-7.0
LIBS += -lvtkIOMovie-7.0
LIBS += -lvtkIONetCDF-7.0
LIBS += -lvtkIOParallel-7.0
LIBS += -lvtkIOParallelXML-7.0
LIBS += -lvtkIOPLY-7.0
LIBS += -lvtkIOSQL-7.0
LIBS += -lvtkIOVideo-7.0
LIBS += -lvtkIOXML-7.0
LIBS += -lvtkIOXMLParser-7.0
LIBS += -lvtkjpeg-7.0
LIBS += -lvtkjsoncpp-7.0
LIBS += -lvtklibxml2-7.0
LIBS += -lvtkmetaio-7.0
LIBS += -lvtkNetCDF-7.0
LIBS += -lvtkNetCDF_cxx-7.0
LIBS += -lvtkoggtheora-7.0
LIBS += -lvtkParallelCore-7.0
LIBS += -lvtkpng-7.0
LIBS += -lvtkproj4-7.0
LIBS += -lvtkRenderingAnnotation-7.0
LIBS += -lvtkRenderingContext2D-7.0
LIBS += -lvtkRenderingContextOpenGL2-7.0
LIBS += -lvtkRenderingCore-7.0
LIBS += -lvtkRenderingFreeType-7.0
#LIBS += -lvtkRenderingGL2PS-7.0
LIBS += -lvtkRenderingImage-7.0
LIBS += -lvtkRenderingLabel-7.0
#LIBS += -lvtkRenderingLIC-7.0
LIBS += -lvtkRenderingLOD-7.0
LIBS += -lvtkRenderingOpenGL2-7.0
LIBS += -lvtkRenderingQt-7.0
LIBS += -lvtkRenderingVolume-7.0
LIBS += -lvtkRenderingVolumeOpenGL2-7.0
LIBS += -lvtksqlite-7.0
LIBS += -lvtksys-7.0
LIBS += -lvtktiff-7.0
LIBS += -lvtkverdict-7.0
LIBS += -lvtkViewsContext2D-7.0
LIBS += -lvtkViewsCore-7.0
LIBS += -lvtkViewsGeovis-7.0
LIBS += -lvtkViewsInfovis-7.0
LIBS += -lvtkViewsQt-7.0
LIBS += -lvtkzlib-7.0
LIBS += -lOpenNI2
LIBS += -lkinect20


SOURCES += main.cpp\
        mainwindow.cpp \
    tdk_centralwidget.cpp \
    tdk_scanwindow.cpp \
    tdk_database.cpp \
    tdk_edit.cpp \
    kinect2_grabber.cpp \
    tdk_scanregistration.cpp \
    tdk_sensor.cpp \
    tdk_intelr200sensor.cpp \
    tdk_sensorcontroller.cpp \
    tdk_kinectv2sensor.cpp \
    tdk_turntable.cpp \
    TDK_PointOperations.cpp

HEADERS  += mainwindow.h \
    tdk_centralwidget.h \
    tdk_scanwindow.h \
    tdk_database.h \
    tdk_edit.h \
    kinect2_grabber.h \
    tdk_scanregistration.h \
    tdk_sensor.h \
    tdk_intelr200sensor.h \
    tdk_sensorcontroller.h \
    tdk_kinectv2sensor.h \
    tdk_turntable.h \
    TDK_PointOperations.h

FORMS    += mainwindow.ui
