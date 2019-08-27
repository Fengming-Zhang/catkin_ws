
HEADERS += \
    WEMapEditor.h \
    WEImageWidget.h \
    gs_mapb.h \
    gs_common.h \
    topo_map.h \
    WEMapDialog.h \
    gs_util.h

SOURCES += \
    WEMapEditor.cpp \
    WEImageWidget.cpp \
    main.cpp \
    gs_mapb.cpp \
    topo_map.cpp \
    WEMapDialog.cpp \
    gs_util.cpp

RESOURCES += \
    WEMapEditor.qrc

FORMS += \
    WEMapDialog.ui

OTHER_FILES += \
    ../we_map_editor-build-desktop/WEMapEditor.ini
