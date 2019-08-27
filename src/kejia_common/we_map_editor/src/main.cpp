#include <QApplication>

#include "WEMapEditor.h"
#include"WEMapDialog.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MapEditor mainWin;
    mainWin.show();
    return app.exec();
}
