#ifndef WEMAPEDITOR_H
#define WEMAPEDITOR_H
#include <QMainWindow>
#include "WEImageWidget.h"
#include "gs_mapb.h"
#include "topo_map.h"
#include "WEMapDialog.h"

class MapEditor : public QMainWindow
{
    Q_OBJECT

public:
    MapEditor();
    ~MapEditor();

signals:
    void setWorkType(WorkType &type);
    void addRect(QRect &rect,QString &type,QString &name);
    void addLine(QLine &line,QString &type,QString &name);
    void loadNewTopoFile(std::vector <Entity> m_entities);

private slots:
    void open();
    void openTopoFile();
    bool save();
    bool saveBinaryMap();
    bool saveAs();
    void showTex(QString tex);
    void saveAnEntity(QString &type, QString &name, QRect &rect);
    void saveAnEntity(QString &type, QString &name, QLine &line);
    void workTypeChanged(QAction* val);
    void eraseMap(QRect &rect,WorkType &type);
    void penMap(QLine &l,WorkType &type,QVector<RoomRect>&rooms);
    void resetMap(QRect &rect,WorkType &type);

protected:
    void closeEvent(QCloseEvent *event);

private:
    void createActions();
    void createMenus();
    void createToolBars();
    void createStatusBar();
    bool okToContinue();

    bool readSettings();
    bool writeSettings();

    QMap<QString ,QVector<QString> > names;

    QToolBar * fileToolBar;
    QToolBar * workTypeToolBar;
    QMenu * fileMenu;
    QAction * openAction;
    QAction * openTopoAction;
    QAction * saveAction;
    QAction * saveAsAction;
    QAction *saveBAction;
    QAction * exitAction;
    QAction * roomAction;
    QAction * doorAction;
    QAction * passwayAction;
    QAction * getThePictureAction;
    QAction * furnitureAction;
    QAction * addLineAction;
    QAction * eraserAction;
    QAction * resetAction;
    QActionGroup * workTypeAction;
    QLabel *locationLabel;
    WEImageWidget * imgWidget;
    WEMapDialog * mapDialog;
    QString curFile;
    bool changed;
    QString BmapFileName;

private:
    TopoMap topomap;
    gslam::GridMapBase map;
};

#endif // WEMAPEDITOR_H
