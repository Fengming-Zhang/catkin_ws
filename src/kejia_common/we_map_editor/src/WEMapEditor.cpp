#include <QtGui>
#include <QSettings>
#include "WEMapEditor.h"
#include <QFile>

MapEditor::MapEditor()
{
    imgWidget = new WEImageWidget(&map, this);
    mapDialog = new WEMapDialog(this);
    setCentralWidget(imgWidget);
    createActions();
    createMenus();
    createToolBars();
    createStatusBar();
    locationLabel->setText("");
    readSettings();
    mapDialog->initDialog(names);

    changed = false;

    connect(imgWidget, SIGNAL(selectedRect(QRect &, WorkType &)), mapDialog, SLOT( showDialog(QRect &, WorkType &)));
    connect(imgWidget, SIGNAL(selectedRect(QRect&,WorkType&)), this, SLOT(eraseMap(QRect&,WorkType&)));
    connect(imgWidget, SIGNAL(selectedRect(QRect&,WorkType&)), this, SLOT(resetMap(QRect&,WorkType&)));
    connect(imgWidget, SIGNAL(selectedLine(QLine&, WorkType &,QVector<RoomRect>&)), mapDialog, SLOT(showDialog(QLine&, WorkType &,QVector<RoomRect>&)));
    connect(imgWidget, SIGNAL(selectedLine(QLine&,WorkType&,QVector<RoomRect>&)),this, SLOT(penMap(QLine&,WorkType&,QVector<RoomRect>&)));
    connect(imgWidget, SIGNAL(showTex(QString)), this, SLOT(showTex(QString)));

    connect(mapDialog, SIGNAL(saveRect(QString&,QString&,QRect&)), this, SLOT(saveAnEntity(QString&,QString&,QRect&)));
    connect(mapDialog, SIGNAL(saveLine(QString&,QString&,QLine&)), this, SLOT(saveAnEntity(QString&,QString&,QLine&)));

    connect(this, SIGNAL(setWorkType(WorkType&)), imgWidget, SLOT(setWorkType(WorkType&)));

    connect(this, SIGNAL(addRect(QRect&,QString &,QString &)), imgWidget,SLOT(addRect(QRect&,QString &,QString &)));
    connect(this, SIGNAL(addLine(QLine&,QString &,QString &)), imgWidget,SLOT(addLine(QLine&,QString &,QString &)));
    connect(this,SIGNAL(loadNewTopoFile(std::vector<Entity>)),imgWidget,SLOT(loadNewTopoFile(std::vector<Entity>)));

    // imgWidget->LoadImageFile(QPixmap(":/images/b.jpg"));
}

MapEditor::~MapEditor()
{
    delete imgWidget;
    delete mapDialog;
    delete fileToolBar;
    delete workTypeToolBar;
    delete fileMenu;
    delete openAction;
    delete openTopoAction;
    delete saveAction;
    delete saveBAction;
    delete saveAsAction;
    delete exitAction;
    delete workTypeAction;
    delete locationLabel;

    delete roomAction;
    delete doorAction;
    delete passwayAction;
    delete furnitureAction;
    delete addLineAction;
    delete eraserAction;
    delete resetAction;
    delete getThePictureAction;

}

void MapEditor::createActions()
{
    openAction = new QAction(tr("&Open"),this);
    openAction->setIcon(QIcon(":/images/open.png"));
    openAction->setShortcut(QKeySequence::Open);
    openAction->setStatusTip(tr("Open an existing map file"));
    connect(openAction, SIGNAL(triggered()), this, SLOT(open()));


    openTopoAction = new QAction(tr("O&pen"),this);
    openTopoAction->setIcon(QIcon(":/images/open2.png"));
    openTopoAction->setStatusTip(tr("Open an existing topo map file"));
    openTopoAction->setDisabled(true);
    connect(openTopoAction, SIGNAL(triggered()), this, SLOT(openTopoFile()));

    saveAction = new QAction(tr("&Save"), this);
    saveAction->setIcon(QIcon(":/images/save.png"));
    saveAction->setShortcut(QKeySequence::Save);
    saveAction->setStatusTip(tr("Save the topo map to disk"));
    connect(saveAction, SIGNAL(triggered()), this, SLOT(save()));

    saveBAction = new QAction(tr("Save &Binary map"), this);
    saveBAction->setIcon(QIcon(":/images/save_2.png"));
    saveBAction->setStatusTip(tr("Save the binary map to disk"));
    connect(saveBAction, SIGNAL(triggered()), this, SLOT(saveBinaryMap()));

    exitAction = new QAction(tr("E&xit"), this);
    exitAction->setShortcut(tr("Alt+F4"));
    exitAction->setStatusTip(tr("Exit the application"));
    connect(exitAction, SIGNAL(triggered()), this, SLOT(close()));

    saveAsAction = new QAction(tr("Save &As..."), this);
    saveAsAction->setStatusTip(tr("Save the topo map under a new name"));
    connect(saveAsAction, SIGNAL(triggered()), this, SLOT(saveAs()));

    roomAction = new QAction(tr("Room"),this);
    roomAction->setCheckable(true);
    roomAction->setChecked(true);

    doorAction = new QAction(tr("Door"),this);
    doorAction->setCheckable(true);

    passwayAction = new QAction(tr("Way"),this);
    passwayAction->setCheckable(true);

    furnitureAction= new QAction(tr("Furni"),this);
    furnitureAction->setCheckable(true);

    addLineAction= new QAction(tr("Pen"),this);
    addLineAction->setCheckable(true);

    eraserAction= new QAction(tr("Eraser"),this);
    eraserAction->setCheckable(true);

    resetAction= new QAction(tr("Reset"),this);
    resetAction->setCheckable(true);

    getThePictureAction=new QAction(tr("PrintScreen"),this);
    getThePictureAction->setCheckable(true);



    workTypeAction = new QActionGroup(this);
    workTypeAction->addAction(roomAction);
    workTypeAction->addAction(doorAction);
    workTypeAction->addAction(passwayAction);
    workTypeAction->addAction(furnitureAction);
    workTypeAction->addAction(addLineAction);
    workTypeAction->addAction(eraserAction);
    workTypeAction->addAction(resetAction);
    workTypeAction->addAction(getThePictureAction);
    connect(workTypeAction, SIGNAL(triggered(QAction*)),this, SLOT(workTypeChanged(QAction*)));

}

void MapEditor::createMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(openAction);
    fileMenu->addAction(openTopoAction);
    fileMenu->addAction(saveAction);
    fileMenu->addAction(saveAsAction);
    fileMenu->addAction(saveBAction);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAction);
}

void MapEditor::createToolBars()
{
    fileToolBar = addToolBar(tr("&File"));
    fileToolBar->addAction(openAction);
    fileToolBar->addAction(openTopoAction);
    fileToolBar->addAction(saveAction);
    fileToolBar->addAction(saveBAction);

    workTypeToolBar = addToolBar(tr("&Work Type"));
    workTypeToolBar->addActions(workTypeAction->actions());
}

void MapEditor::createStatusBar()
{
    locationLabel = new QLabel;
    locationLabel->setIndent(3);
    statusBar()->addWidget(locationLabel, 1);
}

void MapEditor::open()
{
    if (okToContinue()) {
        BmapFileName = QFileDialog::getOpenFileName(this,
                                                tr("Open Map Files"), ".",
                                                tr("Map files (*.map)"));
        if (!BmapFileName.isEmpty())
            map.load((const char *)(BmapFileName.toAscii().data()));
    }
    statusBar()->showMessage(tr("Open file:%1").arg(BmapFileName), 1000);
    QImage temp;
    map.convertoImage(temp);


    imgWidget->LoadImageFile(temp);
    topomap.clear();
    curFile.clear();
    openTopoAction->setEnabled(true);
    return;
}


void MapEditor::openTopoFile()
{

    if(okToContinue())
    {
        QString fileName = QFileDialog::getOpenFileName(this,
                                                        tr("Open Topo Map"), ".",
                                                        tr("Topo Map files (*.txt)"));
        if(fileName.isEmpty())
            return;

        curFile = fileName;
        topomap.clear();

        topomap.load_entities(curFile.toAscii().data());

        std::vector <Entity> copy_entities=topomap.m_entities;


        for(int i=0;i<copy_entities.size();i++)
        {
            Entity &temp=copy_entities[i];

            qDebug(temp.class_id.c_str());
            qDebug(temp.entity_name.c_str());

            gslam::Point p1(temp.x1,temp.y1);
            gslam::Point p2(temp.x2,temp.y2);

            qDebug("%f %f %f %f",p1.x,p1.y,p2.x,p2.y);

            gslam::IntPoint point1 = map.world2map(p1);
            gslam::IntPoint point2 = map.world2map(p2);


            qDebug("%d %d %d %d",point1.x,point1.y,point2.x,point2.y);

            temp.x1=point1.x;
            temp.x2=point2.x;
            temp.y1=point1.y;
            temp.y2=point2.y;



        }


        emit loadNewTopoFile(copy_entities);





    }



}
bool MapEditor::save()
{
    if (curFile.isEmpty())
    {
        return saveAs();
    } else
    {

        changed = !topomap.save_entities(curFile.toAscii().data());
        return !changed;
    }
}

bool MapEditor::saveAs()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save Topo Map"), ".",
                                                    tr("Topo Map files (*.txt)"));
    if (fileName.isEmpty())
        return false;

    curFile = fileName;
    changed = !topomap.save_entities(fileName.toAscii().data());
    return !changed;
}

bool MapEditor::saveBinaryMap()
{
    QString file =BmapFileName.mid(0, BmapFileName.lastIndexOf('.'));
    file = file + QString("_cleanup.map");
    map.save(file.toAscii().data());
    statusBar()->showMessage(tr("Save map:%1").arg(file), 1000);
    return true;
}

bool MapEditor::okToContinue()
{
    if (changed) {
        int r = QMessageBox::warning(this, tr("We Map Editor"),
                                     tr("The topo map  has been modified.\n"
                                        "Do you want to save your changes?"),
                                     QMessageBox::Yes | QMessageBox::No
                                     | QMessageBox::Cancel);
        if (r == QMessageBox::Yes) {
            return save();
        } else if (r == QMessageBox::Cancel) {
            return false;
        }
    }
    return true;
}

void MapEditor::showTex(QString tex)
{
    locationLabel->setText(tex);

}

void MapEditor::saveAnEntity(QString &type, QString &name, QRect &rect)
{

    gslam::IntPoint p1(rect.topLeft().x(),rect.topLeft().y());
    gslam::IntPoint p2(rect.bottomRight().x(),rect.bottomRight().y());
    gslam::Point point1 = map.map2world(p1);
    gslam::Point point2 = map.map2world(p2);

    topomap.addAnEntity(type.toAscii().data(),name.toAscii().data(),point1.x,point1.y,point2.x,point2.y);
    locationLabel->setText(tr(" Save entity[%1.%2] at pose [%3, %4] - [%5, %6]").arg(type).arg(name).arg(rect.topLeft().x()).arg(rect.topLeft().y()).arg(rect.bottomRight().x()).arg(rect.bottomRight().y()));
    changed = true;
    emit addRect(rect,type,name);


}
void MapEditor::saveAnEntity(QString &type, QString &name, QLine &line)
{
    gslam::IntPoint p1(line.x1(),line.y1());
    gslam::IntPoint p2(line.x2(),line.y2());
    gslam::Point point1 = map.map2world(p1);
    gslam::Point point2 = map.map2world(p2);

    topomap.addAnEntity(type.toAscii().data(),name.toAscii().data(),point1.x,point1.y,point2.x,point2.y);
    locationLabel->setText(tr(" Save entity[%1.%2] at pose [%3, %4] - [%5, %6]").arg(type).arg(name).arg(line.x1()).arg(line.y1()).arg(line.x2()).arg(line.y2()));
    changed = true;
    emit addLine(line,type,name);
    return;

}


void MapEditor::closeEvent(QCloseEvent *event)
{
    if (okToContinue()) {
        writeSettings();
        event->accept();
    } else {
        event->ignore();
    }
}

bool MapEditor::readSettings()
{
    QFile file("WEMapEditor.ini");
    file.open(QFile::ReadOnly);
    QTextStream ss(&file);
    QString line;
    QString label;

    while(!ss.atEnd())
    {
        line = ss.readLine();
        if(line.startsWith(' '))
        {
            for(int i =0 ; i < line.size(); i++)
            {
                if(line[i] == ' ')
                {
                    line.remove(i,1);
                    i = i-1;
                }
                else
                {
                    break;
                }
            }
        }

        if(line.endsWith(' '))
        {
            for(int i = line.size() -1; i>=0; i--)
            {
                if(line[i] == ' ')
                {
                    line.remove(i,1);
                }
                else
                {
                    break;
                }
            }
        }

        if(line.isEmpty() || line.startsWith('#'))
            continue;

        if(line.startsWith('['))
        {
            label = line.mid(1,line.indexOf(']') - 1);
            continue;
        }
        else if(!label.isEmpty())
        {
          //  if(!names[label].contains(line))
                  names[label].push_back(line);
        }
    }

    return true;
}


bool MapEditor::writeSettings()
{
    QFile file("WEMapEditor_back.ini");
    if(file.open(QFile::WriteOnly | QFile::Truncate))
    {
        QTextStream ss(&file);
        QList<QString> keys = names.keys();
        for(int i = 0; i < keys.size(); i++)
        {
            QVector<QString> &vals = names[keys[i]];
            ss<<tr("[%1]\n").arg(keys[i]);
            for(int j = 0; j < vals.size(); j++)
            {
                ss<<vals[j]<<'\n';
            }
        }
        ss.flush();
    }
    file.close();
    return true;
}

void MapEditor::workTypeChanged(QAction *val)
{
    WorkType type;
    if(val == roomAction)
        type = ROOM;
    else if(val == doorAction)
        type = DOOR;
    else if(val == passwayAction)
        type = PASSWAY;
    else if(val == furnitureAction)
        type = FURNITURE;
    else if(val == addLineAction)
        type = ADD_LINE;
    else if(val == eraserAction)
        type = ERASER;
    else if(val == resetAction)
        type = RESET;
    else if(val==getThePictureAction)
        type=PrintScreen;
    emit setWorkType(type);

    locationLabel->setText(tr("%1").arg(type));

}

void MapEditor::eraseMap(QRect &rect, WorkType &type)
{
    if(type != ERASER)
        return;

    int xstart = rect.topLeft().x();
    int ystart = rect.topLeft().y();
    int xend = rect.bottomRight().x();
    int yend = rect.bottomRight().y();

    for(int y = ystart; y <=yend; y++)
    {
        for(int x = xstart; x <= xend; x++)
        {
            if(map.isInside(x, y))
            {
                gslam::GridMapBase::Cell & cell = map.cell(x,y);
                cell.n = 0;
                if(cell.visits == 0)
                    cell.visits = 100;
            }

        }
    }
    QImage temp;
    map.convertoImage(temp);
    imgWidget->UpdateImageFile(temp);
}

void MapEditor::resetMap(QRect &rect, WorkType &type)
{
    if(type != RESET)
        return;
    int xstart = rect.topLeft().x();
    int ystart = rect.topLeft().y();
    int xend = rect.bottomRight().x();
    int yend = rect.bottomRight().y();

    for(int y = ystart; y <=yend; y++)
    {
        for(int x = xstart; x <= xend; x++)
        {
            if(map.isInside(x, y))
            {
                gslam::GridMapBase::Cell & cell = map.cell(x,y);
                cell.n = 0;
                cell.visits = 0;
            }

        }
    }
    QImage temp;
    map.convertoImage(temp);
    imgWidget->UpdateImageFile(temp);
}

#include "gs_util.h"
void MapEditor::penMap(QLine &l, WorkType &type,QVector<RoomRect>&rooms)
{
    if(type != ADD_LINE)
        return;

    int xstart = l.p1().x();
    int ystart = l.p1().y();
    int xend = l.p2().x();
    int yend = l.p2().y();
    std::vector<gpose> line;
    _BresenhamLine(xstart, ystart, xend, yend, line);
    bool isx = true;
    if(abs(yend - ystart) > abs(xend - xstart))
        isx = false;

    for(size_t i = 0; i < line.size(); i++)
    {
        if(map.isInside(line[i].x, line[i].y))
        {
            gslam::GridMapBase::Cell &c= map.cell(line[i].x, line[i].y);
            if(!c.visits)
                c.n = c.visits = 100;
            else
                c.n = c.visits;
        }

        if(isx)
        {
            if(map.isInside(line[i].x, line[i].y - 1))
            {
                gslam::GridMapBase::Cell &c= map.cell(line[i].x, line[i].y - 1);
                if(!c.visits)
                    c.n = c.visits = 100;
                else
                    c.n = c.visits;
            }
            if(map.isInside(line[i].x, line[i].y + 1))
            {
                gslam::GridMapBase::Cell &c= map.cell(line[i].x, line[i].y + 1);
                if(!c.visits)
                    c.n = c.visits = 100;
                else
                    c.n = c.visits;
            }
        }
        else
        {
            if(map.isInside(line[i].x - 1, line[i].y))
            {
                gslam::GridMapBase::Cell &c= map.cell(line[i].x - 1, line[i].y);
                if(!c.visits)
                    c.n = c.visits = 100;
                else
                    c.n = c.visits;
            }
            if(map.isInside(line[i].x + 1, line[i].y))
            {
                gslam::GridMapBase::Cell &c= map.cell(line[i].x + 1, line[i].y);
                if(!c.visits)
                    c.n = c.visits = 100;
                else
                    c.n = c.visits;
            }
        }
    }
    QImage temp;
    map.convertoImage(temp);
    imgWidget->UpdateImageFile(temp);
}
































